#include <chrono>
#include <functional>
#include <future>
#include <memory>

#include "more_interfaces/msg/two_ints_iteration_count.hpp"
#include "more_interfaces/srv/times_add_two_ints.hpp"
#include "rclcpp/rclcpp.hpp"

using std::placeholders::_1;
using namespace std::chrono_literals;

class Hw01SubCli : public rclcpp::Node
{
public:
  Hw01SubCli()
  : Node("hw01_sub_cli")
  {
    // 구독 콜백 안에서 서비스 응답을 기다릴 것이므로, 같은 노드 안의 콜백들이
    // 서로 막히지 않도록 Reentrant 콜백 그룹을 따로 만든다.
    callback_group_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);

    // 퍼블리셔가 보낸 a, b 값을 그대로 서비스 서버에 전달하기 위한 클라이언트.
    // 서비스 이름은 hw01_srv.cpp와 동일해야 연결된다.
    client_ = this->create_client<more_interfaces::srv::TimesAddTwoInts>(
      "times_add_two_ints",
      rmw_qos_profile_services_default,
      callback_group_);

    // 구독 콜백도 같은 Reentrant 그룹에 넣어서, 토픽 수신 중에도 서비스 응답 처리가 가능하게 한다.
    rclcpp::SubscriptionOptions subscription_options;
    subscription_options.callback_group = callback_group_;

    subscription_ = this->create_subscription<more_interfaces::msg::TwoIntsIterationCount>(
      // 퍼블리셔 코드에서 실제로 publish 하는 토픽 이름을 그대로 사용한다.
      "topic_pub_hw01",
      10,
      std::bind(&Hw01SubCli::topic_callback, this, _1),
      subscription_options);
  }

private:
  int64_t request_server_result(int64_t left, int64_t right) const
  {
    // 서버가 아직 실행되지 않았을 수 있으므로, 1초 단위로 계속 확인한다.
    while (!client_->wait_for_service(1s)) {
      if (!rclcpp::ok()) {
        RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service.");
        return 0;
      }
      RCLCPP_INFO(this->get_logger(), "Service not available, waiting again...");
    }

    // 퍼블리셔가 보낸 두 정수를 서비스 요청에 그대로 넣는다.
    auto request = std::make_shared<more_interfaces::srv::TimesAddTwoInts::Request>();
    request->a = left;
    request->b = right;

    // 비동기 요청을 보내고, 아래 루프에서 응답이 준비될 때까지 기다린다.
    auto future = client_->async_send_request(request);
    while (rclcpp::ok()) {
      const auto status = future.wait_for(100ms);
      if (status == std::future_status::ready) {
        // TimesAddTwoInts.srv의 응답 필드 이름은 times_sum이지만,
        // 이번 과제에서는 "a * b + 서버 시작 시 입력한 값" 결과를 담아 사용한다.
        return future.get()->times_sum;
      }
    }

    RCLCPP_ERROR(this->get_logger(), "Failed to call service times_add_two_ints.");
    return 0;
  }

  void topic_callback(const more_interfaces::msg::TwoIntsIterationCount::SharedPtr msg) const
  {
    // 첫 번째 출력:
    // 토픽으로 받은 iteration, a, b를 사용해 로컬에서 a + b를 계산해 표시한다.
    const int64_t local_sum = msg->a + msg->b;
    RCLCPP_INFO_STREAM(
      this->get_logger(),
      "iteration " << msg->count << ": "
                   << msg->a << " + " << msg->b << " = " << local_sum);

    // 두 번째 단계:
    // 같은 a, b를 서비스 서버로 보내고, 서버가 계산한 결과를 받아서 출력한다.
    const int64_t server_result = request_server_result(msg->a, msg->b);
    RCLCPP_INFO_STREAM(
      this->get_logger(),
      "service response: " << server_result);
  }

  rclcpp::CallbackGroup::SharedPtr callback_group_;
  rclcpp::Client<more_interfaces::srv::TimesAddTwoInts>::SharedPtr client_;
  rclcpp::Subscription<more_interfaces::msg::TwoIntsIterationCount>::SharedPtr subscription_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<Hw01SubCli>();

  // 구독 콜백이 서비스 응답을 기다리는 동안, 다른 스레드가 서비스 응답 이벤트를 처리할 수 있게
  // 2개의 작업 스레드를 가진 MultiThreadedExecutor를 사용한다.
  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 2);
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
