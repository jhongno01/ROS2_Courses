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
    // 구독 콜백 안에서 서비스 응답을 기다릴 것이므로, 같은 노드 안의 콜백들이 서로 막히지 않도록 Reentrant 콜백 그룹을 따로 만든다. 
      // 서로 다른 콜백 그룹 끼리는 항상 병렬 실행이 가능하다. (멀티스레드 병렬 실행은 ROS 2의 핵심 강점 중 하나)
      // 콜백 그룹을 지정하지 않으면, 노드 전체가 하나의 콜백 그룹에 속하게 되는데, 이 경우에는 같은 노드 내의 모든 콜백이 서로 막히는 Mutually Exclusive 그룹이 된다.
      // Reentrant 콜백 그룹은 같은 노드 내에서 여러 콜백이 동시에 실행될 수 있도록 허용하는 콜백 그룹 유형이다. 
      // 이를 사용하면, 한 콜백이 서비스 응답을 기다리는 동안 다른 콜백이 실행될 수 있다.
    callback_group_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);

    // 퍼블리셔가 보낸 a, b 값을 그대로 서비스 서버에 전달하기 위한 클라이언트.
    // 서비스 이름은 hw01_srv.cpp와 동일해야 연결된다.
    client_ = this->create_client<more_interfaces::srv::TimesAddTwoInts>(
      "times_add_two_ints",  // 서비스 이름은 hw01_srv.cpp와 동일해야 연결된다.
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
    // 서버가 실행되기 전 먼저 실행되었을 수 있으므로, 1초 단위로 계속 확인하며 터미널에 메시지를 띄운다.
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
      // client_->async_send_request(request)는 서비스 요청을 비동기(응답이 안와도 바로 리턴) 서비스 서버로 보내고, std::future 객체를 반환한다. 
      // 객체 future은 추후에 서비스 응답이 준비되었는지 확인하고, 응답 데이터를 가져오는 데 사용된다.
      // **C++ 표준 라이브러리의 비동기 기능 사용. ROS2의 비동기 기능[rclcpp::spin_until_future_complete(node, future)] 로도 구현이 가능하다.
    auto future = client_->async_send_request(request);
    while (rclcpp::ok()) {
      const auto status = future.wait_for(100ms);  // 100ms동안 응답이 준비되었는지 확인한다. wait_for는 std::future의 멤버 함수이다.
      if (status == std::future_status::ready) {
        // 서비스 응답이 준비되었다면, 결과를 가져와서 반환한다. future.get()은 서비스 응답이 준비될 때까지 blocking하며, 준비된 응답 데이터를 반환한다.
          // 즉, get()만 사용해도 응답은 받을 수 있지만, 얼마나 기다릴지 제어가 어렵기에, wait_for()로 상태를 확인하면 로그를 찍거나 하여 추가 대응이 가능하다.'
          // 추가로, get()는 한 번만 호출할 수 있다. 여러 번 호출하면 std::future_error 예외가 발생한다.
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
