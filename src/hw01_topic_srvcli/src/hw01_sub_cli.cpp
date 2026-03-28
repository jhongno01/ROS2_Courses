#include <chrono>
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
    client_ = this->create_client<more_interfaces::srv::TimesAddTwoInts>("times_add_two_ints");
    subscription_ = this->create_subscription<more_interfaces::msg::TwoIntsIterationCount>(
      "topic_hw01",
      10,
      std::bind(&Hw01SubCli::topic_callback, this, _1));
  }

private:
  int64_t request_product(int64_t left, int64_t right) const
  {
    while (!client_->wait_for_service(1s)) {
      if (!rclcpp::ok()) {
        RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service.");
        return 0;
      }
      RCLCPP_INFO(this->get_logger(), "Service not available, waiting again...");
    }

    auto request = std::make_shared<more_interfaces::srv::TimesAddTwoInts::Request>();
    request->a = left;
    request->b = right;

    auto future = client_->async_send_request(request);
    while (rclcpp::ok()) {
      const auto status = future.wait_for(100ms);
      if (status == std::future_status::ready) {
        return future.get()->times_sum;
      }
    }

    RCLCPP_ERROR(this->get_logger(), "Failed to call service times_add_two_ints.");
    return 0;
  }

  void topic_callback(const more_interfaces::msg::TwoIntsIterationCount::SharedPtr msg) const
  {
    const auto result1 = request_product(msg->count, msg->a);
    const auto result2 = request_product(msg->count, msg->b);

    RCLCPP_INFO(
      this->get_logger(),
      "Hello world %ld * %ld = %ld, %ld * %ld = %ld",
      msg->count,
      msg->a,
      result1,
      msg->count,
      msg->b,
      result2);
  }

  rclcpp::Client<more_interfaces::srv::TimesAddTwoInts>::SharedPtr client_;
  rclcpp::Subscription<more_interfaces::msg::TwoIntsIterationCount>::SharedPtr subscription_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<Hw01SubCli>();
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
