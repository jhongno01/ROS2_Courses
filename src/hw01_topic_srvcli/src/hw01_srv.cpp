#include <memory>

#include "more_interfaces/srv/times_add_two_ints.hpp"
#include "rclcpp/rclcpp.hpp"

void multiply(
  const std::shared_ptr<more_interfaces::srv::TimesAddTwoInts::Request> request,
  std::shared_ptr<more_interfaces::srv::TimesAddTwoInts::Response> response)
{
  response->times_sum = request->a * request->b;

  RCLCPP_INFO(
    rclcpp::get_logger("rclcpp"),
    "Incoming request a=%ld, b=%ld",
    request->a,
    request->b);
  RCLCPP_INFO(
    rclcpp::get_logger("rclcpp"),
    "Sending back response: [%ld]",
    response->times_sum);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto node = rclcpp::Node::make_shared("hw01_srv");
  auto service = node->create_service<more_interfaces::srv::TimesAddTwoInts>(
    "times_add_two_ints",
    &multiply);
  (void)service;

  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Ready to multiply two ints.");
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
