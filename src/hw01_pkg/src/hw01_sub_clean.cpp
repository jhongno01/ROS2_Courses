#include <memory>

#include "more_interfaces/msg/hw01_numbers.hpp"
#include "rclcpp/rclcpp.hpp"

using std::placeholders::_1;

class Hw01SubClean : public rclcpp::Node
{
public:
  Hw01SubClean()
  : Node("hw01_sub_clean")
  {
    subscription_ = this->create_subscription<more_interfaces::msg::Hw01Numbers>(
      "topic_hw01_clean",
      10,
      std::bind(&Hw01SubClean::topic_callback, this, _1));
  }

private:
  void topic_callback(const more_interfaces::msg::Hw01Numbers::SharedPtr msg) const
  {
    const auto result1 = msg->count * msg->int1;
    const auto result2 = msg->count * msg->int2;

    RCLCPP_INFO(
      this->get_logger(),
      "Hello world %d * %d = %d, %d * %d = %d",
      msg->count,
      msg->int1,
      result1,
      msg->count,
      msg->int2,
      result2);
  }

  rclcpp::Subscription<more_interfaces::msg::Hw01Numbers>::SharedPtr subscription_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<Hw01SubClean>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
