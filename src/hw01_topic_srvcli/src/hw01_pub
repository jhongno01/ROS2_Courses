#include <chrono>
#include <cstdio>
#include <memory>

#include "more_interfaces/msg/hw01_numbers.hpp"
#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;

class Hw01PubClean : public rclcpp::Node
{
public:
  Hw01PubClean()
  : Node("hw01_pub_clean"), int1_(0), int2_(0), count_(0)
  {
    publisher_ = this->create_publisher<more_interfaces::msg::Hw01Numbers>("topic_hw01_clean", 10);
    timer_ = this->create_wall_timer(500ms, std::bind(&Hw01PubClean::timer_callback, this));
  }

  bool read_input()
  {
    std::printf("Please type two int: ");
    std::fflush(stdout);

    if (std::scanf("%d %d", &int1_, &int2_) != 2) {
      RCLCPP_ERROR(this->get_logger(), "Failed to read two integers from stdin.");
      return false;
    }

    return true;
  }

private:
  void timer_callback()
  {
    ++count_;

    auto msg = more_interfaces::msg::Hw01Numbers();
    msg.int1 = int1_;
    msg.int2 = int2_;
    msg.count = count_;

    RCLCPP_INFO(
      this->get_logger(),
      "%d, %d ... hello world count %d",
      msg.int1,
      msg.int2,
      msg.count);

    publisher_->publish(msg);
  }

  int32_t int1_;
  int32_t int2_;
  int32_t count_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<more_interfaces::msg::Hw01Numbers>::SharedPtr publisher_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<Hw01PubClean>();

  if (!node->read_input()) {
    rclcpp::shutdown();
    return 1;
  }

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
