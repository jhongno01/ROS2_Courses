#include <chrono>
#include <cstdio>
#include <memory>

#include "more_interfaces/msg/hw01_numbers.hpp"
#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;

class Hw01Pub : public rclcpp::Node // rclcpp::Node 클래스를 상속받아 Hw01Pub 클래스를 정의합니다. 이 클래스는 ROS 2 노드로서, 퍼블리셔와 타이머를 포함한다.
{
public:
  Hw01Pub()
  : Node("hw01_pub"), a_(0), b_(0), count_(0) // 노드의 이름이 "hw01_pub"로 설정되고, a_, b_, count_가 0으로 초기화된다.
  {
    publisher_ = this->create_publisher<more_interfaces::msg::TwoInts_Iteration_count>("topic_pub_hw01", 10);
  }

private: 
  bool read_input()
  {
    std::printf("Please type two int: ");
    std::fflush(stdout);

    if (std::scanf("%d %d", &a_, &b_) != 2) {
      RCLCPP_ERROR(this->get_logger(), "Failed to read two integers from stdin.");
      return false;
    }

    auto message_hw01 = more_interfaces::msg::TwoInts_Iteration_count();
    message_hw01.a = a_;
    message_hw01.b = b_;
    message_hw01.iteration_count = ++count_; // 입력이 성공적으로 읽혔을 때 iteration 카운터를 증가시킵니다.

    RCLCPP_INFO(this->get_logger(), "Publishing: '%d %d %d'", message_hw01.a, message_hw01.b, message_hw01.iteration_count);

    publisher_->publish(message_hw01);
    return true;
  }

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<Hw01Pub>();

  if (!node->read_input()) {
    rclcpp::shutdown();
    return 1;
  }

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
