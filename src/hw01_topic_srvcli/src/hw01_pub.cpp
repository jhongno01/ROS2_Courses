#include <chrono>
#include <cstdio>
#include <memory>

#include "more_interfaces/msg/two_ints_iteration_count.hpp"
#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;

class Hw01Pub : public rclcpp::Node // rclcpp::Node 클래스를 상속받아 Hw01Pub 클래스를 정의합니다. 이 클래스는 ROS 2 노드로서, 퍼블리셔를 포함한다.
{
public:
  Hw01Pub()
  : Node("hw01_pub"), a_(0), b_(0), count_(0) // 노드의 이름이 "hw01_pub"로 설정되고, a_, b_, count_가 0으로 초기화된다.
  {
    publisher_ = this->create_publisher<more_interfaces::msg::TwoIntsIterationCount>("topic_pub_hw01", 10);
  }

  bool read_input()
  {
    while (rclcpp::ok()) {
      std::printf("Please type two int: ");
      std::fflush(stdout);

      int ret = std::scanf("%d %d", &a_, &b_);

      if (ret == 2) {
        auto message_hw01 = more_interfaces::msg::TwoIntsIterationCount();
        message_hw01.a = a_;
        message_hw01.b = b_;
        message_hw01.count = ++count_;

        RCLCPP_INFO_STREAM(this->get_logger(),
                          "Publishing: " << message_hw01.a << " "
                                          << message_hw01.b << " "
                                          << message_hw01.count);

        publisher_->publish(message_hw01);
        return true;
      }

      if (ret == EOF) { // End of file. 예를 들어, Ctrl+D (Unix) 또는 Ctrl+Z (Windows)를 누르면 EOF 발생
        return false;  // Ctrl+D 등으로 입력 종료시 루프 종료
      }

      RCLCPP_ERROR(this->get_logger(), "Invalid input. Please enter two integers.");

      int ch;
      while ((ch = std::getchar()) != '\n' && ch != EOF) {} // \n또는 EOF가 나올 떄 까지 입력 버퍼를 비워 실패 Error 무한루프 방지
    }
  return false;
}

private : 
  rclcpp::Publisher<more_interfaces::msg::TwoIntsIterationCount>::SharedPtr publisher_;
  int a_, b_;
  size_t count_;
};



int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<Hw01Pub>();

  while (rclcpp::ok()) {
    if (!node->read_input()) {
      break;
    }
  }

  rclcpp::shutdown();
  return 0;
}
