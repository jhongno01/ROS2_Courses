#include <chrono>
#include <cstdio>
#include <memory>

#include "more_interfaces/msg/two_ints_iteration_count.hpp"
#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;

class Hw01Pub : public rclcpp::Node // rclcpp::Node 클래스를 상속받아 Hw01Pub 클래스를 정의한다. 이 클래스는 ROS 2 노드로서, 퍼블리셔를 포함한다.
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
      std::fflush(stdout);  // 출력 버퍼를 즉시 비워 printf로 출력한 내용이 바로 터미널 화면에 출력되도록 한다.

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

      if (ret == EOF) { // End of file. 예를 들어, Ctrl+D (Unix) 또는 Ctrl+Z (Windows)를 누르면 EOF가 발생한다.
        return false;  // Ctrl+D 등으로 입력 종료시 루프가 종료된다.
      }

      RCLCPP_ERROR(this->get_logger(), "Invalid input. Please enter two integers.");

      int ch;
      while ((ch = std::getchar()) != '\n' && ch != EOF) {} // \n또는 EOF가 나올 떄 까지 입력 버퍼를 비워 실패 Error 무한루를 방지한다.
      // 입력 버퍼를 비우지 않으면, 잘못된 입력이 계속해서 scanf에 남아 있어서 다음 scanf 호출도 계속 실패하게 된다. 
    }
  return false;
}

private : 
  rclcpp::Publisher<more_interfaces::msg::TwoIntsIterationCount>::SharedPtr publisher_;  // ShardPtr은 C++의 std::shared_ptr와 유사한 스마트 포인터로, ROS2에서 정의되었다. 따라서 std::make_shared로 호출이 
  int a_, b_;
  size_t count_;
};



int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<Hw01Pub>();  // make_shared 함수는 Hw01Pub 클래스의 객체를 생성하고, 클래스 내부의 생성자를 호출한다. make_shaared의 인자는 Hw01Pub 클래스의 생성자에 인자로 전달된다.

  while (rclcpp::ok()) {  // rclcpp::ok()는 ROS 2 시스템이 정상적으로 작동 중인지 확인하는 함수로, 노드의 종료 여부를 체크한다. 예를 들어, Ctrl+C로 노드를 종료하면 rclcpp::ok()는 false를 반환하여 루프가 종료된다.
    if (!node->read_input()) {
      break;
    }
  }

  rclcpp::shutdown();
  return 0;
}
