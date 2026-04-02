#include <cstdio>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "more_interfaces/msg/hw02_numbers.hpp"


class LidarAnglePublisher : public rclcpp::Node // rclcpp::Node 클래스를 상속받아 Hw01Pub 클래스를 정의한다. 이 클래스는 ROS 2 노드로서, 퍼블리셔를 포함한다.
{
public:
  LidarAnglePublisher()
  : Node("lidar_angle_publisher"), a_(0), b_(0), c_(0) // 노드의 이름이 "lidar_angle_publisher"로 설정되고, a_, b_가 0으로 초기화된다.
  {
    publisher_ = this->create_publisher<more_interfaces::msg::Hw02Numbers>("lidar_angle", 10);
  }

  bool read_input()
  {
    while (rclcpp::ok()) {
      std::printf("Please type three angles to monitor: ");
      std::fflush(stdout);  // 출력 버퍼를 즉시 비워 printf로 출력한 내용이 바로 터미널 화면에 출력되도록 한다.

      int ret = std::scanf("%d %d %d", &a_, &b_, &c_); // scanf는 사용자로부터 입력을 받아 a_, b_, c_에 저장한다. scanf는 성공적으로 읽은 항목의 수를 반환한다.

      if (ret == 3) { // 세 개의 정수가 성공적으로 읽혔을 때
        auto message_ = more_interfaces::msg::Hw02Numbers();
        message_.a = a_;
        message_.b = b_;
        message_.c = c_;

        RCLCPP_INFO_STREAM(this->get_logger(),
                          "Publishing lidar monitor angles: " 
                            << message_.a << ", " << message_.b << ", " << message_.c);

        publisher_->publish(message_);
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
  rclcpp::Publisher<more_interfaces::msg::Hw02Numbers>::SharedPtr publisher_;  // ShardPtr은 C++의 std::shared_ptr와 유사한 스마트 포인터로, ROS2에서 정의되었다. 따라서 std::make_shared로 호출이 
  int a_, b_, c_;
};



int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<LidarAnglePublisher>();  // make_shared 함수는 LidarAnglePublisher 클래스의 객체를 생성하고, 클래스 내부의 생성자를 호출한다. make_shaared의 인자는 LidarAnglePublisher 클래스의 생성자에 인자로 전달된다.

  while (rclcpp::ok()) {  // rclcpp::ok()는 ROS 2 시스템이 정상적으로 작동 중인지 확인하는 함수로, 노드의 종료 여부를 체크한다. 예를 들어, Ctrl+C로 노드를 종료하면 rclcpp::ok()는 false를 반환하여 루프가 종료된다.
    if (!node->read_input()) {
      break;
    }
  }
  rclcpp::shutdown();
  return 0;
}
