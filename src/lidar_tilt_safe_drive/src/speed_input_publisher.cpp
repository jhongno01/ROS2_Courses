#include <cstdio>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"

class SpeedInputPublisher : public rclcpp::Node
{
public:
  SpeedInputPublisher()
  : Node("speed_input_publisher")
  {
    publisher_ = this->create_publisher<std_msgs::msg::Float32>("drive_speed", 10);
  }

  bool read_and_publish()
  {
    while (rclcpp::ok()) {
      float speed = 0.0f;

      std::printf("Please type target speed (0.0 ~ 1.0): ");
      std::fflush(stdout);

      const int ret = std::scanf("%f", &speed);
      if (ret == 1) {
        if (speed < 0.0f || speed > 1.0f) {
          RCLCPP_WARN(this->get_logger(), "Input %.3f is out of range. Please use 0.0 to 1.0.", speed);
          continue;
        }

        std_msgs::msg::Float32 message;
        message.data = speed;
        publisher_->publish(message);

        RCLCPP_INFO(this->get_logger(), "Requested speed updated to %.3f", speed);
        return true;
      }

      if (ret == EOF) {
        return false;
      }

      RCLCPP_ERROR(this->get_logger(), "Invalid input. Please type one floating-point value.");
      clear_input_buffer();
    }

    return false;
  }

private:
  void clear_input_buffer()
  {
    int ch = 0;
    while ((ch = std::getchar()) != '\n' && ch != EOF) {
    }
  }

  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr publisher_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<SpeedInputPublisher>();

  while (rclcpp::ok()) {
    if (!node->read_and_publish()) {
      break;
    }
    rclcpp::spin_some(node);  // 현재 처리 가능한 콜백만 처리하고 바로 retrun하여 사용자 입력을 계속 받을 수 있도록 함
    // scanf가 어차피 블로킹 하므로, publisher만 있을때는 spin_some이 필요 없지만, 향후 subscriber가 추가될 경우를 대비하여 spin_some을 호출하도록 함
  }

  rclcpp::shutdown();
  return 0;
}
