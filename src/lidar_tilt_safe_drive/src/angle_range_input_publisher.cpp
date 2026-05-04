#include <cstdio>
#include <memory>

#include "more_interfaces/msg/lidar_angle_range.hpp"
#include "rclcpp/rclcpp.hpp"

class AngleRangeInputPublisher : public rclcpp::Node
{
public:
  AngleRangeInputPublisher()
      : Node("angle_range_input_publisher")
  {
    publisher_ = this->create_publisher<more_interfaces::msg::LidarAngleRange>(
        "monitor_angle_range", 10);
  }

  bool read_and_publish()
  {
    while (rclcpp::ok())
    {
      int start_deg = 0;
      int end_deg = 0;

      std::printf("Please type two lidar angles (range: 0~359): ");
      std::fflush(stdout);

      const int ret = std::scanf("%d %d", &start_deg, &end_deg);

      if (start_deg < 0 || start_deg > 359 || end_deg < 0 || end_deg > 359)
      {
        RCLCPP_WARN(this->get_logger(),
                    "Input angles should be in range of 0 to 359. You typed %d and %d.",
                    start_deg,
                    end_deg);
        continue;
      }

      if (ret == 2)
      {
        more_interfaces::msg::LidarAngleRange message;
        message.start_deg = start_deg;
        message.end_deg = end_deg;
        publisher_->publish(message);

        RCLCPP_INFO(
            this->get_logger(),
            "Monitoring angle range updated to %d deg -> %d deg",
            start_deg,
            end_deg);
        return true;
      }

      if (ret == EOF)
      {
        return false;
      }

      RCLCPP_ERROR(this->get_logger(), "Invalid input. Please type two integer angles.");
      clear_input_buffer();
    }

    return false;
  }

private:
  void clear_input_buffer()
  {
    int ch = 0;
    while ((ch = std::getchar()) != '\n' && ch != EOF)
    {
    }
  }

  rclcpp::Publisher<more_interfaces::msg::LidarAngleRange>::SharedPtr publisher_;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<AngleRangeInputPublisher>();

  while (rclcpp::ok())
  {
    if (!node->read_and_publish())
    {
      break;
    }
    rclcpp::spin_some(node);
  }

  rclcpp::shutdown();
  return 0;
}
