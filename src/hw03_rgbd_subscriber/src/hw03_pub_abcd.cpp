#include <cstdio>
#include <memory>

#include "more_interfaces/msg/hw03_abcd.hpp"
#include "rclcpp/rclcpp.hpp"

class AbcdPublisher : public rclcpp::Node
{
public:
  AbcdPublisher() : Node("abcd_publisher"), a_(0), b_(0), c_(0), d_(0)
  {
    publisher_ = this->create_publisher<more_interfaces::msg::Hw03Abcd>("abcd", 10);
  }

  bool read_input()
  {
    while (rclcpp::ok()) {
      std::printf("Please type four integers A, B, C, D: ");
      std::fflush(stdout);

      const int ret = std::scanf("%d %d %d %d", &a_, &b_, &c_, &d_);

      if (ret == 4) {
        auto message = more_interfaces::msg::Hw03Abcd();
        message.a = a_;
        message.b = b_;
        message.c = c_;
        message.d = d_;

        RCLCPP_INFO_STREAM(
          this->get_logger(),
          "Publishing abcd: " << message.a << ", " << message.b << ", " << message.c << ", " << message.d);

        publisher_->publish(message);
        return true;
      }

      if (ret == EOF) {
        return false;
      }

      RCLCPP_ERROR(this->get_logger(), "Invalid input. Please enter four integers.");

      int ch;
      while ((ch = std::getchar()) != '\n' && ch != EOF) {
      }
    }
    return false;
  }

private:
  rclcpp::Publisher<more_interfaces::msg::Hw03Abcd>::SharedPtr publisher_;
  int a_;
  int b_;
  int c_;
  int d_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<AbcdPublisher>();

  while (rclcpp::ok()) {
    if (!node->read_input()) {
      break;
    }
  }

  rclcpp::shutdown();
  return 0;
}
