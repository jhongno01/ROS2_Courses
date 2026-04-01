#include <cstdio>
#include <functional>
#include <memory>

#include "more_interfaces/srv/times_add_two_ints.hpp"
#include "rclcpp/rclcpp.hpp"

class Hw01Srv : public rclcpp::Node
{
public:
  Hw01Srv()
  : Node("hw01_srv"), extra_input_(0)
  {
    read_extra_input();

    // 서비스 서버는 시작할 때 한 번 읽어 둔 extra_input_ 값을 계속 유지한 채,
    // 이후 들어오는 모든 요청에 대해 같은 규칙으로 계산한다.
    service_ = this->create_service<more_interfaces::srv::TimesAddTwoInts>(
      "times_add_two_ints",
      std::bind(&Hw01Srv::handle_request, this, std::placeholders::_1, std::placeholders::_2));
  }

private:
  void read_extra_input()
  {
    // 과제 조건:
    // 서버는 "처음 실행될 때" 정수 하나를 scanf로 입력받고,
    // 그 값을 이후 모든 서비스 계산에 더하는 고정값으로 사용한다.
    while (rclcpp::ok()) {
      std::printf("Please type one int for service server: ");
      std::fflush(stdout);

      const int ret = std::scanf("%ld", &extra_input_);
      if (ret == 1) {
        RCLCPP_INFO_STREAM(
          this->get_logger(),
          "Stored server input value: " << extra_input_);
        return;
      }

      if (ret == EOF) {
        RCLCPP_WARN(
          this->get_logger(),
          "EOF received while reading the initial server input. Using default value 0.");
        extra_input_ = 0;
        return;
      }

      RCLCPP_ERROR(this->get_logger(), "Invalid input. Please enter one integer.");

      // 잘못된 문자가 남아 있으면 다음 scanf도 계속 실패하므로,
      // 개행 또는 EOF가 나올 때까지 입력 버퍼를 비워서 상태를 복구한다.
      int ch;
      while ((ch = std::getchar()) != '\n' && ch != EOF) {}
    }
  }

  void handle_request(
    const std::shared_ptr<more_interfaces::srv::TimesAddTwoInts::Request> request,
    std::shared_ptr<more_interfaces::srv::TimesAddTwoInts::Response> response)
  {
    // 과제에서 요구한 서버 계산:
    // a * b + (서버가 시작할 때 입력받은 정수)
    response->times_sum = (request->a * request->b) + extra_input_;

    RCLCPP_INFO_STREAM(
      this->get_logger(),
      request->a << " * " << request->b << " + " << extra_input_
                 << " = " << response->times_sum);
  }

  long extra_input_;
  rclcpp::Service<more_interfaces::srv::TimesAddTwoInts>::SharedPtr service_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<Hw01Srv>();
  RCLCPP_INFO(node->get_logger(), "Ready to process times_add_two_ints requests.");
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
