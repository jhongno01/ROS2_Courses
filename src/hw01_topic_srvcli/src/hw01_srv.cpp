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
    // 서비스 서버는 시작할 때 한 번 읽어 둔 extra_input_ 값을 계속 유지한 채, 이후 들어오는 모든 요청에 대해 같은 규칙으로 계산한다.
    service_ = this->create_service<more_interfaces::srv::TimesAddTwoInts>(
      "times_add_two_ints", // 서비스 이름은 hw01_sub_cli.cpp에서 클라이언트가 요청할 때 사용하는 이름과 동일해야 연결된다.
      std::bind(&Hw01Srv::handle_request, this, std::placeholders::_1, std::placeholders::_2));
      // std::bind을 사용하여 서비스 요청이 들어올 때마다 handle_request 멤버 함수를 호출하도록 설정한다.
        // 현재 객체(this)의 handle_request 함수를 콜백으로 등록하는데, 서비스 요청과 응답 객체는 ROS2가 넘겨주는 _1, _2로 사용한다.
  }

private:
  void read_extra_input()
  {
    // 서버는 최초 실행 시 정수 하나를 scanf로 입력받고,
    // 그 값을 이후 모든 서비스 클라이언트에게 받는 값 간의 곱에 더하는 고정값으로 사용한다.
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

      // 잘못된 문자가 남아 있으면 다음 scanf도 계속 실패하므로, 개행문자 \n 또는 EOF가 나올 때까지 입력 버퍼를 비운다.
      int ch;
      while ((ch = std::getchar()) != '\n' && ch != EOF) {}
    }
  }

  void handle_request(  // 서비스 이름이 같은 hw01_sub_cli.cpp의 클라이언트가 요청할 때, request와 response 객체는 ROS2가 자동으로 생성하여 넘겨준다. 
    //  따라서, 서비스 서버는 이 함수에서 request 객체의 a, b 필드를 읽어서 계산한 후, response 객체의 빈 times_sum 필드에 결과를 넣어주기만 하면 된다.
    const std::shared_ptr<more_interfaces::srv::TimesAddTwoInts::Request> request,
    std::shared_ptr<more_interfaces::srv::TimesAddTwoInts::Response> response)
  {
    // 서비스 서버 계산: a * b + (extra_input_으로 시작할 때 입력받은 정수)
    // 아래 줄이 서비스의 응답을 만들어주는 부분이 된다.
      // 서비스는 토픽과 달리, 직접 발행하지 않아도 .srv 파일에 지정된 응답 필드를 채우면, 알아서 클라이언트에게 응답이 전달된다!!
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
