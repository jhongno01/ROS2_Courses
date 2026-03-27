#include "rclcpp/rclcpp.hpp" //ROS 기본 헤더 파일
#include "more_interfaces/srv/add_two_ints.hpp" //more_interfaces 패키지의 AddTwoInts 서비스 헤더 파일
// more_interfaces 빌드하면 자동 생성됨

#include <memory> //std::make_shared 사용하기 위한 헤더 파일

void add(const std::shared_ptr<more_interfaces::srv::AddTwoInts::Request> request,
         std::shared_ptr<more_interfaces::srv::AddTwoInts::Response> response)
{
    // 요청된 서비스에 대한 응답 값을 연산
  response->sum = request->a + request->b; //요청으로 받은 a와 b를 더해서 response의 sum에 저장
    //요청 로그 출력
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Incoming request\na=%ld, b=%ld",
        request->a, request->b); 
    //서비스 값으로부터 연산된 응답 값을 표시
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Sending back response: [%ld]", (long int)response->sum);
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv); //ROS2 시스템 초기화

  auto node = rclcpp::Node::make_shared("add_two_ints_server"); //add_two_ints_server 노드 선언
  // add 함수를 사용하는 add_two_ints라는 이름의 서비스 노드 선언
  rclcpp::Service<more_interfaces::srv::AddTwoInts>::SharedPtr service = 
    node->create_service<more_interfaces::srv::AddTwoInts>("add_two_ints", &add);


  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Ready to add two ints."); //서비스 서버가 준비되었다는 로그 출력
  
  rclcpp::spin(node); //노드가 종료될 때까지 대기
  rclcpp::shutdown(); //ROS2 시스템 종료
  return 0;
}
