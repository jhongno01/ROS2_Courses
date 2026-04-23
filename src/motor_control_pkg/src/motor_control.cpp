// Running simple motor control node, which takes in velocity and orientation from the terminal and publishes it to /cmd_vel topic.
#include <stdio.h>
#include <chrono>
/*package dependencies -->CMakelist.txt, package.xml*/
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"

/* This example creates a subclass of Node and uses std::bind() to register a
* member function as a callback from the timer. */

using namespace std::chrono_literals;

int main(int argc, char * argv[])
{
    float velocity_, orientation_;
    rclcpp::init(argc,argv);
    auto node = rclcpp::Node::make_shared("motor_robot");
    // define publisher
    auto motor_ = node->create_publisher<geometry_msgs::msg::TwistStamped>("/cmd_vel",10);

    geometry_msgs::msg::TwistStamped cmd;
    
    while(rclcpp::ok()){
        // get the pressed key
    
        scanf("%f %f", &velocity_, &orientation_);
        cmd.twist.linear.x = velocity_; cmd.twist.linear.y = 0.0; cmd.twist.linear.z = 0.0;
        cmd.twist.angular.x = 0.0; cmd.twist.angular.y = 0.0; cmd.twist.angular.z = orientation_;
        motor_->publish(cmd);
        rclcpp::spin_some(node);
    }

        // 종료 전 정지 명령 여러 번 보내기
    geometry_msgs::msg::TwistStamped stop_cmd;
    stop_cmd.header.frame_id = "";

    for (int i = 0; i < 5; i++) {
        stop_cmd.header.stamp = node->get_clock()->now();

        stop_cmd.twist.linear.x = 0.0;
        stop_cmd.twist.linear.y = 0.0;
        stop_cmd.twist.linear.z = 0.0;
        stop_cmd.twist.angular.x = 0.0;
        stop_cmd.twist.angular.y = 0.0;
        stop_cmd.twist.angular.z = 0.0;

        motor_->publish(stop_cmd);
        rclcpp::spin_some(node);
        rclcpp::sleep_for(100ms);
    }

    rclcpp::shutdown();
    return 0;
}