#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <cstdio>

/*package dependencies -->CMakelist.txt, package.xml*/

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

class hw01_pub : public rclcpp::Node
{
public:
    hw01_pub()
        : Node("hw01_pub"), count_(0)
    {
        hw01_pub_ = this->create_publisher<std_msgs::msg::String>("topic_hw01_a", 10);
        timer_ = this->create_wall_timer(500ms, std::bind(&hw01_pub::hw01_timer_callback, this));
    }
    
    // for input int a, int b and display them
    int a, b;
    void input()
    {
        printf("Please type two int : ");
        scanf("%d %d", &a, &b);
    }
    void display()
    {
        printf("%d, %d...\n", a, b);
        fflush(stdout); 
    }

private:
    // callback to subscriber count_++ * a, count_++ * b
    void hw01_timer_callback()
    {
        auto message_hw01 = std_msgs::msg::String();
        message_hw01.data = "Hello, world! " + std::to_string(count_ * a) + " " + std::to_string(count_ * b);
        count_++;
        RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message_hw01.data.c_str());

        hw01_pub_->publish(message_hw01);
    }
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr hw01_pub_;
    size_t count_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto hw01 = std::make_shared<hw01_pub>();
    hw01->input();
    hw01->display();
    std::this_thread::sleep_for(std::chrono::seconds(2));
    rclcpp::spin(hw01);
    rclcpp::shutdown();
    return 0;
}
