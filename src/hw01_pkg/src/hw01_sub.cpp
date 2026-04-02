#include <functional>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using std::placeholders::_1;

class hw01_sub : public rclcpp::Node
{
public:
    hw01_sub()
        : Node("hw01_sub")
    {
        hw01_sub_ = this->create_subscription<std_msgs::msg::String>
        ("topic_hw01_a", 10, std::bind(&hw01_sub::hw01_topic_callback, this, _1));
    }

private:
    void hw01_topic_callback(const std_msgs::msg::String::SharedPtr msg) const
    {
        RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg->data.c_str());
    }
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr hw01_sub_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto aaa = std::make_shared<hw01_sub>();
    rclcpp::spin(aaa);
    rclcpp::shutdown();
    return 0;
}