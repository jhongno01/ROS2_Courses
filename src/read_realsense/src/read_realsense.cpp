#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/image.hpp"

using std::placeholders::_1;

class rgbd_read : public rclcpp::Node
{
public:
  rgbd_read() : Node("rgbd__")
  {
      auto defualt_qos = rclcpp::SensorDataQoS();
      rgbd__ = this->create_subscription<sensor_msgs::msg::Image>(
          "/camera/camera/depth/image_rect_raw", defualt_qos,
          std::bind(&rgbd_read::rgbd_callback, this, _1));
  }

private:
  void rgbd_callback(const sensor_msgs::msg::Image::SharedPtr _msg) const
  {
    RCLCPP_INFO(this->get_logger(), "[0th] = '%d' [10000th] = '%d'", _msg->data[10001], _msg->data[10000]);
  }
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr rgbd__;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto a = std::make_shared<rgbd_read>();
  rclcpp::spin(a);
  rclcpp::shutdown();
  return 0;
}