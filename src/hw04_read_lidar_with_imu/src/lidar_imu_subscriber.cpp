// 앞으로 30도 기울어졌을 때, 0도 지점의 Lidar 값 출력
// 뒤로 30도 기울어졌을 때, 180도 지점의 Lidar 값 출력
// 모든 데이터는 0.5초마다 한번씩 출력

#include <memory>
#include <chrono>
#include <functional>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

using std::placeholders::_1;   
using namespace std::chrono_literals;

class LidarImuSubscriber : public rclcpp::Node
{
public:
  LidarImuSubscriber()
  : Node("lidar_imu_subscriber"), angle_30_forward_(false), angle_30_backward_(false)
  , current_angle_y_(0.0f), distances_0_(0.0f), distances_180_(0.0f)
  {
    imu_subscription_ = this->create_subscription<sensor_msgs::msg::Imu>(
      "/imu", rclcpp::SensorDataQoS(),std::bind(&LidarImuSubscriber::imu_callback, this, _1));

    lidar_subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      "/scan", rclcpp::SensorDataQoS(), std::bind(&LidarImuSubscriber::lidar_callback, this, _1));

    timer_ = this->create_wall_timer(500ms, std::bind(&LidarImuSubscriber::timer_callback, this));
  }

private:
  void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
    {
      current_angle_y_ = msg->orientation.y; // IMU에서 y축 각도 값을 가져온다.  
      // RCLCPP_INFO(this->get_logger(), "recieved msg = '%f'", current_angle_y_);
    };

  void lidar_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
      int index_0 = (0 - msg->angle_min) / msg->angle_increment; // 0도에 해당하는 인덱스 계산
      int index_180 = (M_PI - msg->angle_min) / msg->angle_increment; // 180도에 해당하는 인덱스 계산

      distances_0_ = msg->ranges[index_0]; // 0도에서의 거리 값
      distances_180_ = msg->ranges[index_180]; // 180도에서의 거리 값
    }

  void timer_callback()
    {
        // Timer callback implementation to print the desired Lidar values based on IMU data

      bool current_distance_0_ = std::isfinite(distances_0_);
      bool current_distance_180_ = std::isfinite(distances_180_);

      if (current_distance_0_ && (current_angle_y_ > 0.3f)) { // 30도 이상 기울어졌을 때
        angle_30_forward_ = true;
        angle_30_backward_ = false;
        RCLCPP_INFO(this->get_logger(), "'%f' degrees forward, distance at 0 degrees: '%f' m", current_angle_y_, distances_0_);

      } else if (current_distance_180_ && (current_angle_y_ < -0.3f)) { // -30도 이하로 기울어졌을 때
        angle_30_forward_ = false;
        angle_30_backward_ = true;
        RCLCPP_INFO(this->get_logger(), "'%f' degrees backward, distance at 180 degrees: '%f' m", current_angle_y_, distances_180_);
      
      } else if (current_distance_0_ && current_distance_180_){ // 무한대는 아니지만, 30도 이상 기울어지지 않았을 때
        angle_30_forward_ = false;
        angle_30_backward_ = false;
        RCLCPP_INFO(this->get_logger(), "Turtlebot is not tilted at 30 degress, current angle: '%f' degrees", current_angle_y_);
      }
    }

    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscription_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr lidar_subscription_;
    rclcpp::TimerBase::SharedPtr timer_;

    bool angle_30_forward_ = false;
    bool angle_30_backward_ = false;
    float current_angle_y_ = 0.0f;
    float distances_0_ = 0.0f;
    float distances_180_ = 0.0f;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LidarImuSubscriber>());
  rclcpp::shutdown();
  return 0;
}