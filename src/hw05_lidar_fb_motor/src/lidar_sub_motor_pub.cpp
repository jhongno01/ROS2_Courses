#include <stdio.h>
#include <signal.h>

#include <array>
#include <chrono>
#include <cmath>
#include <functional>
#include <memory>
#include <mutex>
#include <thread>

#include "geometry_msgs/msg/twist_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

using std::placeholders::_1;
using namespace std::chrono_literals;

namespace
{
volatile sig_atomic_t g_keep_running = 1;

void signal_handler(int)
{
  g_keep_running = 0;
  if (rclcpp::ok()) {
    rclcpp::shutdown();
  }
}
}  // namespace

class LidarSubMotorPub : public rclcpp::Node
{
public:
  LidarSubMotorPub()
  : Node("lider_sub_motor_pub"),
    desired_linear_x_(0.0f),
    desired_angular_z_(0.0f),
    front_distance_(99.9f),
    left_45_distance_(99.9f),
    right_45_distance_(99.9f)
  {
    motor_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>("/cmd_vel", 10);

    lidar_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      "/scan",
      rclcpp::SensorDataQoS(),
      std::bind(&LidarSubMotorPub::lidar_callback, this, _1));

    timer_ = this->create_wall_timer(500ms, std::bind(&LidarSubMotorPub::timer_callback, this));
  }

  void update_user_command(float linear_x, float angular_z)
  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    desired_linear_x_ = linear_x;
    desired_angular_z_ = angular_z;
  }

  void publish_stop_command()
  {
    geometry_msgs::msg::TwistStamped stop_cmd;

    for (int i = 0; i < 5; ++i) {
      stop_cmd.header.stamp = this->get_clock()->now();
      stop_cmd.twist.linear.x = 0.0;
      stop_cmd.twist.linear.y = 0.0;
      stop_cmd.twist.linear.z = 0.0;
      stop_cmd.twist.angular.x = 0.0;
      stop_cmd.twist.angular.y = 0.0;
      stop_cmd.twist.angular.z = 0.0;

      motor_pub_->publish(stop_cmd);
      rclcpp::sleep_for(100ms);
    }
  }

private:
  void lidar_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
  {
    constexpr int front_index = 0;
    constexpr int left_45_index = 45;
    constexpr int right_45_index = 315;

    std::lock_guard<std::mutex> lock(data_mutex_);

    front_distance_ = sanitize_distance(read_distance(msg, front_index));
    left_45_distance_ = sanitize_distance(read_distance(msg, left_45_index));
    right_45_distance_ = sanitize_distance(read_distance(msg, right_45_index));
  }

  float read_distance(const sensor_msgs::msg::LaserScan::SharedPtr msg, int index) const
  {
    if (index < 0 || index >= static_cast<int>(msg->ranges.size())) {
      return 99.9f;
    }

    return msg->ranges[index];
  }

  float sanitize_distance(float distance) const
  {
    if (!std::isfinite(distance) || distance == 0.0f) {
      return 99.9f;
    }

    return distance;
  }

  bool is_obstacle(float distance) const
  {
    return distance < 0.4f;
  }

  void timer_callback()
  {
    float desired_linear_x;
    float desired_angular_z;
    float front_distance;
    float left_45_distance;
    float right_45_distance;

    {
      std::lock_guard<std::mutex> lock(data_mutex_);
      desired_linear_x = desired_linear_x_;
      desired_angular_z = desired_angular_z_;
      front_distance = front_distance_;
      left_45_distance = left_45_distance_;
      right_45_distance = right_45_distance_;
    }

    const bool front_obstacle = is_obstacle(front_distance);
    const bool left_obstacle = is_obstacle(left_45_distance);
    const bool right_obstacle = is_obstacle(right_45_distance);

    float output_linear_x = desired_linear_x;
    float output_angular_z = desired_angular_z;

    if (front_obstacle) {
      output_linear_x = 0.0f;
      output_angular_z = 0.0f;
    } else if (left_obstacle && !right_obstacle) {
      output_linear_x = desired_linear_x > 0.0f ? desired_linear_x * 0.5f : desired_linear_x;
      output_angular_z = -0.6f;
    } else if (right_obstacle && !left_obstacle) {
      output_linear_x = desired_linear_x > 0.0f ? desired_linear_x * 0.5f : desired_linear_x;
      output_angular_z = 0.6f;
    } else if (left_obstacle && right_obstacle) {
      output_linear_x = 0.0f;

      if (desired_angular_z > 0.0f) {
        output_angular_z = 0.6f;
      } else if (desired_angular_z < 0.0f) {
        output_angular_z = -0.6f;
      } else {
        output_angular_z = 0.0f;
      }
    }

    geometry_msgs::msg::TwistStamped cmd;
    cmd.header.stamp = this->get_clock()->now();
    cmd.twist.linear.x = output_linear_x;
    cmd.twist.linear.y = 0.0;
    cmd.twist.linear.z = 0.0;
    cmd.twist.angular.x = 0.0;
    cmd.twist.angular.y = 0.0;
    cmd.twist.angular.z = output_angular_z;
    motor_pub_->publish(cmd);

    RCLCPP_INFO(
      this->get_logger(),
      "front: %.2f m, left45: %.2f m, right45: %.2f m | input: %.2f %.2f | output: %.2f %.2f",
      front_distance,
      left_45_distance,
      right_45_distance,
      desired_linear_x,
      desired_angular_z,
      output_linear_x,
      output_angular_z);
  }

  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr motor_pub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr lidar_sub_;
  rclcpp::TimerBase::SharedPtr timer_;

  std::mutex data_mutex_;
  float desired_linear_x_;
  float desired_angular_z_;
  float front_distance_;
  float left_45_distance_;
  float right_45_distance_;
};

int main(int argc, char * argv[])
{
  signal(SIGINT, signal_handler);

  rclcpp::init(argc, argv);
  auto node = std::make_shared<LidarSubMotorPub>();

  std::thread spin_thread([&node]() {
    rclcpp::spin(node);
  });

  while (rclcpp::ok() && g_keep_running) {
    float velocity = 0.0f;
    float angular_velocity = 0.0f;

    printf("input vel angular_vel: ");
    const int input_count = scanf("%f %f", &velocity, &angular_velocity);

    if (input_count != 2) {
      break;
    }

    node->update_user_command(velocity, angular_velocity);
  }

  node->publish_stop_command();

  if (rclcpp::ok()) {
    rclcpp::shutdown();
  }

  if (spin_thread.joinable()) {
    spin_thread.join();
  }

  return 0;
}
