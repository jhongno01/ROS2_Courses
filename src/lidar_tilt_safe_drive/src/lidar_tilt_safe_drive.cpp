#include <signal.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <limits>
#include <memory>
#include <mutex>
#include <thread>

#include "geometry_msgs/msg/twist_stamped.hpp"
#include "more_interfaces/msg/lidar_angle_range.hpp"
#include "more_interfaces/msg/lidar_safety_status.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "std_msgs/msg/float32.hpp"

using std::placeholders::_1;
using namespace std::chrono_literals;

namespace
{
  volatile sig_atomic_t g_keep_running = 1;
  constexpr double kPi = 3.14159265358979323846;

  double wrap_to_360(double angle_deg)
  {
    double wrapped = std::fmod(angle_deg, 360.0);
    if (wrapped < 0.0)
    {
      wrapped += 360.0;
    }
    return wrapped;
  }

  double normalize_angle_deg(double angle_deg)
  {
    double wrapped = wrap_to_360(angle_deg);
    if (wrapped >= 180.0)
    {
      wrapped -= 360.0;
    }
    return wrapped;
  }

  double radians_to_degrees(double angle_rad)
  {
    return angle_rad * 180.0 / kPi;
  }

  void signal_handler(int)
  {
    g_keep_running = 0;
  }
} // namespace

struct WindowResult
{
  bool valid = false;
  int min_angle_deg = 0;
  float min_distance_m = std::numeric_limits<float>::infinity();
  float left_clearance_m = 0.0f;
  float right_clearance_m = 0.0f;
};

class LidarTiltSafeDrive : public rclcpp::Node
{
public:
  LidarTiltSafeDrive()
      : Node("lidar_tilt_safe_drive"),
        stop_distance_m_(static_cast<float>(this->declare_parameter("stop_distance_m", 0.20))),
        slow_distance_m_(static_cast<float>(this->declare_parameter("slow_distance_m", 0.60))),
        steer_distance_m_(static_cast<float>(this->declare_parameter("steer_distance_m", 0.80))),
        tilt_stop_deg_(static_cast<float>(this->declare_parameter("tilt_stop_deg", 60.0))),
        max_turn_rate_(static_cast<float>(this->declare_parameter("max_turn_rate", 2.84))), // 따로 제한하기 위함
        control_period_ms_(this->declare_parameter("control_period_ms", 100)),
        report_period_ms_(this->declare_parameter("report_period_ms", 1000))
  {
    cmd_vel_publisher_ =
        this->create_publisher<geometry_msgs::msg::TwistStamped>("/cmd_vel", 10);
    status_publisher_ =
        this->create_publisher<more_interfaces::msg::LidarSafetyStatus>("drive_safety_status", 10);

    speed_subscription_ = this->create_subscription<std_msgs::msg::Float32>(
        "drive_speed", 10, std::bind(&LidarTiltSafeDrive::speed_callback, this, _1));

    angle_range_subscription_ =
        this->create_subscription<more_interfaces::msg::LidarAngleRange>(
            "monitor_angle_range", 10, std::bind(&LidarTiltSafeDrive::angle_range_callback, this, _1));

    scan_subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "/scan",
        rclcpp::SensorDataQoS(),
        std::bind(&LidarTiltSafeDrive::scan_callback, this, _1));

    imu_subscription_ = this->create_subscription<sensor_msgs::msg::Imu>(
        "/imu",
        rclcpp::SensorDataQoS(),
        std::bind(&LidarTiltSafeDrive::imu_callback, this, _1));

    control_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(control_period_ms_),
        std::bind(&LidarTiltSafeDrive::control_timer_callback, this));
    report_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(report_period_ms_),
        std::bind(&LidarTiltSafeDrive::report_timer_callback, this));

    RCLCPP_INFO(
        this->get_logger(),
        "Safety drive started. stop_distance=%.2fm, tilt_stop=%.1fdeg, default window=%ddeg -> %ddeg",
        stop_distance_m_,
        tilt_stop_deg_,
        window_start_deg_,
        window_end_deg_);
  }

  void publish_stop_command()
  {
    geometry_msgs::msg::TwistStamped stop_command;

    for (int i = 0; i < 5; ++i)
    {
      stop_command.header.stamp = this->get_clock()->now();
      stop_command.twist.linear.x = 0.0;
      stop_command.twist.linear.y = 0.0;
      stop_command.twist.linear.z = 0.0;
      stop_command.twist.angular.x = 0.0;
      stop_command.twist.angular.y = 0.0;
      stop_command.twist.angular.z = 0.0;

      cmd_vel_publisher_->publish(stop_command);
      rclcpp::sleep_for(100ms);
    }
  }

private:
  struct StateSnapshot
  {
    float requested_speed = 0.0f;
    int window_start_deg = -100;
    int window_end_deg = 80;
    bool imu_ready = false;
    bool scan_ready = false;
    float roll_deg = 0.0f;
    float pitch_deg = 0.0f;
    float max_tilt_deg = 0.0f;
    WindowResult window_result;
  };

  struct OutputSnapshot
  {
    StateSnapshot state;
    float output_linear_x = 0.0f;
    float output_angular_z = 0.0f;
    bool tilt_safe = false;
    bool stop_for_distance = false;
  };

  void speed_callback(const std_msgs::msg::Float32::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    requested_speed_ = max_linear_speed_ * std::clamp(msg->data, 0.0f, 1.0f);
  }

  void angle_range_callback(const more_interfaces::msg::LidarAngleRange::SharedPtr msg)
  {
    const int normalized_start = static_cast<int>(std::lround(normalize_angle_deg(msg->start_deg)));
    const int normalized_end = static_cast<int>(std::lround(normalize_angle_deg(msg->end_deg)));

    {
      std::lock_guard<std::mutex> lock(data_mutex_);
      window_start_deg_ = normalized_start;
      window_end_deg_ = normalized_end;
    }

    RCLCPP_INFO(
        this->get_logger(),
        "Monitoring window normalized to %d deg -> %d deg",
        normalized_start,
        normalized_end);
  }

  void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
  {
    const double x = msg->orientation.x;
    const double y = msg->orientation.y;
    const double z = msg->orientation.z;
    const double w = msg->orientation.w;

    const double sinr_cosp = 2.0 * (w * x + y * z);
    const double cosr_cosp = 1.0 - 2.0 * (x * x + y * y);
    const double roll_rad = std::atan2(sinr_cosp, cosr_cosp);

    const double sinp = 2.0 * (w * y - z * x);
    const double pitch_rad = std::abs(sinp) >= 1.0 ? std::copysign(kPi / 2.0, sinp) : std::asin(sinp);

    std::lock_guard<std::mutex> lock(data_mutex_);
    roll_deg_ = static_cast<float>(radians_to_degrees(roll_rad));
    pitch_deg_ = static_cast<float>(radians_to_degrees(pitch_rad));
    max_tilt_deg_ = std::max(std::fabs(roll_deg_), std::fabs(pitch_deg_));
    imu_ready_ = true;
  }

  void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
  {
    int window_start_deg = 0;
    int window_end_deg = 0;
    {
      std::lock_guard<std::mutex> lock(data_mutex_);
      window_start_deg = window_start_deg_;
      window_end_deg = window_end_deg_;
    }

    WindowResult result;
    result.left_clearance_m = 0.0f;
    result.right_clearance_m = 0.0f;

    for (std::size_t i = 0; i < msg->ranges.size(); ++i)
    {
      const double angle_rad = msg->angle_min + static_cast<double>(i) * msg->angle_increment; // 각 beam의 실제 각도 계산
      const double angle_deg = normalize_angle_deg(radians_to_degrees(angle_rad));

      if (!is_in_monitoring_window(angle_deg, window_start_deg, window_end_deg)) // Beam이 감시 구간 내에 있는지 판정
      {
        continue;
      }

      const float distance = sanitize_distance(msg->ranges[i], msg->range_min, msg->range_max);
      result.valid = true;

      if (angle_deg >= 0.0) // 왼쪽이 양수, 오른쪽이 음수
      {
        result.left_clearance_m = std::max(result.left_clearance_m, distance);
      }
      else
      {
        result.right_clearance_m = std::max(result.right_clearance_m, distance);
      }

      if (distance < result.min_distance_m)
      {
        result.min_distance_m = distance;
        result.min_angle_deg = static_cast<int>(std::lround(angle_deg));
      }
    }

    std::lock_guard<std::mutex> lock(data_mutex_);
    latest_window_result_ = result;
    scan_ready_ = true;
  }

  float sanitize_distance(float distance, float range_min, float range_max) const
  {
    if (!std::isfinite(distance))
    {
      return invalid_lidar_distance_m_;
    }

    if (distance <= 0.0f)
    {
      return invalid_lidar_distance_m_;
    }

    if (distance < range_min)
    {
      return invalid_lidar_distance_m_;
    }

    if (std::isfinite(range_max) && distance > range_max)
    {
      return invalid_lidar_distance_m_;
    }

    return distance;
  }

  bool is_in_monitoring_window(double angle_deg, double start_deg, double end_deg) const
  {
    const double span_to_end = wrap_to_360(end_deg - start_deg);
    const double span_to_angle = wrap_to_360(angle_deg - start_deg);
    return span_to_angle <= span_to_end + 1e-6; //
  }

  StateSnapshot get_state_snapshot()
  {
    std::lock_guard<std::mutex> lock(data_mutex_);

    StateSnapshot snapshot;
    snapshot.requested_speed = requested_speed_;
    snapshot.window_start_deg = window_start_deg_;
    snapshot.window_end_deg = window_end_deg_;
    snapshot.imu_ready = imu_ready_;
    snapshot.scan_ready = scan_ready_;
    snapshot.roll_deg = roll_deg_;
    snapshot.pitch_deg = pitch_deg_;
    snapshot.max_tilt_deg = max_tilt_deg_;
    snapshot.window_result = latest_window_result_;
    return snapshot;
  }

  OutputSnapshot evaluate_output(const StateSnapshot &state) const
  {
    OutputSnapshot output;
    output.state = state;
    output.tilt_safe = state.imu_ready && state.max_tilt_deg <= tilt_stop_deg_;
    output.stop_for_distance =
        state.scan_ready &&
        state.window_result.valid &&
        state.window_result.min_distance_m <= stop_distance_m_;

    if (!state.imu_ready || !state.scan_ready)
    {
      return output;
    }

    if (state.requested_speed <= 0.0f)
    {
      output.tilt_safe = state.max_tilt_deg <= tilt_stop_deg_;
      return output;
    }

    if (!output.tilt_safe || output.stop_for_distance)
    {
      return output;
    }

    float output_linear_x = state.requested_speed;
    float output_angular_z = 0.0f;

    if (state.window_result.valid)
    {
      output_linear_x *= compute_linear_scale(state.window_result.min_distance_m);
      output_angular_z = compute_steering_command(state.window_result);
    }

    output.output_linear_x = output_linear_x;
    output.output_angular_z = output_angular_z;
    return output;
  }

  float compute_linear_scale(float min_distance_m) const
  {
    if (!std::isfinite(min_distance_m) || min_distance_m >= slow_distance_m_)
    {
      return 1.0f;
    }

    if (min_distance_m <= stop_distance_m_)
    {
      return 0.0f;
    }

    const float span = slow_distance_m_ - stop_distance_m_; // 장애물 거리에 따라 점차 느려지게
    if (span <= 0.0f)
    {
      return 1.0f;
    }

    return std::clamp((min_distance_m - stop_distance_m_) / span, 0.0f, 1.0f);
  }

  float compute_steering_command(const WindowResult &result) const
  {
    if (!result.valid || !std::isfinite(result.min_distance_m))
    {
      return 0.0f;
    }

    const float distance_span = std::max(steer_distance_m_ - stop_distance_m_, 0.01f); // 장애물이 가까울수록 1에 가까움
    const float proximity = std::clamp(
        (steer_distance_m_ - result.min_distance_m) / distance_span,
        0.0f,
        1.0f);

    int turn_direction = 0;
    if (result.min_angle_deg > 5)
    {
      turn_direction = -1;
    }
    else if (result.min_angle_deg < -5)
    {
      turn_direction = 1;
    } // +-5도 이내에 장애물이 있으면, 좌우 클리어런스 비교하여 더 넓은 쪽으로 회전하도록 함. (장애물이 정면에 있는 경우를 좀 더 부드럽게 처리하기 위함)
    else if (result.left_clearance_m > result.right_clearance_m + 0.05f)
    {
      turn_direction = 1;
    }
    else if (result.right_clearance_m > result.left_clearance_m + 0.05f)
    {
      turn_direction = -1;
    }

    const float requested_turn_rate = requested_speed_ * static_cast<float>(turn_direction) * max_turn_rate_ * proximity;
    return std::clamp(requested_turn_rate, -max_angular_speed_, max_angular_speed_);
  }

  void control_timer_callback()
  {
    const StateSnapshot state = get_state_snapshot();
    const OutputSnapshot output = evaluate_output(state);

    geometry_msgs::msg::TwistStamped cmd_message;
    cmd_message.header.stamp = this->get_clock()->now();
    cmd_message.twist.linear.x = output.output_linear_x;
    cmd_message.twist.linear.y = 0.0;
    cmd_message.twist.linear.z = 0.0;
    cmd_message.twist.angular.x = 0.0;
    cmd_message.twist.angular.y = 0.0;
    cmd_message.twist.angular.z = output.output_angular_z;
    cmd_vel_publisher_->publish(cmd_message);

    more_interfaces::msg::LidarSafetyStatus status_message;
    status_message.window_start_deg = output.state.window_start_deg;
    status_message.window_end_deg = output.state.window_end_deg;
    status_message.min_angle_deg = output.state.window_result.min_angle_deg;
    status_message.min_distance_m = output.state.window_result.valid ? output.state.window_result.min_distance_m : std::numeric_limits<float>::quiet_NaN();
    status_message.tilt_deg = output.state.max_tilt_deg;
    status_message.requested_speed = output.state.requested_speed;
    status_message.output_linear_x = output.output_linear_x;
    status_message.output_angular_z = output.output_angular_z;
    status_message.imu_ready = output.state.imu_ready;
    status_message.scan_ready = output.state.scan_ready;
    status_message.window_measurement_valid = output.state.window_result.valid;
    status_message.tilt_safe = output.tilt_safe;
    status_message.stop_for_distance = output.stop_for_distance;
    status_publisher_->publish(status_message);

    std::lock_guard<std::mutex> lock(data_mutex_);
    last_output_snapshot_ = output;
  }

  void report_timer_callback()
  {
    OutputSnapshot output;
    {
      std::lock_guard<std::mutex> lock(data_mutex_);
      output = last_output_snapshot_;
    }

    if (!output.state.imu_ready || !output.state.scan_ready)
    {
      RCLCPP_INFO(this->get_logger(), "Waiting for IMU and LiDAR data...");
      return;
    }

    if (!output.state.window_result.valid)
    {
      RCLCPP_INFO(
          this->get_logger(),
          "No valid LiDAR sample in %d deg -> %d deg | tilt=%.1f deg | cmd=(%.2f, %.2f)",
          output.state.window_start_deg,
          output.state.window_end_deg,
          output.state.max_tilt_deg,
          output.output_linear_x,
          output.output_angular_z);
      return;
    }

    RCLCPP_INFO(
        this->get_logger(),
        "Window %d deg -> %d deg | closest object: angle %d deg, distance %.1f cm | tilt %.1f deg | stop(tlt=%s dst=%s) | cmd=(%.2f, %.2f)",
        output.state.window_start_deg,
        output.state.window_end_deg,
        output.state.window_result.min_angle_deg,
        output.state.window_result.min_distance_m * 100.0f,
        output.state.max_tilt_deg,
        output.tilt_safe ? "no" : "yes",
        output.stop_for_distance ? "yes" : "no",
        output.output_linear_x,
        output.output_angular_z);
  }

  const float stop_distance_m_;
  const float slow_distance_m_;
  const float steer_distance_m_;
  const float tilt_stop_deg_;
  const float max_turn_rate_;
  const int control_period_ms_;
  const int report_period_ms_;

  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr cmd_vel_publisher_;
  rclcpp::Publisher<more_interfaces::msg::LidarSafetyStatus>::SharedPtr status_publisher_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr speed_subscription_;
  rclcpp::Subscription<more_interfaces::msg::LidarAngleRange>::SharedPtr angle_range_subscription_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_subscription_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscription_;
  rclcpp::TimerBase::SharedPtr control_timer_;
  rclcpp::TimerBase::SharedPtr report_timer_;

  std::mutex data_mutex_;
  const float invalid_lidar_distance_m_ = 99.9f;
  const float max_linear_speed_ = 0.22f;  // Turtlebot3-burger max linear speed (m/s)
  const float max_angular_speed_ = 2.84f; // Turtlebot3-burger max angular speed (rad/s)
  float requested_speed_ = 0.0f;
  int window_start_deg_ = -100;
  int window_end_deg_ = 80;
  bool imu_ready_ = false;
  bool scan_ready_ = false;
  float roll_deg_ = 0.0f;
  float pitch_deg_ = 0.0f;
  float max_tilt_deg_ = 0.0f;
  WindowResult latest_window_result_;
  OutputSnapshot last_output_snapshot_;
};

int main(int argc, char *argv[])
{
  signal(SIGINT, signal_handler);

  rclcpp::init(argc, argv);
  auto node = std::make_shared<LidarTiltSafeDrive>();
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);

  while (rclcpp::ok() && g_keep_running)
  {
    executor.spin_some();
    std::this_thread::sleep_for(10ms);
  }

  node->publish_stop_command();

  if (rclcpp::ok())
  {
    rclcpp::shutdown();
  }

  return 0;
}
