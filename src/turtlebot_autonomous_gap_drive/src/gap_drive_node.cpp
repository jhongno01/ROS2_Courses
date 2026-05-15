#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <limits>
#include <memory>
#include <mutex>
#include <string>

#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

using std::placeholders::_1;
using namespace std::chrono_literals;

namespace
{
constexpr double kPi = 3.14159265358979323846;
constexpr double kTwoPi = 2.0 * kPi;

double deg_to_rad(double degree)
{
  return degree * kPi / 180.0;
}

double rad_to_deg(double radian)
{
  return radian * 180.0 / kPi;
}

double clamp_value(double value, double low, double high)
{
  return std::max(low, std::min(value, high));
}

double sign_value(double value)
{
  if (value > 0.0) {
    return 1.0;
  }
  if (value < 0.0) {
    return -1.0;
  }
  return 0.0;
}
}  // namespace

struct ScanSummary
{
  double front_m = 0.0;
  double left_m = 0.0;
  double right_m = 0.0;
  double best_gap_m = 0.0;
  double best_angle_rad = 0.0;
  bool has_safe_gap = false;
};

struct DriveOutput
{
  double linear_x = 0.0;
  double angular_z = 0.0;
  std::string mode = "WAIT_SCAN";
  ScanSummary scan;
};

class TurtlebotAutonomousGapDrive : public rclcpp::Node
{
public:
  TurtlebotAutonomousGapDrive()
  : Node("turtlebot_autonomous_gap_drive"),
    scan_topic_(this->declare_parameter<std::string>("scan_topic", "/scan")),
    cmd_vel_topic_(this->declare_parameter<std::string>("cmd_vel_topic", "/cmd_vel")),
    cmd_vel_stamped_(this->declare_parameter<bool>("cmd_vel_stamped", false)),
    control_period_ms_(this->declare_parameter<int>("control_period_ms", 100)),
    report_period_ms_(this->declare_parameter<int>("report_period_ms", 1000)),
    max_linear_speed_(this->declare_parameter<double>("max_linear_speed", 0.10)),
    min_linear_speed_(this->declare_parameter<double>("min_linear_speed", 0.03)),
    max_angular_speed_(this->declare_parameter<double>("max_angular_speed", 1.25)),
    emergency_stop_distance_m_(this->declare_parameter<double>("emergency_stop_distance_m", 0.22)),
    front_stop_distance_m_(this->declare_parameter<double>("front_stop_distance_m", 0.34)),
    front_slow_distance_m_(this->declare_parameter<double>("front_slow_distance_m", 0.85)),
    side_stop_distance_m_(this->declare_parameter<double>("side_stop_distance_m", 0.18)),
    safe_gap_distance_m_(this->declare_parameter<double>("safe_gap_distance_m", 0.45)),
    target_distance_cap_m_(this->declare_parameter<double>("target_distance_cap_m", 2.50)),
    search_angle_deg_(this->declare_parameter<double>("search_angle_deg", 105.0)),
    gap_window_deg_(this->declare_parameter<double>("gap_window_deg", 22.0)),
    sample_step_deg_(this->declare_parameter<double>("sample_step_deg", 3.0)),
    front_window_deg_(this->declare_parameter<double>("front_window_deg", 18.0)),
    side_angle_deg_(this->declare_parameter<double>("side_angle_deg", 70.0)),
    side_window_deg_(this->declare_parameter<double>("side_window_deg", 18.0)),
    heading_gain_(this->declare_parameter<double>("heading_gain", 1.55)),
    centering_gain_(this->declare_parameter<double>("centering_gain", 0.55)),
    forward_bias_weight_(this->declare_parameter<double>("forward_bias_weight", 0.55)),
    turn_slowdown_gain_(this->declare_parameter<double>("turn_slowdown_gain", 0.65)),
    recovery_turn_speed_(this->declare_parameter<double>("recovery_turn_speed", 0.75)),
    recovery_flip_seconds_(this->declare_parameter<double>("recovery_flip_seconds", 4.0)),
    lost_scan_timeout_seconds_(this->declare_parameter<double>("lost_scan_timeout_seconds", 1.0)),
    prefer_left_recovery_(this->declare_parameter<bool>("prefer_left_recovery", true))
  {
    if (cmd_vel_stamped_) {
      // ROS2: TwistStamped 퍼블리셔는 수업용 모터 노드처럼 stamped /cmd_vel을 쓰는 환경에서 사용한다.
      cmd_vel_stamped_publisher_ =
        this->create_publisher<geometry_msgs::msg::TwistStamped>(cmd_vel_topic_, 10);
    } else {
      // ROS2: TurtleBot3 Foxy 기본 구동 토픽은 geometry_msgs/msg/Twist 타입의 /cmd_vel이다.
      cmd_vel_publisher_ =
        this->create_publisher<geometry_msgs::msg::Twist>(cmd_vel_topic_, 10);
    }

    // ROS2: 라이다 LaserScan 토픽은 센서 데이터 QoS로 구독해 실시간성을 우선한다.
    scan_subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      scan_topic_,
      rclcpp::SensorDataQoS(),
      std::bind(&TurtlebotAutonomousGapDrive::scan_callback, this, _1));

    // ROS2: wall timer로 일정 주기마다 최신 scan을 해석하고 속도 명령을 만든다.
    control_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(control_period_ms_),
      std::bind(&TurtlebotAutonomousGapDrive::control_timer_callback, this));

    // ROS2: 별도 timer로 주행 상태를 낮은 빈도로 출력해서 터미널 로그를 읽기 쉽게 유지한다.
    report_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(report_period_ms_),
      std::bind(&TurtlebotAutonomousGapDrive::report_timer_callback, this));

    RCLCPP_INFO(
      this->get_logger(),
      "Started gap drive: scan=%s cmd=%s type=%s max_v=%.2f stop=%.2fm safe_gap=%.2fm",
      scan_topic_.c_str(),
      cmd_vel_topic_.c_str(),
      cmd_vel_stamped_ ? "TwistStamped" : "Twist",
      max_linear_speed_,
      front_stop_distance_m_,
      safe_gap_distance_m_);
  }

  void publish_stop_command()
  {
    for (int i = 0; i < 5; ++i) {
      publish_velocity(0.0, 0.0);
      rclcpp::sleep_for(100ms);
    }
  }

private:
  void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    latest_scan_ = msg;
    latest_scan_time_ = this->now();
  }

  void control_timer_callback()
  {
    sensor_msgs::msg::LaserScan::SharedPtr scan;
    rclcpp::Time scan_time;
    {
      std::lock_guard<std::mutex> lock(data_mutex_);
      scan = latest_scan_;
      scan_time = latest_scan_time_;
    }

    DriveOutput output;

    if (!scan || scan->ranges.empty()) {
      output.mode = "WAIT_SCAN";
      publish_velocity(0.0, 0.0);
      store_output(output);
      return;
    }

    const double scan_age = (this->now() - scan_time).seconds();
    if (scan_age > lost_scan_timeout_seconds_) {
      output.mode = "SCAN_TIMEOUT";
      publish_velocity(0.0, 0.0);
      store_output(output);
      return;
    }

    output = compute_drive_output(*scan);
    publish_velocity(output.linear_x, output.angular_z);
    store_output(output);
  }

  DriveOutput compute_drive_output(const sensor_msgs::msg::LaserScan & scan)
  {
    DriveOutput output;
    output.scan = summarize_scan(scan);

    const double left = output.scan.left_m;
    const double right = output.scan.right_m;
    const double front = output.scan.front_m;

    if (front <= emergency_stop_distance_m_) {
      output.linear_x = 0.0;
      output.angular_z = choose_recovery_turn(left, right);
      output.mode = "EMERGENCY_TURN";
      return output;
    }

    if (front <= front_stop_distance_m_ || !output.scan.has_safe_gap) {
      output.linear_x = 0.0;
      output.angular_z = choose_recovery_turn(left, right);
      output.mode = "BLOCKED_TURN";
      return output;
    }

    reset_recovery_state();

    const double heading_cmd = heading_gain_ * output.scan.best_angle_rad;
    const double left_capped = std::min(left, target_distance_cap_m_);
    const double right_capped = std::min(right, target_distance_cap_m_);
    const double centering_error = left_capped - right_capped;
    const double centering_cmd = centering_gain_ * centering_error;

    double angular_z = heading_cmd + centering_cmd;

    if (left < side_stop_distance_m_ && right > left) {
      angular_z = -std::max(0.45, std::fabs(angular_z));
    } else if (right < side_stop_distance_m_ && left > right) {
      angular_z = std::max(0.45, std::fabs(angular_z));
    }

    output.angular_z = clamp_value(angular_z, -max_angular_speed_, max_angular_speed_);

    const double clearance_ratio = clamp_value(
      (front - front_stop_distance_m_) /
      std::max(0.01, front_slow_distance_m_ - front_stop_distance_m_),
      0.0,
      1.0);
    const double turn_ratio = clamp_value(std::fabs(output.angular_z) / max_angular_speed_, 0.0, 1.0);
    const double turn_scale = clamp_value(1.0 - turn_slowdown_gain_ * turn_ratio, 0.25, 1.0);
    const double base_speed =
      min_linear_speed_ + (max_linear_speed_ - min_linear_speed_) * clearance_ratio;

    output.linear_x = clamp_value(base_speed * turn_scale, min_linear_speed_, max_linear_speed_);
    output.mode = "GAP_DRIVE";
    return output;
  }

  ScanSummary summarize_scan(const sensor_msgs::msg::LaserScan & scan) const
  {
    ScanSummary summary;
    summary.front_m = min_range_in_sector(scan, 0.0, front_window_deg_);
    summary.left_m = min_range_in_sector(scan, side_angle_deg_, side_window_deg_);
    summary.right_m = min_range_in_sector(scan, -side_angle_deg_, side_window_deg_);

    double best_score = -std::numeric_limits<double>::infinity();
    for (double angle_deg = -search_angle_deg_;
      angle_deg <= search_angle_deg_;
      angle_deg += std::max(1.0, sample_step_deg_))
    {
      const double clearance = min_range_in_sector(scan, angle_deg, gap_window_deg_);
      if (clearance < safe_gap_distance_m_) {
        continue;
      }

      const double capped_clearance = std::min(clearance, target_distance_cap_m_);
      const double forward_penalty =
        forward_bias_weight_ * std::fabs(angle_deg) / std::max(1.0, search_angle_deg_);
      const double score = capped_clearance - forward_penalty;

      if (score > best_score) {
        best_score = score;
        summary.best_gap_m = clearance;
        summary.best_angle_rad = deg_to_rad(angle_deg);
        summary.has_safe_gap = true;
      }
    }

    return summary;
  }

  double min_range_in_sector(
    const sensor_msgs::msg::LaserScan & scan,
    double center_deg,
    double half_width_deg) const
  {
    double min_range = target_distance_cap_m_;
    bool sampled = false;

    for (double offset_deg = -half_width_deg;
      offset_deg <= half_width_deg;
      offset_deg += std::max(1.0, sample_step_deg_))
    {
      double range = 0.0;
      if (range_at_angle(scan, deg_to_rad(center_deg + offset_deg), range)) {
        min_range = std::min(min_range, sanitize_range(scan, range));
        sampled = true;
      }
    }

    if (!sampled) {
      return 0.0;
    }
    return min_range;
  }

  bool range_at_angle(
    const sensor_msgs::msg::LaserScan & scan,
    double target_angle_rad,
    double & range) const
  {
    if (scan.ranges.empty() || std::fabs(scan.angle_increment) < 1e-9) {
      return false;
    }

    double angle = target_angle_rad;
    while (angle < scan.angle_min) {
      angle += kTwoPi;
    }
    while (angle > scan.angle_max) {
      angle -= kTwoPi;
    }

    if (angle < scan.angle_min || angle > scan.angle_max) {
      return false;
    }

    const int index =
      static_cast<int>(std::lround((angle - scan.angle_min) / scan.angle_increment));
    if (index < 0 || index >= static_cast<int>(scan.ranges.size())) {
      return false;
    }

    range = scan.ranges[index];
    return true;
  }

  double sanitize_range(
    const sensor_msgs::msg::LaserScan & scan,
    double range) const
  {
    if (!std::isfinite(range) || range <= std::max(0.0f, scan.range_min)) {
      if (std::isfinite(scan.range_max) && scan.range_max > 0.0f) {
        return std::min(static_cast<double>(scan.range_max), target_distance_cap_m_);
      }
      return target_distance_cap_m_;
    }

    return clamp_value(range, 0.0, target_distance_cap_m_);
  }

  double choose_recovery_turn(double left, double right)
  {
    const rclcpp::Time now = this->now();

    if (!recovery_active_) {
      recovery_active_ = true;
      recovery_started_at_ = now;

      if (std::fabs(left - right) < 0.05) {
        recovery_direction_ = prefer_left_recovery_ ? 1 : -1;
      } else {
        recovery_direction_ = left > right ? 1 : -1;
      }
    } else if ((now - recovery_started_at_).seconds() > recovery_flip_seconds_) {
      recovery_direction_ *= -1;
      recovery_started_at_ = now;
    }

    return sign_value(static_cast<double>(recovery_direction_)) * recovery_turn_speed_;
  }

  void reset_recovery_state()
  {
    recovery_active_ = false;
  }

  void publish_velocity(double linear_x, double angular_z)
  {
    if (cmd_vel_stamped_) {
      geometry_msgs::msg::TwistStamped cmd;
      cmd.header.stamp = this->now();
      cmd.header.frame_id = "base_link";
      cmd.twist.linear.x = linear_x;
      cmd.twist.linear.y = 0.0;
      cmd.twist.linear.z = 0.0;
      cmd.twist.angular.x = 0.0;
      cmd.twist.angular.y = 0.0;
      cmd.twist.angular.z = angular_z;
      cmd_vel_stamped_publisher_->publish(cmd);
      return;
    }

    geometry_msgs::msg::Twist cmd;
    cmd.linear.x = linear_x;
    cmd.linear.y = 0.0;
    cmd.linear.z = 0.0;
    cmd.angular.x = 0.0;
    cmd.angular.y = 0.0;
    cmd.angular.z = angular_z;
    cmd_vel_publisher_->publish(cmd);
  }

  void store_output(const DriveOutput & output)
  {
    std::lock_guard<std::mutex> lock(output_mutex_);
    last_output_ = output;
  }

  void report_timer_callback()
  {
    DriveOutput output;
    {
      std::lock_guard<std::mutex> lock(output_mutex_);
      output = last_output_;
    }

    RCLCPP_INFO(
      this->get_logger(),
      "[%s] front=%.2fm left=%.2fm right=%.2fm gap=%.2fm target=%.1fdeg cmd=(%.2f, %.2f)",
      output.mode.c_str(),
      output.scan.front_m,
      output.scan.left_m,
      output.scan.right_m,
      output.scan.best_gap_m,
      rad_to_deg(output.scan.best_angle_rad),
      output.linear_x,
      output.angular_z);
  }

  const std::string scan_topic_;
  const std::string cmd_vel_topic_;
  const bool cmd_vel_stamped_;
  const int control_period_ms_;
  const int report_period_ms_;
  const double max_linear_speed_;
  const double min_linear_speed_;
  const double max_angular_speed_;
  const double emergency_stop_distance_m_;
  const double front_stop_distance_m_;
  const double front_slow_distance_m_;
  const double side_stop_distance_m_;
  const double safe_gap_distance_m_;
  const double target_distance_cap_m_;
  const double search_angle_deg_;
  const double gap_window_deg_;
  const double sample_step_deg_;
  const double front_window_deg_;
  const double side_angle_deg_;
  const double side_window_deg_;
  const double heading_gain_;
  const double centering_gain_;
  const double forward_bias_weight_;
  const double turn_slowdown_gain_;
  const double recovery_turn_speed_;
  const double recovery_flip_seconds_;
  const double lost_scan_timeout_seconds_;
  const bool prefer_left_recovery_;

  // ROS2: /cmd_vel 타입이 환경마다 다를 수 있어 Twist와 TwistStamped 퍼블리셔 중 하나만 활성화한다.
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_publisher_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr cmd_vel_stamped_publisher_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_subscription_;
  rclcpp::TimerBase::SharedPtr control_timer_;
  rclcpp::TimerBase::SharedPtr report_timer_;

  std::mutex data_mutex_;
  sensor_msgs::msg::LaserScan::SharedPtr latest_scan_;
  rclcpp::Time latest_scan_time_;

  std::mutex output_mutex_;
  DriveOutput last_output_;

  bool recovery_active_ = false;
  int recovery_direction_ = 1;
  rclcpp::Time recovery_started_at_;
};

int main(int argc, char * argv[])
{
  // ROS2: rclcpp 초기화 후 노드를 spin해서 토픽 콜백과 timer 콜백을 실행한다.
  rclcpp::init(argc, argv);
  auto node = std::make_shared<TurtlebotAutonomousGapDrive>();
  rclcpp::spin(node);
  node->publish_stop_command();
  rclcpp::shutdown();
  return 0;
}
