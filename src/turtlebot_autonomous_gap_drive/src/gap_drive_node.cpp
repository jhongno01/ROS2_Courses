#include <signal.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <limits>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/magnetic_field.hpp"
#include "std_srvs/srv/set_bool.hpp"

using std::placeholders::_1;
using std::placeholders::_2;
using namespace std::chrono_literals;

namespace
{
volatile sig_atomic_t g_keep_running = 1;
constexpr double kPi = 3.14159265358979323846;
constexpr double kTwoPi = 2.0 * kPi;

void signal_handler(int)
{
  g_keep_running = 0;
}

double deg_to_rad(double degree)
{
  return degree * kPi / 180.0;
}

double rad_to_deg(double radian)
{
  return radian * 180.0 / kPi;
}

double normalize_angle_rad(double radian)
{
  double normalized = std::fmod(radian + kPi, kTwoPi);
  if (normalized < 0.0) {
    normalized += kTwoPi;
  }
  return normalized - kPi;
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

bool all_finite(double x, double y, double z)
{
  return std::isfinite(x) && std::isfinite(y) && std::isfinite(z);
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
  std::string detail;
  ScanSummary scan;
  bool enabled_requested = false;
  bool imu_ready = false;
  bool mag_ready = false;
  bool heading_reference_valid = false;
  double tilt_deg = 0.0;
  double heading_error_deg = 0.0;
};

class TurtlebotAutonomousGapDrive : public rclcpp::Node
{
public:
  TurtlebotAutonomousGapDrive()
  : Node("turtlebot_autonomous_gap_drive"),
    scan_topic_(this->declare_parameter<std::string>("scan_topic", "/scan")),
    imu_topic_(this->declare_parameter<std::string>("imu_topic", "/imu")),
    mag_topic_(this->declare_parameter<std::string>("mag_topic", "/mag")),
    cmd_vel_topic_(this->declare_parameter<std::string>("cmd_vel_topic", "/cmd_vel")),
    cmd_vel_stamped_(this->declare_parameter<bool>("cmd_vel_stamped", false)),
    auto_start_(this->declare_parameter<bool>("auto_start", false)),
    reset_heading_on_start_(this->declare_parameter<bool>("reset_heading_on_start", true)),
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
    prefer_left_recovery_(this->declare_parameter<bool>("prefer_left_recovery", true)),
    tilt_stop_deg_(this->declare_parameter<double>("tilt_stop_deg", 60.0)),
    imu_timeout_seconds_(this->declare_parameter<double>("imu_timeout_seconds", 1.0)),
    mag_timeout_seconds_(this->declare_parameter<double>("mag_timeout_seconds", 1.0)),
    reverse_heading_threshold_deg_(
      this->declare_parameter<double>("reverse_heading_threshold_deg", 150.0)),
    reverse_hold_seconds_(this->declare_parameter<double>("reverse_hold_seconds", 2.0)),
    reverse_linear_threshold_(this->declare_parameter<double>("reverse_linear_threshold", 0.02)),
    heading_recovery_turn_speed_(
      this->declare_parameter<double>("heading_recovery_turn_speed", 0.60)),
    reverse_resume_threshold_deg_(this->declare_parameter<double>("reverse_resume_threshold_deg", 45.0)),
    mag_yaw_sign_(this->declare_parameter<double>("mag_yaw_sign", 1.0)),
    mag_yaw_offset_deg_(this->declare_parameter<double>("mag_yaw_offset_deg", 0.0))
  {
    enabled_requested_ = auto_start_;

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

    imu_subscription_ = this->create_subscription<sensor_msgs::msg::Imu>(
      imu_topic_,
      rclcpp::SensorDataQoS(),
      std::bind(&TurtlebotAutonomousGapDrive::imu_callback, this, _1));

    mag_subscription_ = this->create_subscription<sensor_msgs::msg::MagneticField>(
      mag_topic_,
      rclcpp::SensorDataQoS(),
      std::bind(&TurtlebotAutonomousGapDrive::mag_callback, this, _1));

    set_enabled_service_ = this->create_service<std_srvs::srv::SetBool>(
      "/turtlebot_autonomous_gap_drive/set_enabled",
      std::bind(&TurtlebotAutonomousGapDrive::set_enabled_callback, this, _1, _2));

    // ROS2: wall timer로 일정 주기마다 최신 scan과 안전 센서를 해석하고 속도 명령을 만든다.
    control_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(control_period_ms_),
      std::bind(&TurtlebotAutonomousGapDrive::control_timer_callback, this));

    // ROS2: 별도 timer로 주행 상태를 낮은 빈도로 출력해서 터미널 로그를 읽기 쉽게 유지한다.
    report_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(report_period_ms_),
      std::bind(&TurtlebotAutonomousGapDrive::report_timer_callback, this));

    RCLCPP_INFO(
      this->get_logger(),
      "Started gap drive: scan=%s imu=%s mag=%s cmd=%s type=%s auto_start=%s max_v=%.2f",
      scan_topic_.c_str(),
      imu_topic_.c_str(),
      mag_topic_.c_str(),
      cmd_vel_topic_.c_str(),
      cmd_vel_stamped_ ? "TwistStamped" : "Twist",
      auto_start_ ? "true" : "false",
      max_linear_speed_);
    RCLCPP_INFO(
      this->get_logger(),
      "Use: ros2 service call /turtlebot_autonomous_gap_drive/set_enabled std_srvs/srv/SetBool \"{data: true}\"");
  }

  void publish_stop_command()
  {
    for (int i = 0; i < 5; ++i) {
      publish_velocity(0.0, 0.0);
      rclcpp::sleep_for(100ms);
    }
  }

private:
  struct SafetySnapshot
  {
    bool enabled_requested = false;
    bool imu_ready = false;
    bool mag_ready = false;
    bool heading_reference_valid = false;
    double roll_rad = 0.0;
    double pitch_rad = 0.0;
    double max_tilt_deg = 0.0;
    double current_heading_rad = 0.0;
    double heading_reference_rad = 0.0;
    rclcpp::Time latest_imu_time;
    rclcpp::Time latest_mag_time;
  };

  void set_enabled_callback(
    const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
    std::shared_ptr<std_srvs::srv::SetBool::Response> response)
  {
    const rclcpp::Time now = this->now();
    bool publish_stop = false;
    bool heading_reset = false;
    bool waiting_for_sensors = false;

    {
      std::lock_guard<std::mutex> lock(data_mutex_);

      if (request->data) {
        enabled_requested_ = true;
        reverse_condition_active_ = false;
        reverse_recovery_active_ = false;
        reset_recovery_state();

        if (reset_heading_on_start_ || !heading_reference_valid_) {
          heading_reference_valid_ = false;
          if (sensors_current_locked(now)) {
            set_heading_reference_locked();
            heading_reset = true;
          } else {
            waiting_for_sensors = true;
          }
        }
      } else {
        enabled_requested_ = false;
        heading_reference_valid_ = false;
        reverse_condition_active_ = false;
        reverse_recovery_active_ = false;
        reset_recovery_state();
        publish_stop = true;
      }
    }

    if (request->data) {
      response->success = true;
      if (heading_reset) {
        response->message = "Gap drive enabled. Heading reference reset from current magnetometer heading.";
      } else if (waiting_for_sensors) {
        response->message = "Gap drive start requested. Waiting for current IMU and magnetometer data.";
      } else {
        response->message = "Gap drive enabled.";
      }
    } else {
      response->success = true;
      response->message = "Gap drive stopped. Publishing zero velocity.";
      DriveOutput output;
      output.mode = "STOPPED";
      output.detail = "service stop";
      store_output(output);
    }

    if (publish_stop) {
      publish_velocity(0.0, 0.0);
    }
  }

  void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    latest_scan_ = msg;
    latest_scan_time_ = this->now();
  }

  void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
  {
    const double norm = std::sqrt(
      msg->orientation.x * msg->orientation.x +
      msg->orientation.y * msg->orientation.y +
      msg->orientation.z * msg->orientation.z +
      msg->orientation.w * msg->orientation.w);

    if (!std::isfinite(norm) || norm < 1e-6) {
      std::lock_guard<std::mutex> lock(data_mutex_);
      imu_ready_ = false;
      return;
    }

    const double x = msg->orientation.x / norm;
    const double y = msg->orientation.y / norm;
    const double z = msg->orientation.z / norm;
    const double w = msg->orientation.w / norm;

    const double sinr_cosp = 2.0 * (w * x + y * z);
    const double cosr_cosp = 1.0 - 2.0 * (x * x + y * y);
    const double roll_rad = std::atan2(sinr_cosp, cosr_cosp);

    const double sinp = 2.0 * (w * y - z * x);
    const double pitch_rad =
      std::fabs(sinp) >= 1.0 ? std::copysign(kPi / 2.0, sinp) : std::asin(sinp);

    std::lock_guard<std::mutex> lock(data_mutex_);
    roll_rad_ = roll_rad;
    pitch_rad_ = pitch_rad;
    max_tilt_deg_ = std::max(std::fabs(rad_to_deg(roll_rad_)), std::fabs(rad_to_deg(pitch_rad_)));
    latest_imu_time_ = this->now();
    imu_ready_ = true;
  }

  void mag_callback(const sensor_msgs::msg::MagneticField::SharedPtr msg)
  {
    const double mx = msg->magnetic_field.x;
    const double my = msg->magnetic_field.y;
    const double mz = msg->magnetic_field.z;

    std::lock_guard<std::mutex> lock(data_mutex_);
    if (!all_finite(mx, my, mz) || !imu_ready_) {
      mag_ready_ = false;
      return;
    }

    current_heading_rad_ = compute_tilt_compensated_heading(mx, my, mz, roll_rad_, pitch_rad_);
    latest_mag_time_ = this->now();
    mag_ready_ = true;
  }

  void control_timer_callback()
  {
    const rclcpp::Time now = this->now();
    SafetySnapshot safety = get_safety_snapshot();

    DriveOutput output;
    annotate_output(output, safety);

    if (!safety.enabled_requested) {
      output.mode = "STOPPED";
      output.detail = "waiting for set_enabled true";
      publish_velocity(0.0, 0.0);
      store_output(output);
      return;
    }

    std::string wait_reason;
    if (!safety_data_ready(safety, now, wait_reason)) {
      output.mode = "WAIT_SENSORS";
      output.detail = wait_reason;
      publish_velocity(0.0, 0.0);
      store_output(output);
      return;
    }

    if (safety.max_tilt_deg > tilt_stop_deg_) {
      output.mode = "TILT_STOP";
      output.detail = "tilt limit exceeded";
      publish_velocity(0.0, 0.0);
      store_output(output);
      return;
    }

    if (!safety.heading_reference_valid) {
      if (!reset_heading_reference_from_current()) {
        output.mode = "WAIT_SENSORS";
        output.detail = "waiting for heading reference";
        publish_velocity(0.0, 0.0);
        store_output(output);
        return;
      }
      safety = get_safety_snapshot();
      annotate_output(output, safety);
    }

    sensor_msgs::msg::LaserScan::SharedPtr scan;
    rclcpp::Time scan_time;
    {
      std::lock_guard<std::mutex> lock(data_mutex_);
      scan = latest_scan_;
      scan_time = latest_scan_time_;
    }

    if (!scan || scan->ranges.empty()) {
      output.mode = "WAIT_SCAN";
      output.detail = "waiting for lidar scan";
      publish_velocity(0.0, 0.0);
      store_output(output);
      return;
    }

    const double scan_age = (now - scan_time).seconds();
    if (scan_age > lost_scan_timeout_seconds_) {
      output.mode = "SCAN_TIMEOUT";
      output.detail = "lidar scan timeout";
      publish_velocity(0.0, 0.0);
      store_output(output);
      return;
    }

    output = compute_drive_output(*scan);
    annotate_output(output, get_safety_snapshot());
    apply_reverse_guard(output, now);

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

  double compute_tilt_compensated_heading(
    double mx,
    double my,
    double mz,
    double roll_rad,
    double pitch_rad) const
  {
    const double cos_roll = std::cos(roll_rad);
    const double sin_roll = std::sin(roll_rad);
    const double cos_pitch = std::cos(pitch_rad);
    const double sin_pitch = std::sin(pitch_rad);

    const double compensated_x = mx * cos_pitch + mz * sin_pitch;
    const double compensated_y =
      mx * sin_roll * sin_pitch + my * cos_roll - mz * sin_roll * cos_pitch;
    const double raw_heading = std::atan2(compensated_y, compensated_x);

    return normalize_angle_rad(mag_yaw_sign_ * raw_heading + deg_to_rad(mag_yaw_offset_deg_));
  }

  bool sensors_current_locked(const rclcpp::Time & now) const
  {
    return imu_ready_ &&
           mag_ready_ &&
           (now - latest_imu_time_).seconds() <= imu_timeout_seconds_ &&
           (now - latest_mag_time_).seconds() <= mag_timeout_seconds_;
  }

  bool reset_heading_reference_from_current()
  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    if (!mag_ready_) {
      return false;
    }

    set_heading_reference_locked();
    RCLCPP_INFO(
      this->get_logger(),
      "Heading reference reset: %.1f deg",
      rad_to_deg(heading_reference_rad_));
    return true;
  }

  void set_heading_reference_locked()
  {
    heading_reference_rad_ = current_heading_rad_;
    heading_reference_valid_ = true;
    reverse_condition_active_ = false;
    reverse_recovery_active_ = false;
  }

  SafetySnapshot get_safety_snapshot()
  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    SafetySnapshot snapshot;
    snapshot.enabled_requested = enabled_requested_;
    snapshot.imu_ready = imu_ready_;
    snapshot.mag_ready = mag_ready_;
    snapshot.heading_reference_valid = heading_reference_valid_;
    snapshot.roll_rad = roll_rad_;
    snapshot.pitch_rad = pitch_rad_;
    snapshot.max_tilt_deg = max_tilt_deg_;
    snapshot.current_heading_rad = current_heading_rad_;
    snapshot.heading_reference_rad = heading_reference_rad_;
    snapshot.latest_imu_time = latest_imu_time_;
    snapshot.latest_mag_time = latest_mag_time_;
    return snapshot;
  }

  bool safety_data_ready(
    const SafetySnapshot & safety,
    const rclcpp::Time & now,
    std::string & wait_reason) const
  {
    if (!safety.imu_ready) {
      wait_reason = "waiting for imu";
      return false;
    }
    if (!safety.mag_ready) {
      wait_reason = "waiting for magnetometer";
      return false;
    }
    if ((now - safety.latest_imu_time).seconds() > imu_timeout_seconds_) {
      wait_reason = "imu timeout";
      return false;
    }
    if ((now - safety.latest_mag_time).seconds() > mag_timeout_seconds_) {
      wait_reason = "magnetometer timeout";
      return false;
    }
    return true;
  }

  void annotate_output(DriveOutput & output, const SafetySnapshot & safety) const
  {
    output.enabled_requested = safety.enabled_requested;
    output.imu_ready = safety.imu_ready;
    output.mag_ready = safety.mag_ready;
    output.heading_reference_valid = safety.heading_reference_valid;
    output.tilt_deg = safety.max_tilt_deg;

    if (safety.mag_ready && safety.heading_reference_valid) {
      output.heading_error_deg =
        rad_to_deg(normalize_angle_rad(safety.current_heading_rad - safety.heading_reference_rad));
    }
  }

  void apply_reverse_guard(DriveOutput & output, const rclcpp::Time & now)
  {
    double heading_error_rad = 0.0;
    double heading_error_abs_deg = 0.0;
    bool run_recovery = false;
    bool recovery_resumed = false;

    {
      std::lock_guard<std::mutex> lock(data_mutex_);
      if (!mag_ready_ || !heading_reference_valid_) {
        return;
      }

      heading_error_rad = normalize_angle_rad(current_heading_rad_ - heading_reference_rad_);
      heading_error_abs_deg = std::fabs(rad_to_deg(heading_error_rad));
      output.heading_error_deg = rad_to_deg(heading_error_rad);

      if (reverse_recovery_active_) {
        if (heading_error_abs_deg <= reverse_resume_threshold_deg_) {
          reverse_recovery_active_ = false;
          reverse_condition_active_ = false;
          recovery_resumed = true;
        } else {
          run_recovery = true;
        }
      } else if (
        output.linear_x > reverse_linear_threshold_ &&
        heading_error_abs_deg >= reverse_heading_threshold_deg_)
      {
        if (!reverse_condition_active_) {
          reverse_condition_active_ = true;
          reverse_condition_started_at_ = now;
        }

        if ((now - reverse_condition_started_at_).seconds() >= reverse_hold_seconds_) {
          reverse_recovery_active_ = true;
          run_recovery = true;
        } else {
          output.mode = "REVERSE_WARN";
        }
      } else {
        reverse_condition_active_ = false;
      }
    }

    if (recovery_resumed) {
      return;
    }

    if (run_recovery) {
      const double recovery_direction = heading_error_rad > 0.0 ? -1.0 : 1.0;
      output.linear_x = 0.0;
      output.angular_z = recovery_direction * heading_recovery_turn_speed_;
      output.mode = "REVERSE_RECOVERY";
      output.detail = "turning back to start heading";
      reset_recovery_state();
    }
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
      "[%s] enabled=%s imu=%s mag=%s ref=%s tilt=%.1fdeg heading_err=%.1fdeg "
      "front=%.2fm left=%.2fm right=%.2fm gap=%.2fm target=%.1fdeg cmd=(%.2f, %.2f) %s",
      output.mode.c_str(),
      output.enabled_requested ? "yes" : "no",
      output.imu_ready ? "yes" : "no",
      output.mag_ready ? "yes" : "no",
      output.heading_reference_valid ? "yes" : "no",
      output.tilt_deg,
      output.heading_error_deg,
      output.scan.front_m,
      output.scan.left_m,
      output.scan.right_m,
      output.scan.best_gap_m,
      rad_to_deg(output.scan.best_angle_rad),
      output.linear_x,
      output.angular_z,
      output.detail.c_str());
  }

  const std::string scan_topic_;
  const std::string imu_topic_;
  const std::string mag_topic_;
  const std::string cmd_vel_topic_;
  const bool cmd_vel_stamped_;
  const bool auto_start_;
  const bool reset_heading_on_start_;
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
  const double tilt_stop_deg_;
  const double imu_timeout_seconds_;
  const double mag_timeout_seconds_;
  const double reverse_heading_threshold_deg_;
  const double reverse_hold_seconds_;
  const double reverse_linear_threshold_;
  const double heading_recovery_turn_speed_;
  const double reverse_resume_threshold_deg_;
  const double mag_yaw_sign_;
  const double mag_yaw_offset_deg_;

  // ROS2: /cmd_vel 타입이 환경마다 다를 수 있어 Twist와 TwistStamped 퍼블리셔 중 하나만 활성화한다.
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_publisher_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr cmd_vel_stamped_publisher_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_subscription_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscription_;
  rclcpp::Subscription<sensor_msgs::msg::MagneticField>::SharedPtr mag_subscription_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr set_enabled_service_;
  rclcpp::TimerBase::SharedPtr control_timer_;
  rclcpp::TimerBase::SharedPtr report_timer_;

  std::mutex data_mutex_;
  sensor_msgs::msg::LaserScan::SharedPtr latest_scan_;
  rclcpp::Time latest_scan_time_;
  rclcpp::Time latest_imu_time_;
  rclcpp::Time latest_mag_time_;
  bool enabled_requested_ = false;
  bool imu_ready_ = false;
  bool mag_ready_ = false;
  bool heading_reference_valid_ = false;
  bool reverse_condition_active_ = false;
  bool reverse_recovery_active_ = false;
  double roll_rad_ = 0.0;
  double pitch_rad_ = 0.0;
  double max_tilt_deg_ = 0.0;
  double current_heading_rad_ = 0.0;
  double heading_reference_rad_ = 0.0;
  rclcpp::Time reverse_condition_started_at_;

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
  signal(SIGINT, signal_handler);
  signal(SIGTERM, signal_handler);

  auto node = std::make_shared<TurtlebotAutonomousGapDrive>();
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);

  while (rclcpp::ok() && g_keep_running) {
    executor.spin_some();
    std::this_thread::sleep_for(10ms);
  }

  node->publish_stop_command();

  if (rclcpp::ok()) {
    rclcpp::shutdown();
  }
  return 0;
}
