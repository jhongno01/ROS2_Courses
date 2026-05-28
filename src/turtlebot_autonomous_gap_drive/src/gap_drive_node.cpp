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
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
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

}  // namespace

struct ScanSummary
{
  double front_m = 0.0;
  double left_m = 0.0;
  double right_m = 0.0;
  double best_gap_m = 0.0;
  double best_center_m = 0.0;
  double best_gap_width_m = 0.0;
  double best_angle_rad = 0.0;
  bool has_safe_gap = false;
  bool has_gap_width = false;
  bool best_is_narrow_passage = false;
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
  bool odom_ready = false;
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
    odom_topic_(this->declare_parameter<std::string>("odom_topic", "/odom")),
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
    heading_alignment_weight_(this->declare_parameter<double>("heading_alignment_weight", 0.35)),
    turn_slowdown_gain_(this->declare_parameter<double>("turn_slowdown_gain", 0.65)),
    wall_assist_enabled_(this->declare_parameter<bool>("wall_assist_enabled", true)),
    wall_assist_side_detect_m_(this->declare_parameter<double>("wall_assist_side_detect_m", 0.42)),
    wall_assist_target_distance_m_(
      this->declare_parameter<double>("wall_assist_target_distance_m", 0.16)),
    wall_assist_gain_(this->declare_parameter<double>("wall_assist_gain", 1.8)),
    wall_assist_max_angular_speed_(
      this->declare_parameter<double>("wall_assist_max_angular_speed", 0.55)),
    wall_assist_hard_stop_distance_m_(
      this->declare_parameter<double>("wall_assist_hard_stop_distance_m", 0.09)),
    wall_assist_gap_width_max_m_(
      this->declare_parameter<double>("wall_assist_gap_width_max_m", 0.36)),
    narrow_passage_enabled_(this->declare_parameter<bool>("narrow_passage_enabled", true)),
    narrow_passage_min_center_distance_m_(
      this->declare_parameter<double>("narrow_passage_min_center_distance_m", 0.30)),
    narrow_passage_min_sector_distance_m_(
      this->declare_parameter<double>("narrow_passage_min_sector_distance_m", 0.10)),
    narrow_passage_max_angle_deg_(
      this->declare_parameter<double>("narrow_passage_max_angle_deg", 35.0)),
    narrow_passage_max_width_m_(
      this->declare_parameter<double>("narrow_passage_max_width_m", 0.42)),
    narrow_passage_min_depth_gain_m_(
      this->declare_parameter<double>("narrow_passage_min_depth_gain_m", 0.05)),
    narrow_passage_bonus_(this->declare_parameter<double>("narrow_passage_bonus", 0.75)),
    narrow_passage_speed_m_s_(
      this->declare_parameter<double>("narrow_passage_speed_m_s", 0.04)),
    recovery_turn_speed_(this->declare_parameter<double>("recovery_turn_speed", 0.75)),
    recovery_flip_seconds_(this->declare_parameter<double>("recovery_flip_seconds", 4.0)),
    lost_scan_timeout_seconds_(this->declare_parameter<double>("lost_scan_timeout_seconds", 1.0)),
    prefer_left_recovery_(this->declare_parameter<bool>("prefer_left_recovery", true)),
    tilt_stop_deg_(this->declare_parameter<double>("tilt_stop_deg", 60.0)),
    imu_timeout_seconds_(this->declare_parameter<double>("imu_timeout_seconds", 1.0)),
    gyro_yaw_sign_(this->declare_parameter<double>("gyro_yaw_sign", 1.0)),
    gyro_angular_deadband_rad_s_(
      this->declare_parameter<double>("gyro_angular_deadband_rad_s", 0.01)),
    gyro_integration_max_dt_seconds_(
      this->declare_parameter<double>("gyro_integration_max_dt_seconds", 0.25)),
    odom_timeout_seconds_(this->declare_parameter<double>("odom_timeout_seconds", 1.0)),
    reverse_heading_threshold_deg_(
      this->declare_parameter<double>("reverse_heading_threshold_deg", 150.0)),
    reverse_hold_seconds_(this->declare_parameter<double>("reverse_hold_seconds", 2.0)),
    reverse_linear_threshold_(this->declare_parameter<double>("reverse_linear_threshold", 0.02)),
    heading_recovery_turn_speed_(
      this->declare_parameter<double>("heading_recovery_turn_speed", 0.60)),
    reverse_resume_threshold_deg_(this->declare_parameter<double>("reverse_resume_threshold_deg", 45.0)),
    stuck_recovery_enabled_(this->declare_parameter<bool>("stuck_recovery_enabled", true)),
    stuck_command_linear_threshold_(
      this->declare_parameter<double>("stuck_command_linear_threshold", 0.03)),
    stuck_odom_linear_threshold_(
      this->declare_parameter<double>("stuck_odom_linear_threshold", 0.01)),
    stuck_min_progress_m_(this->declare_parameter<double>("stuck_min_progress_m", 0.03)),
    stuck_hold_seconds_(this->declare_parameter<double>("stuck_hold_seconds", 3.0)),
    stuck_backup_speed_(this->declare_parameter<double>("stuck_backup_speed", 0.04)),
    stuck_backup_seconds_(this->declare_parameter<double>("stuck_backup_seconds", 0.8)),
    stuck_turn_speed_(this->declare_parameter<double>("stuck_turn_speed", 0.60)),
    stuck_turn_seconds_(this->declare_parameter<double>("stuck_turn_seconds", 1.0)),
    stuck_cooldown_seconds_(this->declare_parameter<double>("stuck_cooldown_seconds", 2.0)),
    enforce_gap_width_(this->declare_parameter<bool>("enforce_gap_width", true)),
    min_passage_width_m_(this->declare_parameter<double>("min_passage_width_m", 0.24)),
    gap_width_search_deg_(this->declare_parameter<double>("gap_width_search_deg", 45.0)),
    gap_width_obstacle_max_range_m_(
      this->declare_parameter<double>("gap_width_obstacle_max_range_m", 1.20)),
    gap_width_boundary_drop_m_(
      this->declare_parameter<double>("gap_width_boundary_drop_m", 0.05))
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

    odom_subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
      odom_topic_,
      rclcpp::SensorDataQoS(),
      std::bind(&TurtlebotAutonomousGapDrive::odom_callback, this, _1));

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
      "Started gap drive: scan=%s imu=%s odom=%s cmd=%s type=%s auto_start=%s max_v=%.2f",
      scan_topic_.c_str(),
      imu_topic_.c_str(),
      odom_topic_.c_str(),
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
    bool odom_ready = false;
    bool heading_reference_valid = false;
    double roll_rad = 0.0;
    double pitch_rad = 0.0;
    double max_tilt_deg = 0.0;
    double current_heading_rad = 0.0;
    double heading_reference_rad = 0.0;
    rclcpp::Time latest_imu_time;
    rclcpp::Time latest_odom_time;
  };

  struct GapWidthMeasurement
  {
    bool measured = false;
    double width_m = 0.0;
    double left_angle_rad = 0.0;
    double left_range_m = 0.0;
    double right_angle_rad = 0.0;
    double right_range_m = 0.0;
  };

  enum class StuckRecoveryPhase
  {
    IDLE,
    BACKUP,
    TURN
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
        stuck_candidate_active_ = false;
        stuck_recovery_phase_ = StuckRecoveryPhase::IDLE;
        reset_recovery_state();
        publish_stop = true;
      }
    }

    if (request->data) {
      response->success = true;
      if (heading_reset) {
        response->message = "Gap drive enabled. Gyro yaw reference reset.";
      } else if (waiting_for_sensors) {
        response->message = "Gap drive start requested. Waiting for current IMU data.";
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
    const double angular_velocity_z = msg->angular_velocity.z;
    const double norm = std::sqrt(
      msg->orientation.x * msg->orientation.x +
      msg->orientation.y * msg->orientation.y +
      msg->orientation.z * msg->orientation.z +
      msg->orientation.w * msg->orientation.w);

    if (!std::isfinite(norm) || norm < 1e-6 || !std::isfinite(angular_velocity_z)) {
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

    const rclcpp::Time now = this->now();

    std::lock_guard<std::mutex> lock(data_mutex_);
    if (imu_ready_) {
      const double dt = (now - latest_imu_time_).seconds();
      if (dt > 0.0 && dt <= gyro_integration_max_dt_seconds_) {
        const double gyro_z =
          std::fabs(angular_velocity_z) < gyro_angular_deadband_rad_s_ ? 0.0 : angular_velocity_z;
        current_heading_rad_ =
          normalize_angle_rad(current_heading_rad_ + gyro_yaw_sign_ * gyro_z * dt);
      }
    }

    roll_rad_ = roll_rad;
    pitch_rad_ = pitch_rad;
    max_tilt_deg_ = std::max(std::fabs(rad_to_deg(roll_rad_)), std::fabs(rad_to_deg(pitch_rad_)));
    latest_imu_time_ = now;
    imu_ready_ = true;
  }

  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    const double x = msg->pose.pose.position.x;
    const double y = msg->pose.pose.position.y;
    const double vx = msg->twist.twist.linear.x;
    const double vy = msg->twist.twist.linear.y;

    std::lock_guard<std::mutex> lock(data_mutex_);
    if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(vx) || !std::isfinite(vy)) {
      odom_ready_ = false;
      return;
    }

    odom_x_ = x;
    odom_y_ = y;
    odom_linear_speed_ = std::sqrt(vx * vx + vy * vy);
    latest_odom_time_ = this->now();
    odom_ready_ = true;
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

    safety = get_safety_snapshot();
    output = compute_drive_output(*scan, safety);
    annotate_output(output, safety);
    apply_reverse_guard(output, now);
    apply_stuck_recovery(output, now);

    publish_velocity(output.linear_x, output.angular_z);
    store_output(output);
  }

  DriveOutput compute_drive_output(
    const sensor_msgs::msg::LaserScan & scan,
    const SafetySnapshot & safety)
  {
    DriveOutput output;
    output.scan = summarize_scan(scan, safety);

    const double left = output.scan.left_m;
    const double right = output.scan.right_m;
    const double front = output.scan.front_m;
    const bool narrow_passage = output.scan.best_is_narrow_passage;
    const bool narrow_front_entry =
      narrow_passage && std::fabs(rad_to_deg(output.scan.best_angle_rad)) <= front_window_deg_;

    if (front <= emergency_stop_distance_m_ &&
      (!narrow_front_entry || output.scan.best_center_m <= narrow_passage_min_center_distance_m_))
    {
      output.linear_x = 0.0;
      output.angular_z = choose_recovery_turn(left, right);
      output.mode = "EMERGENCY_TURN";
      return output;
    }

    if ((front <= front_stop_distance_m_ && !narrow_front_entry) || !output.scan.has_safe_gap) {
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

    double wall_assist_cmd = 0.0;
    std::string wall_assist_detail;
    const bool wall_assist_active =
      compute_wall_assist_command(output.scan, wall_assist_cmd, wall_assist_detail);

    double angular_z = heading_cmd + (wall_assist_active ? wall_assist_cmd : centering_cmd);

    const double hard_side_stop =
      wall_assist_active ? wall_assist_hard_stop_distance_m_ : side_stop_distance_m_;
    if (left < hard_side_stop && right > left) {
      angular_z = -std::max(0.45, std::fabs(angular_z));
    } else if (right < hard_side_stop && left > right) {
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
    const double max_forward_speed =
      narrow_passage ? std::max(min_linear_speed_, narrow_passage_speed_m_s_) : max_linear_speed_;

    output.linear_x = clamp_value(base_speed * turn_scale, min_linear_speed_, max_forward_speed);
    if (wall_assist_active) {
      output.mode = "WALL_ASSIST";
      output.detail = wall_assist_detail;
    } else if (narrow_passage) {
      output.mode = "NARROW_GAP_DRIVE";
      output.detail = "width-valid narrow passage";
    } else {
      output.mode = "GAP_DRIVE";
    }
    return output;
  }

  bool compute_wall_assist_command(
    const ScanSummary & scan,
    double & wall_cmd,
    std::string & detail) const
  {
    wall_cmd = 0.0;
    detail.clear();

    if (!wall_assist_enabled_) {
      return false;
    }

    const bool both_sides_near =
      scan.left_m <= wall_assist_side_detect_m_ &&
      scan.right_m <= wall_assist_side_detect_m_;
    const bool narrow_measured_gap =
      scan.has_gap_width &&
      scan.best_gap_width_m >= min_passage_width_m_ &&
      scan.best_gap_width_m <= wall_assist_gap_width_max_m_;

    if (!both_sides_near && !narrow_measured_gap) {
      return false;
    }

    const bool follow_left = scan.left_m <= scan.right_m;
    const double wall_distance = follow_left ? scan.left_m : scan.right_m;
    if (wall_distance > wall_assist_side_detect_m_) {
      return false;
    }

    const double distance_error = wall_distance - wall_assist_target_distance_m_;
    const double direction = follow_left ? 1.0 : -1.0;
    wall_cmd = clamp_value(
      direction * wall_assist_gain_ * distance_error,
      -wall_assist_max_angular_speed_,
      wall_assist_max_angular_speed_);
    detail = follow_left ? "dynamic left wall assist" : "dynamic right wall assist";
    return true;
  }

  ScanSummary summarize_scan(
    const sensor_msgs::msg::LaserScan & scan,
    const SafetySnapshot & safety) const
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

      double center_range = 0.0;
      if (!range_at_angle(scan, deg_to_rad(angle_deg), center_range)) {
        continue;
      }
      center_range = sanitize_range(scan, center_range);

      const GapWidthMeasurement gap_width = measure_gap_width(scan, angle_deg, center_range);
      if (!gap_width_allows(gap_width)) {
        continue;
      }

      const bool normal_gap = clearance >= safe_gap_distance_m_;
      const bool narrow_passage =
        narrow_passage_allows(angle_deg, center_range, clearance, gap_width);
      if (!normal_gap && !narrow_passage) {
        continue;
      }

      const double score_clearance = narrow_passage ? center_range : clearance;
      const double capped_clearance = std::min(score_clearance, target_distance_cap_m_);
      const double forward_penalty =
        forward_bias_weight_ * std::fabs(angle_deg) / std::max(1.0, search_angle_deg_);
      const double heading_bonus =
        compute_heading_alignment_bonus(deg_to_rad(angle_deg), safety);
      const double narrow_bonus = narrow_passage ? narrow_passage_bonus_ : 0.0;
      const double score = capped_clearance + heading_bonus + narrow_bonus - forward_penalty;

      if (score > best_score) {
        best_score = score;
        summary.best_gap_m = clearance;
        summary.best_center_m = center_range;
        summary.best_gap_width_m = gap_width.width_m;
        summary.best_angle_rad = deg_to_rad(angle_deg);
        summary.has_safe_gap = true;
        summary.has_gap_width = gap_width.measured;
        summary.best_is_narrow_passage = narrow_passage;
      }
    }

    return summary;
  }

  double compute_heading_alignment_bonus(
    double candidate_angle_rad,
    const SafetySnapshot & safety) const
  {
    if (!safety.heading_reference_valid || heading_alignment_weight_ <= 0.0) {
      return 0.0;
    }

    const double candidate_heading_error = normalize_angle_rad(
      safety.current_heading_rad + candidate_angle_rad - safety.heading_reference_rad);
    const double error_ratio = clamp_value(std::fabs(candidate_heading_error) / kPi, 0.0, 1.0);
    return heading_alignment_weight_ * (1.0 - error_ratio);
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

  GapWidthMeasurement measure_gap_width(
    const sensor_msgs::msg::LaserScan & scan,
    double center_deg,
    double center_range_m) const
  {
    GapWidthMeasurement measurement;

    const bool has_left = find_gap_boundary(
      scan, center_deg, center_range_m, 1.0, measurement.left_angle_rad,
      measurement.left_range_m);
    const bool has_right = find_gap_boundary(
      scan, center_deg, center_range_m, -1.0, measurement.right_angle_rad,
      measurement.right_range_m);

    if (!has_left || !has_right) {
      return measurement;
    }

    const double angle_between = std::fabs(
      normalize_angle_rad(measurement.left_angle_rad - measurement.right_angle_rad));
    const double width_squared =
      measurement.left_range_m * measurement.left_range_m +
      measurement.right_range_m * measurement.right_range_m -
      2.0 * measurement.left_range_m * measurement.right_range_m * std::cos(angle_between);

    measurement.width_m = std::sqrt(std::max(0.0, width_squared));
    measurement.measured = true;
    return measurement;
  }

  bool gap_width_allows(const GapWidthMeasurement & measurement) const
  {
    if (!enforce_gap_width_ || !measurement.measured) {
      return true;
    }

    return measurement.width_m >= min_passage_width_m_;
  }

  bool narrow_passage_allows(
    double center_deg,
    double center_range_m,
    double sector_clearance_m,
    const GapWidthMeasurement & measurement) const
  {
    if (!narrow_passage_enabled_ || !measurement.measured) {
      return false;
    }

    if (std::fabs(center_deg) > narrow_passage_max_angle_deg_) {
      return false;
    }

    if (measurement.width_m < min_passage_width_m_ ||
      measurement.width_m > narrow_passage_max_width_m_)
    {
      return false;
    }

    if (center_range_m < narrow_passage_min_center_distance_m_ ||
      sector_clearance_m < narrow_passage_min_sector_distance_m_)
    {
      return false;
    }

    const double side_mean_range =
      0.5 * (measurement.left_range_m + measurement.right_range_m);
    return center_range_m >= side_mean_range + narrow_passage_min_depth_gain_m_;
  }

  bool find_gap_boundary(
    const sensor_msgs::msg::LaserScan & scan,
    double center_deg,
    double center_range_m,
    double direction,
    double & boundary_angle_rad,
    double & boundary_range_m) const
  {
    const double step_deg = std::max(1.0, sample_step_deg_);
    for (double offset_deg = step_deg;
      offset_deg <= gap_width_search_deg_;
      offset_deg += step_deg)
    {
      const double angle_rad = deg_to_rad(center_deg + direction * offset_deg);
      double range = 0.0;
      if (!range_at_angle(scan, angle_rad, range)) {
        continue;
      }

      const double sanitized = sanitize_range(scan, range);
      const bool close_enough = sanitized <= gap_width_obstacle_max_range_m_;
      const bool distinct_from_center =
        sanitized <= center_range_m - std::max(0.0, gap_width_boundary_drop_m_);
      if (close_enough && distinct_from_center) {
        boundary_angle_rad = angle_rad;
        boundary_range_m = sanitized;
        return true;
      }
    }

    return false;
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

  bool sensors_current_locked(const rclcpp::Time & now) const
  {
    return imu_ready_ &&
           (now - latest_imu_time_).seconds() <= imu_timeout_seconds_;
  }

  bool reset_heading_reference_from_current()
  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    if (!imu_ready_) {
      return false;
    }

    set_heading_reference_locked();
    RCLCPP_INFO(
      this->get_logger(),
      "Gyro yaw reference reset");
    return true;
  }

  void set_heading_reference_locked()
  {
    current_heading_rad_ = 0.0;
    heading_reference_rad_ = 0.0;
    heading_reference_valid_ = true;
    reverse_condition_active_ = false;
    reverse_recovery_active_ = false;
    stuck_candidate_active_ = false;
    stuck_recovery_phase_ = StuckRecoveryPhase::IDLE;
  }

  SafetySnapshot get_safety_snapshot()
  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    SafetySnapshot snapshot;
    snapshot.enabled_requested = enabled_requested_;
    snapshot.imu_ready = imu_ready_;
    snapshot.odom_ready = odom_ready_;
    snapshot.heading_reference_valid = heading_reference_valid_;
    snapshot.roll_rad = roll_rad_;
    snapshot.pitch_rad = pitch_rad_;
    snapshot.max_tilt_deg = max_tilt_deg_;
    snapshot.current_heading_rad = current_heading_rad_;
    snapshot.heading_reference_rad = heading_reference_rad_;
    snapshot.latest_imu_time = latest_imu_time_;
    snapshot.latest_odom_time = latest_odom_time_;
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
    if ((now - safety.latest_imu_time).seconds() > imu_timeout_seconds_) {
      wait_reason = "imu timeout";
      return false;
    }
    return true;
  }

  void annotate_output(DriveOutput & output, const SafetySnapshot & safety) const
  {
    output.enabled_requested = safety.enabled_requested;
    output.imu_ready = safety.imu_ready;
    output.odom_ready = safety.odom_ready;
    output.heading_reference_valid = safety.heading_reference_valid;
    output.tilt_deg = safety.max_tilt_deg;

    if (safety.heading_reference_valid) {
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
      if (!heading_reference_valid_) {
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

  void apply_stuck_recovery(DriveOutput & output, const rclcpp::Time & now)
  {
    if (!stuck_recovery_enabled_) {
      return;
    }

    if (stuck_recovery_phase_ != StuckRecoveryPhase::IDLE) {
      update_stuck_recovery_command(output, now);
      return;
    }

    if (stuck_recovery_finished_once_ &&
      (now - last_stuck_recovery_finished_at_).seconds() < stuck_cooldown_seconds_)
    {
      return;
    }

    if (output.linear_x <= stuck_command_linear_threshold_) {
      stuck_candidate_active_ = false;
      return;
    }

    bool odom_ready = false;
    double odom_age = 0.0;
    double odom_x = 0.0;
    double odom_y = 0.0;
    double odom_linear_speed = 0.0;

    {
      std::lock_guard<std::mutex> lock(data_mutex_);
      odom_ready = odom_ready_;
      odom_age = (now - latest_odom_time_).seconds();
      odom_x = odom_x_;
      odom_y = odom_y_;
      odom_linear_speed = odom_linear_speed_;
    }

    if (!odom_ready || odom_age > odom_timeout_seconds_) {
      stuck_candidate_active_ = false;
      return;
    }

    if (odom_linear_speed > stuck_odom_linear_threshold_) {
      stuck_candidate_active_ = false;
      return;
    }

    if (!stuck_candidate_active_) {
      stuck_candidate_active_ = true;
      stuck_candidate_started_at_ = now;
      stuck_candidate_x_ = odom_x;
      stuck_candidate_y_ = odom_y;
      return;
    }

    const double dx = odom_x - stuck_candidate_x_;
    const double dy = odom_y - stuck_candidate_y_;
    const double progress = std::sqrt(dx * dx + dy * dy);
    if (progress > stuck_min_progress_m_) {
      stuck_candidate_active_ = false;
      return;
    }

    if ((now - stuck_candidate_started_at_).seconds() >= stuck_hold_seconds_) {
      start_stuck_recovery(output, now);
      update_stuck_recovery_command(output, now);
    }
  }

  void start_stuck_recovery(const DriveOutput & output, const rclcpp::Time & now)
  {
    stuck_candidate_active_ = false;
    stuck_recovery_phase_ = StuckRecoveryPhase::BACKUP;
    stuck_phase_started_at_ = now;
    stuck_turn_direction_ = choose_stuck_turn_direction(output.scan);
    reverse_condition_active_ = false;
    reverse_recovery_active_ = false;
    reset_recovery_state();
  }

  int choose_stuck_turn_direction(const ScanSummary & scan) const
  {
    if (std::fabs(scan.left_m - scan.right_m) < 0.05) {
      return prefer_left_recovery_ ? 1 : -1;
    }
    return scan.left_m > scan.right_m ? 1 : -1;
  }

  void update_stuck_recovery_command(DriveOutput & output, const rclcpp::Time & now)
  {
    double phase_elapsed = (now - stuck_phase_started_at_).seconds();

    if (stuck_recovery_phase_ == StuckRecoveryPhase::BACKUP &&
      phase_elapsed >= stuck_backup_seconds_)
    {
      stuck_recovery_phase_ = StuckRecoveryPhase::TURN;
      stuck_phase_started_at_ = now;
      phase_elapsed = 0.0;
    }

    if (stuck_recovery_phase_ == StuckRecoveryPhase::TURN &&
      phase_elapsed >= stuck_turn_seconds_)
    {
      stuck_recovery_phase_ = StuckRecoveryPhase::IDLE;
      stuck_recovery_finished_once_ = true;
      last_stuck_recovery_finished_at_ = now;
      return;
    }

    if (stuck_recovery_phase_ == StuckRecoveryPhase::BACKUP) {
      output.linear_x = -std::fabs(stuck_backup_speed_);
      output.angular_z = 0.0;
      output.mode = "STUCK_BACKUP";
      output.detail = "odom shows little progress";
      return;
    }

    if (stuck_recovery_phase_ == StuckRecoveryPhase::TURN) {
      output.linear_x = 0.0;
      output.angular_z =
        sign_value(static_cast<double>(stuck_turn_direction_)) * std::fabs(stuck_turn_speed_);
      output.mode = "STUCK_TURN";
      output.detail = "turning after backup";
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
      "[%s] enabled=%s imu=%s odom=%s ref=%s tilt=%.1fdeg gyro_yaw=%.1fdeg "
      "front=%.2fm left=%.2fm right=%.2fm gap=%.2fm center=%.2fm width=%.2fm "
      "narrow=%s target=%.1fdeg cmd=(%.2f, %.2f) %s",
      output.mode.c_str(),
      output.enabled_requested ? "yes" : "no",
      output.imu_ready ? "yes" : "no",
      output.odom_ready ? "yes" : "no",
      output.heading_reference_valid ? "yes" : "no",
      output.tilt_deg,
      output.heading_error_deg,
      output.scan.front_m,
      output.scan.left_m,
      output.scan.right_m,
      output.scan.best_gap_m,
      output.scan.best_center_m,
      output.scan.best_gap_width_m,
      output.scan.best_is_narrow_passage ? "yes" : "no",
      rad_to_deg(output.scan.best_angle_rad),
      output.linear_x,
      output.angular_z,
      output.detail.c_str());
  }

  const std::string scan_topic_;
  const std::string imu_topic_;
  const std::string odom_topic_;
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
  const double heading_alignment_weight_;
  const double turn_slowdown_gain_;
  const bool wall_assist_enabled_;
  const double wall_assist_side_detect_m_;
  const double wall_assist_target_distance_m_;
  const double wall_assist_gain_;
  const double wall_assist_max_angular_speed_;
  const double wall_assist_hard_stop_distance_m_;
  const double wall_assist_gap_width_max_m_;
  const bool narrow_passage_enabled_;
  const double narrow_passage_min_center_distance_m_;
  const double narrow_passage_min_sector_distance_m_;
  const double narrow_passage_max_angle_deg_;
  const double narrow_passage_max_width_m_;
  const double narrow_passage_min_depth_gain_m_;
  const double narrow_passage_bonus_;
  const double narrow_passage_speed_m_s_;
  const double recovery_turn_speed_;
  const double recovery_flip_seconds_;
  const double lost_scan_timeout_seconds_;
  const bool prefer_left_recovery_;
  const double tilt_stop_deg_;
  const double imu_timeout_seconds_;
  const double gyro_yaw_sign_;
  const double gyro_angular_deadband_rad_s_;
  const double gyro_integration_max_dt_seconds_;
  const double odom_timeout_seconds_;
  const double reverse_heading_threshold_deg_;
  const double reverse_hold_seconds_;
  const double reverse_linear_threshold_;
  const double heading_recovery_turn_speed_;
  const double reverse_resume_threshold_deg_;
  const bool stuck_recovery_enabled_;
  const double stuck_command_linear_threshold_;
  const double stuck_odom_linear_threshold_;
  const double stuck_min_progress_m_;
  const double stuck_hold_seconds_;
  const double stuck_backup_speed_;
  const double stuck_backup_seconds_;
  const double stuck_turn_speed_;
  const double stuck_turn_seconds_;
  const double stuck_cooldown_seconds_;
  const bool enforce_gap_width_;
  const double min_passage_width_m_;
  const double gap_width_search_deg_;
  const double gap_width_obstacle_max_range_m_;
  const double gap_width_boundary_drop_m_;

  // ROS2: /cmd_vel 타입이 환경마다 다를 수 있어 Twist와 TwistStamped 퍼블리셔 중 하나만 활성화한다.
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_publisher_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr cmd_vel_stamped_publisher_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_subscription_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscription_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscription_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr set_enabled_service_;
  rclcpp::TimerBase::SharedPtr control_timer_;
  rclcpp::TimerBase::SharedPtr report_timer_;

  std::mutex data_mutex_;
  sensor_msgs::msg::LaserScan::SharedPtr latest_scan_;
  rclcpp::Time latest_scan_time_;
  rclcpp::Time latest_imu_time_;
  rclcpp::Time latest_odom_time_;
  bool enabled_requested_ = false;
  bool imu_ready_ = false;
  bool odom_ready_ = false;
  bool heading_reference_valid_ = false;
  bool reverse_condition_active_ = false;
  bool reverse_recovery_active_ = false;
  bool stuck_candidate_active_ = false;
  bool stuck_recovery_finished_once_ = false;
  double roll_rad_ = 0.0;
  double pitch_rad_ = 0.0;
  double max_tilt_deg_ = 0.0;
  double current_heading_rad_ = 0.0;
  double heading_reference_rad_ = 0.0;
  double odom_x_ = 0.0;
  double odom_y_ = 0.0;
  double odom_linear_speed_ = 0.0;
  double stuck_candidate_x_ = 0.0;
  double stuck_candidate_y_ = 0.0;
  int stuck_turn_direction_ = 1;
  StuckRecoveryPhase stuck_recovery_phase_ = StuckRecoveryPhase::IDLE;
  rclcpp::Time reverse_condition_started_at_;
  rclcpp::Time stuck_candidate_started_at_;
  rclcpp::Time stuck_phase_started_at_;
  rclcpp::Time last_stuck_recovery_finished_at_;

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
