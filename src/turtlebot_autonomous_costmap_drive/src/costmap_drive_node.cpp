#include <signal.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <deque>
#include <functional>
#include <limits>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

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
    if (normalized < 0.0)
    {
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
    if (value > 0.0)
    {
      return 1.0;
    }
    if (value < 0.0)
    {
      return -1.0;
    }
    return 0.0;
  }

  double yaw_from_quaternion(double x, double y, double z, double w)
  {
    const double siny_cosp = 2.0 * (w * z + x * y);
    const double cosy_cosp = 1.0 - 2.0 * (y * y + z * z);
    return std::atan2(siny_cosp, cosy_cosp);
  }

} // namespace

struct Pose2D
{
  double x = 0.0;
  double y = 0.0;
  double yaw = 0.0;
};

struct ObstaclePoint
{
  double x = 0.0;
  double y = 0.0;
  rclcpp::Time stamp;
};

struct LocalCostmap
{
  int width = 0;
  int height = 0;
  double resolution = 0.05;
  double origin_x = 0.0;
  double origin_y = 0.0;
  std::vector<int8_t> occupancy;
  std::vector<double> cost;
};

struct LongRangeTarget
{
  bool valid = false;
  double angle_rad = 0.0;
  double distance_m = 0.0;
  double score = 0.0;
};

struct GapWidthMeasurement
{
  bool measured = false;
  double left_angle_rad = 0.0;
  double right_angle_rad = 0.0;
  double left_range_m = 0.0;
  double right_range_m = 0.0;
  double boundary_width_m = 0.0;
  double throat_width_m = 0.0;
  double width_m = 0.0;
};

struct NarrowGapTarget
{
  bool valid = false;
  bool held = false;
  double angle_rad = 0.0;
  double center_range_m = 0.0;
  double sector_clearance_m = 0.0;
  double width_m = 0.0;
  double score = 0.0;
  GapWidthMeasurement width_measurement;
};

struct CostmapSummary
{
  double front_m = 0.0;
  double left_m = 0.0;
  double right_m = 0.0;
  double best_score = -std::numeric_limits<double>::infinity();
  double best_avg_cost = 0.0;
  double best_unknown_ratio = 0.0;
  double best_angular_z = 0.0;
  int evaluated_trajectories = 0;
  int rejected_trajectories = 0;
  int obstacle_points = 0;
  bool has_safe_trajectory = false;
  bool narrow_corridor = false;
  LongRangeTarget long_range_target;
  NarrowGapTarget narrow_gap_target;
};

struct DriveOutput
{
  double linear_x = 0.0;
  double angular_z = 0.0;
  std::string mode = "WAIT_SCAN";
  std::string detail;
  CostmapSummary costmap;
  std::vector<Pose2D> selected_path;
  bool enabled_requested = false;
  bool imu_ready = false;
  bool odom_ready = false;
  bool heading_reference_valid = false;
  double tilt_deg = 0.0;
  double heading_error_deg = 0.0;
};

class TurtlebotAutonomousCostmapDrive : public rclcpp::Node
{
public:
  TurtlebotAutonomousCostmapDrive()
      : Node("turtlebot_autonomous_costmap_drive"),
        scan_topic_(this->declare_parameter<std::string>("scan_topic", "/scan")),
        imu_topic_(this->declare_parameter<std::string>("imu_topic", "/imu")),
        odom_topic_(this->declare_parameter<std::string>("odom_topic", "/odom")),
        cmd_vel_topic_(this->declare_parameter<std::string>("cmd_vel_topic", "/cmd_vel")),
        cmd_vel_stamped_(this->declare_parameter<bool>("cmd_vel_stamped", false)),
        local_costmap_topic_(
            this->declare_parameter<std::string>("local_costmap_topic", "/costmap_drive/local_costmap")),
        trajectory_marker_topic_(this->declare_parameter<std::string>(
            "trajectory_marker_topic", "/costmap_drive/trajectory_markers")),
        costmap_frame_id_(this->declare_parameter<std::string>("costmap_frame_id", "base_link")),
        auto_start_(this->declare_parameter<bool>("auto_start", false)),
        reset_heading_on_start_(this->declare_parameter<bool>("reset_heading_on_start", true)),
        require_odom_(this->declare_parameter<bool>("require_odom", true)),
        control_period_ms_(this->declare_parameter<int>("control_period_ms", 100)),
        report_period_ms_(this->declare_parameter<int>("report_period_ms", 1000)),
        max_linear_speed_(this->declare_parameter<double>("max_linear_speed", 0.12)),
        min_linear_speed_(this->declare_parameter<double>("min_linear_speed", 0.02)),
        max_angular_speed_(this->declare_parameter<double>("max_angular_speed", 0.85)),
        recovery_turn_speed_(this->declare_parameter<double>("recovery_turn_speed", 0.65)),
        recovery_flip_seconds_(this->declare_parameter<double>("recovery_flip_seconds", 3.5)),
        prefer_left_recovery_(this->declare_parameter<bool>("prefer_left_recovery", true)),
        emergency_stop_distance_m_(this->declare_parameter<double>("emergency_stop_distance_m", 0.12)),
        front_stop_distance_m_(this->declare_parameter<double>("front_stop_distance_m", 0.20)),
        front_slow_distance_m_(this->declare_parameter<double>("front_slow_distance_m", 0.55)),
        side_angle_deg_(this->declare_parameter<double>("side_angle_deg", 75.0)),
        side_window_deg_(this->declare_parameter<double>("side_window_deg", 18.0)),
        front_window_deg_(this->declare_parameter<double>("front_window_deg", 18.0)),
        scan_sample_step_deg_(this->declare_parameter<double>("scan_sample_step_deg", 2.0)),
        scan_obstacle_min_range_m_(
            this->declare_parameter<double>("scan_obstacle_min_range_m", 0.05)),
        scan_obstacle_max_range_m_(
            this->declare_parameter<double>("scan_obstacle_max_range_m", 3.0)),
        raytrace_max_range_m_(this->declare_parameter<double>("raytrace_max_range_m", 3.0)),
        lost_scan_timeout_seconds_(this->declare_parameter<double>("lost_scan_timeout_seconds", 1.0)),
        grid_resolution_m_(this->declare_parameter<double>("grid_resolution_m", 0.04)),
        grid_forward_m_(this->declare_parameter<double>("grid_forward_m", 1.60)),
        grid_back_m_(this->declare_parameter<double>("grid_back_m", 0.35)),
        grid_half_width_m_(this->declare_parameter<double>("grid_half_width_m", 0.95)),
        obstacle_memory_seconds_(this->declare_parameter<double>("obstacle_memory_seconds", 1.2)),
        obstacle_memory_max_points_(this->declare_parameter<int>("obstacle_memory_max_points", 6000)),
        robot_radius_m_(this->declare_parameter<double>("robot_radius_m", 0.15)),
        inflation_radius_m_(this->declare_parameter<double>("inflation_radius_m", 0.10)),
        lethal_cost_threshold_(this->declare_parameter<double>("lethal_cost_threshold", 75.0)),
        unknown_cell_cost_(this->declare_parameter<double>("unknown_cell_cost", 35.0)),
        allow_unknown_trajectory_(this->declare_parameter<bool>("allow_unknown_trajectory", true)),
        trajectory_horizon_s_(this->declare_parameter<double>("trajectory_horizon_s", 1.2)),
        trajectory_dt_s_(this->declare_parameter<double>("trajectory_dt_s", 0.10)),
        angular_sample_count_(this->declare_parameter<int>("angular_sample_count", 15)),
        turn_slowdown_gain_(this->declare_parameter<double>("turn_slowdown_gain", 0.65)),
        progress_reward_weight_(this->declare_parameter<double>("progress_reward_weight", 2.2)),
        cost_obstacle_weight_(this->declare_parameter<double>("cost_obstacle_weight", 4.0)),
        cost_unknown_weight_(this->declare_parameter<double>("cost_unknown_weight", 0.45)),
        cost_turn_weight_(this->declare_parameter<double>("cost_turn_weight", 0.45)),
        cost_smooth_weight_(this->declare_parameter<double>("cost_smooth_weight", 0.35)),
        cost_lateral_weight_(this->declare_parameter<double>("cost_lateral_weight", 0.35)),
        heading_alignment_weight_(this->declare_parameter<double>("heading_alignment_weight", 0.35)),
        long_range_clearance_enabled_(
            this->declare_parameter<bool>("long_range_clearance_enabled", true)),
        long_range_clearance_weight_(
            this->declare_parameter<double>("long_range_clearance_weight", 0.65)),
        long_range_clearance_max_range_m_(
            this->declare_parameter<double>("long_range_clearance_max_range_m", 2.0)),
        long_range_clearance_min_range_m_(
            this->declare_parameter<double>("long_range_clearance_min_range_m", 0.60)),
        long_range_clearance_search_deg_(
            this->declare_parameter<double>("long_range_clearance_search_deg", 100.0)),
        long_range_clearance_window_deg_(
            this->declare_parameter<double>("long_range_clearance_window_deg", 6.0)),
        long_range_clearance_alignment_deg_(
            this->declare_parameter<double>("long_range_clearance_alignment_deg", 45.0)),
        long_range_clearance_start_bias_(
            this->declare_parameter<double>("long_range_clearance_start_bias", 0.35)),
        narrow_gap_target_enabled_(this->declare_parameter<bool>("narrow_gap_target_enabled", true)),
        narrow_gap_bonus_weight_(this->declare_parameter<double>("narrow_gap_bonus_weight", 0.85)),
        narrow_gap_min_width_m_(this->declare_parameter<double>("narrow_gap_min_width_m", 0.23)),
        narrow_gap_max_width_m_(this->declare_parameter<double>("narrow_gap_max_width_m", 0.46)),
        narrow_gap_search_deg_(this->declare_parameter<double>("narrow_gap_search_deg", 100.0)),
        narrow_gap_sector_half_width_deg_(
            this->declare_parameter<double>("narrow_gap_sector_half_width_deg", 10.0)),
        narrow_gap_boundary_search_deg_(
            this->declare_parameter<double>("narrow_gap_boundary_search_deg", 45.0)),
        narrow_gap_boundary_obstacle_max_range_m_(this->declare_parameter<double>(
            "narrow_gap_boundary_obstacle_max_range_m", 1.25)),
        narrow_gap_boundary_drop_m_(
            this->declare_parameter<double>("narrow_gap_boundary_drop_m", 0.05)),
        narrow_gap_min_center_distance_m_(
            this->declare_parameter<double>("narrow_gap_min_center_distance_m", 0.30)),
        narrow_gap_min_sector_distance_m_(
            this->declare_parameter<double>("narrow_gap_min_sector_distance_m", 0.08)),
        narrow_gap_min_depth_gain_m_(
            this->declare_parameter<double>("narrow_gap_min_depth_gain_m", 0.05)),
        narrow_gap_alignment_deg_(this->declare_parameter<double>("narrow_gap_alignment_deg", 35.0)),
        narrow_gap_hold_seconds_(this->declare_parameter<double>("narrow_gap_hold_seconds", 1.0)),
        narrow_gap_hold_max_angle_error_deg_(
            this->declare_parameter<double>("narrow_gap_hold_max_angle_error_deg", 25.0)),
        narrow_gap_speed_m_s_(this->declare_parameter<double>("narrow_gap_speed_m_s", 0.055)),
        narrow_corridor_enabled_(this->declare_parameter<bool>("narrow_corridor_enabled", true)),
        narrow_corridor_width_m_(this->declare_parameter<double>("narrow_corridor_width_m", 0.62)),
        narrow_corridor_side_detect_m_(
            this->declare_parameter<double>("narrow_corridor_side_detect_m", 0.42)),
        narrow_corridor_speed_m_s_(
            this->declare_parameter<double>("narrow_corridor_speed_m_s", 0.045)),
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
            this->declare_parameter<double>("heading_recovery_turn_speed", 0.75)),
        reverse_resume_threshold_deg_(
            this->declare_parameter<double>("reverse_resume_threshold_deg", 25.0)),
        stuck_recovery_enabled_(this->declare_parameter<bool>("stuck_recovery_enabled", true)),
        stuck_command_linear_threshold_(
            this->declare_parameter<double>("stuck_command_linear_threshold", 0.03)),
        stuck_odom_linear_threshold_(
            this->declare_parameter<double>("stuck_odom_linear_threshold", 0.01)),
        stuck_min_progress_m_(this->declare_parameter<double>("stuck_min_progress_m", 0.03)),
        stuck_hold_seconds_(this->declare_parameter<double>("stuck_hold_seconds", 2.0)),
        stuck_backup_speed_(this->declare_parameter<double>("stuck_backup_speed", 0.04)),
        stuck_backup_seconds_(this->declare_parameter<double>("stuck_backup_seconds", 0.8)),
        stuck_turn_speed_(this->declare_parameter<double>("stuck_turn_speed", 0.60)),
        stuck_turn_seconds_(this->declare_parameter<double>("stuck_turn_seconds", 1.0)),
        stuck_cooldown_seconds_(this->declare_parameter<double>("stuck_cooldown_seconds", 2.0)),
        blocked_recovery_enabled_(this->declare_parameter<bool>("blocked_recovery_enabled", true)),
        blocked_recovery_hold_seconds_(
            this->declare_parameter<double>("blocked_recovery_hold_seconds", 1.2))
  {
    enabled_requested_ = auto_start_;

    if (cmd_vel_stamped_)
    {
      cmd_vel_stamped_publisher_ =
          this->create_publisher<geometry_msgs::msg::TwistStamped>(cmd_vel_topic_, 10);
    }
    else
    {
      cmd_vel_publisher_ =
          this->create_publisher<geometry_msgs::msg::Twist>(cmd_vel_topic_, 10);
    }

    local_costmap_publisher_ =
        this->create_publisher<nav_msgs::msg::OccupancyGrid>(local_costmap_topic_, 1);
    trajectory_marker_publisher_ =
        this->create_publisher<visualization_msgs::msg::MarkerArray>(trajectory_marker_topic_, 1);

    scan_subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        scan_topic_,
        rclcpp::SensorDataQoS(),
        std::bind(&TurtlebotAutonomousCostmapDrive::scan_callback, this, _1));

    imu_subscription_ = this->create_subscription<sensor_msgs::msg::Imu>(
        imu_topic_,
        rclcpp::SensorDataQoS(),
        std::bind(&TurtlebotAutonomousCostmapDrive::imu_callback, this, _1));

    odom_subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
        odom_topic_,
        rclcpp::SensorDataQoS(),
        std::bind(&TurtlebotAutonomousCostmapDrive::odom_callback, this, _1));

    set_enabled_service_ = this->create_service<std_srvs::srv::SetBool>(
        "/turtlebot_autonomous_costmap_drive/set_enabled",
        std::bind(&TurtlebotAutonomousCostmapDrive::set_enabled_callback, this, _1, _2));

    control_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(control_period_ms_),
        std::bind(&TurtlebotAutonomousCostmapDrive::control_timer_callback, this));

    report_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(report_period_ms_),
        std::bind(&TurtlebotAutonomousCostmapDrive::report_timer_callback, this));

    RCLCPP_INFO(
        this->get_logger(),
        "Started costmap drive: scan=%s imu=%s odom=%s cmd=%s type=%s auto_start=%s max_v=%.2f",
        scan_topic_.c_str(),
        imu_topic_.c_str(),
        odom_topic_.c_str(),
        cmd_vel_topic_.c_str(),
        cmd_vel_stamped_ ? "TwistStamped" : "Twist",
        auto_start_ ? "true" : "false",
        max_linear_speed_);
    RCLCPP_INFO(
        this->get_logger(),
        "Use: ros2 service call /turtlebot_autonomous_costmap_drive/set_enabled std_srvs/srv/SetBool \"{data: true}\"");
  }

  void publish_stop_command()
  {
    for (int i = 0; i < 5; ++i)
    {
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
    Pose2D odom_pose;
    double odom_linear_speed = 0.0;
    rclcpp::Time latest_imu_time;
    rclcpp::Time latest_odom_time;
  };

  enum class StuckRecoveryPhase
  {
    IDLE,
    BACKUP,
    TURN
  };

  enum class RecoverySource
  {
    NONE,
    STUCK,
    BLOCKED
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

      if (request->data)
      {
        enabled_requested_ = true;
        reverse_condition_active_ = false;
        reverse_recovery_active_ = false;
        blocked_recovery_candidate_active_ = false;
        held_narrow_gap_target_ = NarrowGapTarget();
        reset_recovery_state();

        if (reset_heading_on_start_ || !heading_reference_valid_)
        {
          heading_reference_valid_ = false;
          if (sensors_current_locked(now))
          {
            set_heading_reference_locked();
            heading_reset = true;
          }
          else
          {
            waiting_for_sensors = true;
          }
        }
      }
      else
      {
        enabled_requested_ = false;
        heading_reference_valid_ = false;
        reverse_condition_active_ = false;
        reverse_recovery_active_ = false;
        blocked_recovery_candidate_active_ = false;
        stuck_candidate_active_ = false;
        stuck_recovery_phase_ = StuckRecoveryPhase::IDLE;
        recovery_source_ = RecoverySource::NONE;
        held_narrow_gap_target_ = NarrowGapTarget();
        obstacle_memory_.clear();
        reset_recovery_state();
        publish_stop = true;
      }
    }

    response->success = true;
    if (request->data)
    {
      if (heading_reset)
      {
        response->message = "Costmap drive enabled. Gyro yaw reference reset.";
      }
      else if (waiting_for_sensors)
      {
        response->message = "Costmap drive start requested. Waiting for current sensors.";
      }
      else
      {
        response->message = "Costmap drive enabled.";
      }
    }
    else
    {
      response->message = "Costmap drive stopped. Publishing zero velocity.";
      DriveOutput output;
      output.mode = "STOPPED";
      output.detail = "service stop";
      store_output(output);
    }

    if (publish_stop)
    {
      publish_velocity(0.0, 0.0);
    }
  }

  void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
  {
    auto sanitized_scan = std::make_shared<sensor_msgs::msg::LaserScan>(*msg);
    sanitize_scan_ranges(*sanitized_scan);

    std::lock_guard<std::mutex> lock(data_mutex_);
    latest_scan_ = sanitized_scan;
    latest_scan_time_ = this->now();
  }

  void sanitize_scan_ranges(sensor_msgs::msg::LaserScan &scan) const
  {
    float previous_valid_range = std::numeric_limits<float>::quiet_NaN();
    bool has_previous_valid_range = false;

    for (auto &range : scan.ranges)
    {
      if (std::isfinite(static_cast<double>(range)) && range > 0.0f)
      {
        previous_valid_range = range;
        has_previous_valid_range = true;
        continue;
      }

      range = has_previous_valid_range
                  ? previous_valid_range
                  : std::numeric_limits<float>::quiet_NaN();
    }
  }

  void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
  {
    const double angular_velocity_z = msg->angular_velocity.z;
    const double norm = std::sqrt(
        msg->orientation.x * msg->orientation.x +
        msg->orientation.y * msg->orientation.y +
        msg->orientation.z * msg->orientation.z +
        msg->orientation.w * msg->orientation.w);

    if (!std::isfinite(norm) || norm < 1e-6 || !std::isfinite(angular_velocity_z))
    {
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
    if (imu_ready_)
    {
      const double dt = (now - latest_imu_time_).seconds();
      if (dt > 0.0 && dt <= gyro_integration_max_dt_seconds_)
      {
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
    const double qx = msg->pose.pose.orientation.x;
    const double qy = msg->pose.pose.orientation.y;
    const double qz = msg->pose.pose.orientation.z;
    const double qw = msg->pose.pose.orientation.w;

    std::lock_guard<std::mutex> lock(data_mutex_);
    if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(vx) || !std::isfinite(vy) ||
        !std::isfinite(qx) || !std::isfinite(qy) || !std::isfinite(qz) || !std::isfinite(qw))
    {
      odom_ready_ = false;
      return;
    }

    odom_pose_.x = x;
    odom_pose_.y = y;
    odom_pose_.yaw = yaw_from_quaternion(qx, qy, qz, qw);
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

    if (!safety.enabled_requested)
    {
      output.mode = "STOPPED";
      output.detail = "waiting for set_enabled true";
      publish_velocity(0.0, 0.0);
      store_output(output);
      return;
    }

    std::string wait_reason;
    if (!safety_data_ready(safety, now, wait_reason))
    {
      output.mode = "WAIT_SENSORS";
      output.detail = wait_reason;
      publish_velocity(0.0, 0.0);
      store_output(output);
      return;
    }

    if (safety.max_tilt_deg > tilt_stop_deg_)
    {
      output.mode = "TILT_STOP";
      output.detail = "tilt limit exceeded";
      publish_velocity(0.0, 0.0);
      store_output(output);
      return;
    }

    if (!safety.heading_reference_valid)
    {
      if (!reset_heading_reference_from_current())
      {
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

    if (!scan || scan->ranges.empty())
    {
      output.mode = "WAIT_SCAN";
      output.detail = "waiting for lidar scan";
      publish_velocity(0.0, 0.0);
      store_output(output);
      return;
    }

    const double scan_age = (now - scan_time).seconds();
    if (scan_age > lost_scan_timeout_seconds_)
    {
      output.mode = "SCAN_TIMEOUT";
      output.detail = "lidar scan timeout";
      publish_velocity(0.0, 0.0);
      store_output(output);
      return;
    }

    integrate_scan_if_new(*scan, scan_time, safety.odom_pose);
    prune_obstacle_memory(now);

    LocalCostmap costmap = build_local_costmap(*scan, safety.odom_pose);
    publish_local_costmap(costmap);

    output = compute_costmap_drive_output(*scan, costmap, safety);
    annotate_output(output, safety);
    apply_reverse_guard(output, now);
    apply_stuck_recovery(output, now, costmap);

    publish_trajectory_markers(output);
    publish_velocity(output.linear_x, output.angular_z);
    store_output(output);
  }

  DriveOutput compute_costmap_drive_output(
      const sensor_msgs::msg::LaserScan &scan,
      const LocalCostmap &costmap,
      const SafetySnapshot &safety)
  {
    DriveOutput output;
    output.costmap.front_m = min_range_in_sector(scan, 0.0, front_window_deg_);
    output.costmap.left_m = min_range_in_sector(scan, side_angle_deg_, side_window_deg_);
    output.costmap.right_m = min_range_in_sector(scan, -side_angle_deg_, side_window_deg_);
    output.costmap.obstacle_points = obstacle_memory_size();
    output.costmap.narrow_corridor = detect_narrow_corridor(output.costmap);
    output.costmap.long_range_target = compute_long_range_target(scan, safety);
    output.costmap.narrow_gap_target = compute_narrow_gap_target(scan);

    if (output.costmap.front_m <= emergency_stop_distance_m_)
    {
      output.linear_x = 0.0;
      output.angular_z = choose_recovery_turn(output.costmap.left_m, output.costmap.right_m);
      output.mode = "EMERGENCY_TURN";
      output.detail = "front obstacle inside emergency distance";
      return output;
    }

    if (!select_best_trajectory(costmap, safety, output))
    {
      output.linear_x = 0.0;
      output.angular_z = choose_recovery_turn(output.costmap.left_m, output.costmap.right_m);
      output.mode = "BLOCKED_TURN";
      output.detail = "no collision-free trajectory in local costmap";
      return output;
    }

    reset_recovery_state();
    if (output.costmap.narrow_gap_target.valid)
    {
      output.mode = "NARROW_GAP_COSTMAP_DRIVE";
      output.detail = "width-valid gate target, speed limited";
    }
    else if (output.costmap.narrow_corridor)
    {
      output.mode = "NARROW_COSTMAP_DRIVE";
      output.detail = "inflated corridor is passable, speed limited";
    }
    else
    {
      output.mode = "COSTMAP_DRIVE";
      output.detail = "selected lowest-cost trajectory";
    }
    return output;
  }

  bool select_best_trajectory(
      const LocalCostmap &costmap,
      const SafetySnapshot &safety,
      DriveOutput &output) const
  {
    int sample_count = std::max(3, angular_sample_count_);
    if (sample_count % 2 == 0)
    {
      ++sample_count;
    }
    const double front_ratio = clamp_value(
        (output.costmap.front_m - front_stop_distance_m_) /
            std::max(0.01, front_slow_distance_m_ - front_stop_distance_m_),
        0.0,
        1.0);
    double speed_cap = max_linear_speed_;
    if (output.costmap.narrow_corridor)
    {
      speed_cap = std::min(speed_cap, std::max(min_linear_speed_, narrow_corridor_speed_m_s_));
    }
    if (output.costmap.narrow_gap_target.valid)
    {
      speed_cap = std::min(speed_cap, std::max(min_linear_speed_, narrow_gap_speed_m_s_));
    }
    const double base_linear =
        min_linear_speed_ + (speed_cap - min_linear_speed_) * front_ratio;

    bool found = false;
    const double angular_limit = std::max(0.01, max_angular_speed_);
    double best_score = -std::numeric_limits<double>::infinity();
    double best_linear = 0.0;
    double best_angular = 0.0;
    double best_avg_cost = 0.0;
    double best_unknown_ratio = 0.0;
    std::vector<Pose2D> best_path;

    for (int i = 0; i < sample_count; ++i)
    {
      const double ratio = sample_count == 1 ? 0.0 : static_cast<double>(i) / static_cast<double>(sample_count - 1);
      const double angular_z = -angular_limit + 2.0 * angular_limit * ratio;
      const double turn_ratio = clamp_value(std::fabs(angular_z) / angular_limit, 0.0, 1.0);
      const double turn_scale = clamp_value(1.0 - turn_slowdown_gain_ * turn_ratio, 0.25, 1.0);
      const double linear_x = clamp_value(base_linear * turn_scale, min_linear_speed_, speed_cap);

      double score = 0.0;
      double avg_cost = 0.0;
      double unknown_ratio = 0.0;
      std::vector<Pose2D> path;
      ++output.costmap.evaluated_trajectories;
      if (!score_trajectory(
              costmap,
              safety,
              output.costmap.long_range_target,
              output.costmap.narrow_gap_target,
              linear_x,
              angular_z,
              score,
              avg_cost,
              unknown_ratio,
              path))
      {
        ++output.costmap.rejected_trajectories;
        continue;
      }

      if (!found || score > best_score)
      {
        found = true;
        best_score = score;
        best_linear = linear_x;
        best_angular = angular_z;
        best_avg_cost = avg_cost;
        best_unknown_ratio = unknown_ratio;
        best_path = path;
      }
    }

    if (!found)
    {
      return false;
    }

    output.costmap.has_safe_trajectory = true;
    output.costmap.best_score = best_score;
    output.costmap.best_avg_cost = best_avg_cost;
    output.costmap.best_unknown_ratio = best_unknown_ratio;
    output.costmap.best_angular_z = best_angular;
    output.linear_x = best_linear;
    output.angular_z = best_angular;
    output.selected_path = best_path;
    return true;
  }

  bool score_trajectory(
      const LocalCostmap &costmap,
      const SafetySnapshot &safety,
      const LongRangeTarget &long_range_target,
      const NarrowGapTarget &narrow_gap_target,
      double linear_x,
      double angular_z,
      double &score,
      double &avg_cost,
      double &unknown_ratio,
      std::vector<Pose2D> &path) const
  {
    Pose2D pose;
    double cost_sum = 0.0;
    int cost_samples = 0;
    int unknown_samples = 0;
    double max_cost = 0.0;

    const double dt = std::max(0.01, trajectory_dt_s_);
    const double horizon = std::max(dt, trajectory_horizon_s_);
    const double angular_limit = std::max(0.01, max_angular_speed_);
    const int steps = std::max(1, static_cast<int>(std::ceil(horizon / dt)));
    path.clear();
    path.reserve(static_cast<std::size_t>(steps) + 1);
    path.push_back(pose);

    for (int step = 0; step < steps; ++step)
    {
      pose.x += linear_x * std::cos(pose.yaw) * dt;
      pose.y += linear_x * std::sin(pose.yaw) * dt;
      pose.yaw = normalize_angle_rad(pose.yaw + angular_z * dt);
      path.push_back(pose);

      if (!accumulate_footprint_cost(
              costmap, pose.x, pose.y, cost_sum, cost_samples, unknown_samples, max_cost))
      {
        return false;
      }
    }

    if (cost_samples == 0)
    {
      return false;
    }

    avg_cost = cost_sum / static_cast<double>(cost_samples);
    unknown_ratio = static_cast<double>(unknown_samples) / static_cast<double>(cost_samples);

    const double turn_ratio = clamp_value(std::fabs(angular_z) / angular_limit, 0.0, 1.0);
    const double smooth_ratio =
        clamp_value(std::fabs(angular_z - last_command_angular_z_) / (2.0 * angular_limit), 0.0, 1.0);
    const double heading_error = safety.heading_reference_valid ? std::fabs(normalize_angle_rad(safety.current_heading_rad + pose.yaw - safety.heading_reference_rad)) : 0.0;
    const double heading_ratio = clamp_value(heading_error / kPi, 0.0, 1.0);
    double long_range_reward = 0.0;
    if (long_range_target.valid && long_range_clearance_weight_ > 0.0)
    {
      const double alignment_width = deg_to_rad(std::max(1.0, long_range_clearance_alignment_deg_));
      const double target_error =
          std::fabs(normalize_angle_rad(pose.yaw - long_range_target.angle_rad));
      const double target_alignment =
          1.0 - clamp_value(target_error / alignment_width, 0.0, 1.0);
      const double max_distance = std::max(
          long_range_clearance_min_range_m_ + 0.01,
          long_range_clearance_max_range_m_);
      const double clearance_ratio = clamp_value(
          (long_range_target.distance_m - long_range_clearance_min_range_m_) /
              (max_distance - long_range_clearance_min_range_m_),
          0.0,
          1.0);
      long_range_reward =
          long_range_clearance_weight_ * clearance_ratio * target_alignment;
    }
    double narrow_gap_reward = 0.0;
    if (narrow_gap_target.valid && narrow_gap_bonus_weight_ > 0.0)
    {
      const double alignment_width = deg_to_rad(std::max(1.0, narrow_gap_alignment_deg_));
      const double target_error =
          std::fabs(normalize_angle_rad(pose.yaw - narrow_gap_target.angle_rad));
      const double target_alignment =
          1.0 - clamp_value(target_error / alignment_width, 0.0, 1.0);
      const double width_span =
          std::max(0.01, narrow_gap_max_width_m_ - narrow_gap_min_width_m_);
      const double width_ratio =
          clamp_value((narrow_gap_target.width_m - narrow_gap_min_width_m_) / width_span, 0.0, 1.0);
      const double depth_ratio = clamp_value(
          (narrow_gap_target.center_range_m - narrow_gap_min_center_distance_m_) /
              std::max(0.01, scan_obstacle_max_range_m_ - narrow_gap_min_center_distance_m_),
          0.0,
          1.0);
      const double held_multiplier = narrow_gap_target.held ? 1.10 : 1.0;
      narrow_gap_reward =
          narrow_gap_bonus_weight_ *
          (0.65 + 0.20 * depth_ratio + 0.15 * width_ratio) *
          target_alignment *
          held_multiplier;
    }

    score =
        progress_reward_weight_ * pose.x -
        cost_obstacle_weight_ * (avg_cost / 100.0) -
        cost_unknown_weight_ * unknown_ratio -
        cost_turn_weight_ * turn_ratio -
        cost_smooth_weight_ * smooth_ratio -
        cost_lateral_weight_ * std::fabs(pose.y) -
        heading_alignment_weight_ * heading_ratio +
        long_range_reward +
        narrow_gap_reward;

    (void)max_cost;
    return true;
  }

  bool accumulate_footprint_cost(
      const LocalCostmap &costmap,
      double x,
      double y,
      double &cost_sum,
      int &cost_samples,
      int &unknown_samples,
      double &max_cost) const
  {
    const int perimeter_samples = 12;
    if (!accumulate_cell_cost(costmap, x, y, cost_sum, cost_samples, unknown_samples, max_cost))
    {
      return false;
    }

    for (int i = 0; i < perimeter_samples; ++i)
    {
      const double angle = kTwoPi * static_cast<double>(i) / static_cast<double>(perimeter_samples);
      const double px = x + robot_radius_m_ * std::cos(angle);
      const double py = y + robot_radius_m_ * std::sin(angle);
      if (!accumulate_cell_cost(costmap, px, py, cost_sum, cost_samples, unknown_samples, max_cost))
      {
        return false;
      }
    }
    return true;
  }

  bool accumulate_cell_cost(
      const LocalCostmap &costmap,
      double x,
      double y,
      double &cost_sum,
      int &cost_samples,
      int &unknown_samples,
      double &max_cost) const
  {
    int gx = 0;
    int gy = 0;
    if (!world_to_grid(costmap, x, y, gx, gy))
    {
      return false;
    }

    const int index = gy * costmap.width + gx;
    if (index < 0 || index >= static_cast<int>(costmap.occupancy.size()))
    {
      return false;
    }

    if (costmap.occupancy[index] < 0)
    {
      if (!allow_unknown_trajectory_)
      {
        return false;
      }
      cost_sum += unknown_cell_cost_;
      max_cost = std::max(max_cost, unknown_cell_cost_);
      ++cost_samples;
      ++unknown_samples;
      return true;
    }

    const double cell_cost = costmap.cost[index];
    if (cell_cost >= lethal_cost_threshold_)
    {
      return false;
    }

    cost_sum += cell_cost;
    max_cost = std::max(max_cost, cell_cost);
    ++cost_samples;
    return true;
  }

  bool detect_narrow_corridor(const CostmapSummary &summary) const
  {
    if (!narrow_corridor_enabled_)
    {
      return false;
    }
    const bool both_sides_near =
        summary.left_m <= narrow_corridor_side_detect_m_ &&
        summary.right_m <= narrow_corridor_side_detect_m_;
    const bool measured_width_narrow =
        summary.left_m + summary.right_m <= narrow_corridor_width_m_;
    return both_sides_near && measured_width_narrow;
  }

  LongRangeTarget compute_long_range_target(
      const sensor_msgs::msg::LaserScan &scan,
      const SafetySnapshot &safety) const
  {
    LongRangeTarget target;
    if (!long_range_clearance_enabled_ || scan.ranges.empty() ||
        std::fabs(scan.angle_increment) < 1e-9)
    {
      return target;
    }

    const double max_distance = effective_long_range_max(scan);
    const double min_distance = std::max(0.0, long_range_clearance_min_range_m_);
    if (max_distance <= min_distance + 0.01)
    {
      return target;
    }

    const double start_angle = safety.heading_reference_valid
                                   ? normalize_angle_rad(
                                         safety.heading_reference_rad -
                                         safety.current_heading_rad)
                                   : 0.0;
    const double search_width = deg_to_rad(std::max(1.0, long_range_clearance_search_deg_));
    const double forward_limit = deg_to_rad(120.0);
    const double window_half_width = std::max(0.0, long_range_clearance_window_deg_);
    const double step_deg = std::max(1.0, scan_sample_step_deg_);
    const double start_bias = std::max(0.0, long_range_clearance_start_bias_);

    for (int i = 0; i < static_cast<int>(scan.ranges.size()); ++i)
    {
      const double angle = scan.angle_min + static_cast<double>(i) * scan.angle_increment;
      const double start_error = std::fabs(normalize_angle_rad(angle - start_angle));
      if (start_error > search_width)
      {
        continue;
      }
      if (std::fabs(normalize_angle_rad(angle)) > forward_limit)
      {
        continue;
      }

      double window_min = max_distance;
      bool sampled = false;
      for (double offset_deg = -window_half_width;
           offset_deg <= window_half_width + 1e-6;
           offset_deg += step_deg)
      {
        double range = 0.0;
        double clearance = 0.0;
        if (!range_at_angle(scan, angle + deg_to_rad(offset_deg), range) ||
            !range_to_long_range_clearance(scan, range, clearance))
        {
          continue;
        }
        sampled = true;
        window_min = std::min(window_min, clearance);
      }

      if (!sampled || window_min < min_distance)
      {
        continue;
      }

      const double clearance_ratio =
          clamp_value((window_min - min_distance) / (max_distance - min_distance), 0.0, 1.0);
      const double start_alignment =
          1.0 - clamp_value(start_error / search_width, 0.0, 1.0);
      const double candidate_score = clearance_ratio + start_bias * start_alignment;

      if (!target.valid || candidate_score > target.score)
      {
        target.valid = true;
        target.angle_rad = normalize_angle_rad(angle);
        target.distance_m = window_min;
        target.score = candidate_score;
      }
    }

    return target;
  }

  NarrowGapTarget compute_narrow_gap_target(const sensor_msgs::msg::LaserScan &scan)
  {
    NarrowGapTarget best_target;
    NarrowGapTarget held_candidate;
    if (!narrow_gap_target_enabled_ || scan.ranges.empty() ||
        std::fabs(scan.angle_increment) < 1e-9)
    {
      held_narrow_gap_target_ = NarrowGapTarget();
      return best_target;
    }

    const rclcpp::Time now = this->now();
    const bool held_active =
        held_narrow_gap_target_.valid &&
        narrow_gap_hold_seconds_ > 0.0 &&
        (now - held_narrow_gap_target_updated_at_).seconds() <= narrow_gap_hold_seconds_;
    const double held_angle_limit =
        deg_to_rad(std::max(1.0, narrow_gap_hold_max_angle_error_deg_));
    const double search_limit = std::max(1.0, narrow_gap_search_deg_);
    const double step_deg = std::max(1.0, scan_sample_step_deg_);
    const double max_center_range =
        std::max(narrow_gap_min_center_distance_m_ + 0.01, scan_obstacle_max_range_m_);

    for (double angle_deg = -search_limit; angle_deg <= search_limit; angle_deg += step_deg)
    {
      double center_range = 0.0;
      if (!range_at_angle(scan, deg_to_rad(angle_deg), center_range) ||
          !range_is_usable(scan, center_range))
      {
        continue;
      }

      const double sector_clearance =
          min_range_in_sector(scan, angle_deg, narrow_gap_sector_half_width_deg_);
      const GapWidthMeasurement width_measurement =
          measure_narrow_gap_width(scan, angle_deg, center_range);
      if (!narrow_gap_allows(angle_deg, center_range, sector_clearance, width_measurement))
      {
        continue;
      }

      NarrowGapTarget candidate;
      candidate.valid = true;
      candidate.angle_rad = deg_to_rad(angle_deg);
      candidate.center_range_m = center_range;
      candidate.sector_clearance_m = sector_clearance;
      candidate.width_m = width_measurement.width_m;
      candidate.width_measurement = width_measurement;

      const double center_ratio = clamp_value(
          (center_range - narrow_gap_min_center_distance_m_) /
              (max_center_range - narrow_gap_min_center_distance_m_),
          0.0,
          1.0);
      const double width_ratio = clamp_value(
          (width_measurement.width_m - narrow_gap_min_width_m_) /
              std::max(0.01, narrow_gap_max_width_m_ - narrow_gap_min_width_m_),
          0.0,
          1.0);
      const double forward_ratio =
          1.0 - clamp_value(std::fabs(angle_deg) / search_limit, 0.0, 1.0);
      candidate.score = center_ratio + 0.25 * width_ratio + 0.35 * forward_ratio;

      if (!best_target.valid || candidate.score > best_target.score)
      {
        best_target = candidate;
      }

      if (held_active &&
          std::fabs(normalize_angle_rad(candidate.angle_rad - held_narrow_gap_target_.angle_rad)) <=
              held_angle_limit &&
          (!held_candidate.valid || candidate.score > held_candidate.score))
      {
        held_candidate = candidate;
        held_candidate.held = true;
      }
    }

    NarrowGapTarget target = held_candidate.valid ? held_candidate : best_target;
    if (target.valid)
    {
      held_narrow_gap_target_ = target;
      held_narrow_gap_target_updated_at_ = now;
    }
    else if (!held_active)
    {
      held_narrow_gap_target_ = NarrowGapTarget();
    }

    return target;
  }

  GapWidthMeasurement measure_narrow_gap_width(
      const sensor_msgs::msg::LaserScan &scan,
      double center_deg,
      double center_range_m) const
  {
    GapWidthMeasurement measurement;
    const bool has_left = find_narrow_gap_boundary(
        scan,
        center_deg,
        center_range_m,
        1.0,
        measurement.left_angle_rad,
        measurement.left_range_m);
    const bool has_right = find_narrow_gap_boundary(
        scan,
        center_deg,
        center_range_m,
        -1.0,
        measurement.right_angle_rad,
        measurement.right_range_m);

    if (!has_left || !has_right)
    {
      return measurement;
    }

    const double angle_between =
        std::fabs(normalize_angle_rad(measurement.left_angle_rad - measurement.right_angle_rad));
    const double boundary_width_squared =
        measurement.left_range_m * measurement.left_range_m +
        measurement.right_range_m * measurement.right_range_m -
        2.0 * measurement.left_range_m * measurement.right_range_m * std::cos(angle_between);

    measurement.boundary_width_m = std::sqrt(std::max(0.0, boundary_width_squared));
    const double near_boundary_range =
        std::min(measurement.left_range_m, measurement.right_range_m);
    measurement.throat_width_m = 2.0 * near_boundary_range * std::sin(0.5 * angle_between);
    measurement.width_m = std::min(measurement.boundary_width_m, measurement.throat_width_m);
    measurement.measured = true;
    return measurement;
  }

  bool narrow_gap_allows(
      double center_deg,
      double center_range_m,
      double sector_clearance_m,
      const GapWidthMeasurement &measurement) const
  {
    if (!measurement.measured)
    {
      return false;
    }
    if (std::fabs(center_deg) > narrow_gap_search_deg_)
    {
      return false;
    }
    if (measurement.width_m < narrow_gap_min_width_m_ ||
        measurement.width_m > narrow_gap_max_width_m_)
    {
      return false;
    }
    if (center_range_m < narrow_gap_min_center_distance_m_ ||
        sector_clearance_m < narrow_gap_min_sector_distance_m_)
    {
      return false;
    }

    const double farther_boundary_range =
        std::max(measurement.left_range_m, measurement.right_range_m);
    return center_range_m >= farther_boundary_range + narrow_gap_min_depth_gain_m_;
  }

  bool find_narrow_gap_boundary(
      const sensor_msgs::msg::LaserScan &scan,
      double center_deg,
      double center_range_m,
      double direction,
      double &boundary_angle_rad,
      double &boundary_range_m) const
  {
    const double step_deg = std::max(1.0, scan_sample_step_deg_);
    const double max_offset = std::max(step_deg, narrow_gap_boundary_search_deg_);
    for (double offset_deg = step_deg; offset_deg <= max_offset; offset_deg += step_deg)
    {
      const double angle_deg = center_deg + sign_value(direction) * offset_deg;
      double range = 0.0;
      if (!range_at_angle(scan, deg_to_rad(angle_deg), range) ||
          !range_is_usable(scan, range))
      {
        continue;
      }

      const bool close_enough = range <= narrow_gap_boundary_obstacle_max_range_m_;
      const bool shallower_than_center =
          range <= center_range_m - std::max(0.0, narrow_gap_boundary_drop_m_);
      if (close_enough && shallower_than_center)
      {
        boundary_angle_rad = deg_to_rad(angle_deg);
        boundary_range_m = range;
        return true;
      }
    }

    return false;
  }

  void integrate_scan_if_new(
      const sensor_msgs::msg::LaserScan &scan,
      const rclcpp::Time &scan_time,
      const Pose2D &odom_pose)
  {
    if (scan.ranges.empty() || std::fabs(scan.angle_increment) < 1e-9)
    {
      return;
    }

    {
      std::lock_guard<std::mutex> lock(data_mutex_);
      if (scan_time.nanoseconds() == last_integrated_scan_time_.nanoseconds())
      {
        return;
      }
      last_integrated_scan_time_ = scan_time;
    }

    const double angle_step = std::max(1.0, scan_sample_step_deg_);
    const int index_step = std::max(
        1,
        static_cast<int>(std::round(deg_to_rad(angle_step) / std::fabs(scan.angle_increment))));

    std::vector<ObstaclePoint> new_points;
    for (int i = 0; i < static_cast<int>(scan.ranges.size()); i += index_step)
    {
      const double range = scan.ranges[static_cast<std::size_t>(i)];
      if (!range_is_obstacle(scan, range))
      {
        continue;
      }

      const double angle = scan.angle_min + static_cast<double>(i) * scan.angle_increment;
      const double local_x = range * std::cos(angle);
      const double local_y = range * std::sin(angle);
      if (local_x < -grid_back_m_ || local_x > grid_forward_m_ ||
          std::fabs(local_y) > grid_half_width_m_)
      {
        continue;
      }

      ObstaclePoint point;
      point.x = odom_pose.x + std::cos(odom_pose.yaw) * local_x - std::sin(odom_pose.yaw) * local_y;
      point.y = odom_pose.y + std::sin(odom_pose.yaw) * local_x + std::cos(odom_pose.yaw) * local_y;
      point.stamp = scan_time;
      new_points.push_back(point);
    }

    std::lock_guard<std::mutex> lock(data_mutex_);
    for (const auto &point : new_points)
    {
      obstacle_memory_.push_back(point);
    }
    while (static_cast<int>(obstacle_memory_.size()) > obstacle_memory_max_points_)
    {
      obstacle_memory_.pop_front();
    }
  }

  void prune_obstacle_memory(const rclcpp::Time &now)
  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    while (!obstacle_memory_.empty() &&
           (now - obstacle_memory_.front().stamp).seconds() > obstacle_memory_seconds_)
    {
      obstacle_memory_.pop_front();
    }
  }

  LocalCostmap build_local_costmap(
      const sensor_msgs::msg::LaserScan &scan,
      const Pose2D &odom_pose) const
  {
    LocalCostmap costmap;
    costmap.resolution = std::max(0.01, grid_resolution_m_);
    costmap.origin_x = -std::max(0.0, grid_back_m_);
    costmap.origin_y = -std::max(0.1, grid_half_width_m_);
    costmap.width = std::max(
        1,
        static_cast<int>(std::ceil((grid_forward_m_ + grid_back_m_) / costmap.resolution)));
    costmap.height = std::max(
        1,
        static_cast<int>(std::ceil((2.0 * grid_half_width_m_) / costmap.resolution)));
    const int cell_count = costmap.width * costmap.height;
    costmap.occupancy.assign(static_cast<std::size_t>(cell_count), -1);
    costmap.cost.assign(static_cast<std::size_t>(cell_count), 0.0);

    std::deque<ObstaclePoint> memory_copy;
    {
      std::lock_guard<std::mutex> lock(data_mutex_);
      memory_copy = obstacle_memory_;
    }

    for (const auto &point : memory_copy)
    {
      const double dx = point.x - odom_pose.x;
      const double dy = point.y - odom_pose.y;
      const double local_x = std::cos(odom_pose.yaw) * dx + std::sin(odom_pose.yaw) * dy;
      const double local_y = -std::sin(odom_pose.yaw) * dx + std::cos(odom_pose.yaw) * dy;
      int gx = 0;
      int gy = 0;
      if (world_to_grid(costmap, local_x, local_y, gx, gy))
      {
        mark_obstacle_cell(costmap, gx, gy);
      }
    }

    mark_scan_free_space(scan, costmap);
    mark_current_scan_obstacles(scan, costmap);
    inflate_obstacles(costmap);
    return costmap;
  }

  void mark_scan_free_space(
      const sensor_msgs::msg::LaserScan &scan,
      LocalCostmap &costmap) const
  {
    if (scan.ranges.empty() || std::fabs(scan.angle_increment) < 1e-9)
    {
      return;
    }

    const double angle_step = std::max(1.0, scan_sample_step_deg_);
    const int index_step = std::max(
        1,
        static_cast<int>(std::round(deg_to_rad(angle_step) / std::fabs(scan.angle_increment))));

    for (int i = 0; i < static_cast<int>(scan.ranges.size()); i += index_step)
    {
      const double raw_range = scan.ranges[static_cast<std::size_t>(i)];
      double trace_range = 0.0;
      if (!range_to_raytrace_distance(scan, raw_range, trace_range))
      {
        continue;
      }
      const double angle = scan.angle_min + static_cast<double>(i) * scan.angle_increment;
      const double step = std::max(0.01, costmap.resolution * 0.5);
      for (double dist = 0.0; dist <= trace_range; dist += step)
      {
        const double local_x = dist * std::cos(angle);
        const double local_y = dist * std::sin(angle);
        int gx = 0;
        int gy = 0;
        if (world_to_grid(costmap, local_x, local_y, gx, gy))
        {
          mark_free_cell(costmap, gx, gy);
        }
      }
    }
  }

  void mark_current_scan_obstacles(
      const sensor_msgs::msg::LaserScan &scan,
      LocalCostmap &costmap) const
  {
    if (scan.ranges.empty() || std::fabs(scan.angle_increment) < 1e-9)
    {
      return;
    }

    const double angle_step = std::max(1.0, scan_sample_step_deg_);
    const int index_step = std::max(
        1,
        static_cast<int>(std::round(deg_to_rad(angle_step) / std::fabs(scan.angle_increment))));

    for (int i = 0; i < static_cast<int>(scan.ranges.size()); i += index_step)
    {
      const double range = scan.ranges[static_cast<std::size_t>(i)];
      if (!range_is_obstacle(scan, range))
      {
        continue;
      }

      const double angle = scan.angle_min + static_cast<double>(i) * scan.angle_increment;
      const double local_x = range * std::cos(angle);
      const double local_y = range * std::sin(angle);
      int gx = 0;
      int gy = 0;
      if (world_to_grid(costmap, local_x, local_y, gx, gy))
      {
        mark_obstacle_cell(costmap, gx, gy);
      }
    }
  }

  void inflate_obstacles(LocalCostmap &costmap) const
  {
    std::vector<int> obstacle_indices;
    for (int i = 0; i < static_cast<int>(costmap.occupancy.size()); ++i)
    {
      if (costmap.occupancy[static_cast<std::size_t>(i)] == 100)
      {
        obstacle_indices.push_back(i);
      }
    }

    // The trajectory checker already samples the robot footprint with robot_radius_m_.
    // Inflation here is only the extra safety margin around obstacle cells.
    const double inflation_total = std::max(0.0, inflation_radius_m_);
    const int inflation_cells =
        static_cast<int>(std::ceil(inflation_total / costmap.resolution));

    for (const int index : obstacle_indices)
    {
      const int ox = index % costmap.width;
      const int oy = index / costmap.width;
      for (int dy = -inflation_cells; dy <= inflation_cells; ++dy)
      {
        for (int dx = -inflation_cells; dx <= inflation_cells; ++dx)
        {
          const int gx = ox + dx;
          const int gy = oy + dy;
          if (gx < 0 || gy < 0 || gx >= costmap.width || gy >= costmap.height)
          {
            continue;
          }
          const double dist = std::sqrt(static_cast<double>(dx * dx + dy * dy)) * costmap.resolution;
          if (dist > inflation_total && dist > 1e-9)
          {
            continue;
          }

          double inflated_cost = 100.0;
          if (dist > 1e-9 && inflation_total > 1e-6)
          {
            const double ratio = clamp_value(
                1.0 - dist / inflation_total,
                0.0,
                1.0);
            inflated_cost = std::max(1.0, 100.0 * ratio);
          }

          const int cell_index = gy * costmap.width + gx;
          costmap.cost[static_cast<std::size_t>(cell_index)] =
              std::max(costmap.cost[static_cast<std::size_t>(cell_index)], inflated_cost);
          const int8_t occupancy_value =
              static_cast<int8_t>(clamp_value(std::round(inflated_cost), 0.0, 100.0));
          if (costmap.occupancy[static_cast<std::size_t>(cell_index)] < occupancy_value)
          {
            costmap.occupancy[static_cast<std::size_t>(cell_index)] = occupancy_value;
          }
        }
      }
    }
  }

  bool world_to_grid(
      const LocalCostmap &costmap,
      double x,
      double y,
      int &gx,
      int &gy) const
  {
    gx = static_cast<int>(std::floor((x - costmap.origin_x) / costmap.resolution));
    gy = static_cast<int>(std::floor((y - costmap.origin_y) / costmap.resolution));
    return gx >= 0 && gy >= 0 && gx < costmap.width && gy < costmap.height;
  }

  void mark_free_cell(LocalCostmap &costmap, int gx, int gy) const
  {
    const int index = gy * costmap.width + gx;
    if (index < 0 || index >= static_cast<int>(costmap.occupancy.size()))
    {
      return;
    }
    costmap.occupancy[static_cast<std::size_t>(index)] = 0;
    costmap.cost[static_cast<std::size_t>(index)] = 0.0;
  }

  void mark_obstacle_cell(LocalCostmap &costmap, int gx, int gy) const
  {
    const int index = gy * costmap.width + gx;
    if (index < 0 || index >= static_cast<int>(costmap.occupancy.size()))
    {
      return;
    }
    costmap.occupancy[static_cast<std::size_t>(index)] = 100;
    costmap.cost[static_cast<std::size_t>(index)] = 100.0;
  }

  bool range_is_obstacle(const sensor_msgs::msg::LaserScan &scan, double range) const
  {
    if (!range_is_usable(scan, range))
    {
      return false;
    }
    return range <= scan_obstacle_max_range_m_;
  }

  bool range_to_raytrace_distance(
      const sensor_msgs::msg::LaserScan &scan,
      double range,
      double &trace_range) const
  {
    trace_range = 0.0;

    if (!range_is_usable(scan, range))
    {
      return false;
    }

    trace_range = std::min(range, raytrace_max_range_m_);
    return trace_range > 0.0;
  }

  bool range_to_long_range_clearance(
      const sensor_msgs::msg::LaserScan &scan,
      double range,
      double &clearance) const
  {
    clearance = 0.0;

    if (!range_is_usable(scan, range))
    {
      return false;
    }

    const double max_distance = effective_long_range_max(scan);
    clearance = clamp_value(range, 0.0, max_distance);
    return clearance > 0.0;
  }

  double effective_long_range_max(const sensor_msgs::msg::LaserScan &scan) const
  {
    double max_distance = std::max(0.1, long_range_clearance_max_range_m_);
    if (std::isfinite(scan.range_max) && scan.range_max > 0.0)
    {
      max_distance = std::min(max_distance, static_cast<double>(scan.range_max));
    }
    return std::max(max_distance, long_range_clearance_min_range_m_ + 0.01);
  }

  double minimum_usable_scan_range(const sensor_msgs::msg::LaserScan &scan) const
  {
    return std::max(static_cast<double>(std::max(0.0f, scan.range_min)), scan_obstacle_min_range_m_);
  }

  bool range_is_usable(const sensor_msgs::msg::LaserScan &scan, double range) const
  {
    if (!std::isfinite(range))
    {
      return false;
    }
    if (range <= minimum_usable_scan_range(scan))
    {
      return false;
    }
    if (std::isfinite(scan.range_max) && range > scan.range_max)
    {
      return false;
    }
    return true;
  }

  double min_range_in_sector(
      const sensor_msgs::msg::LaserScan &scan,
      double center_deg,
      double half_width_deg) const
  {
    double min_range = scan_obstacle_max_range_m_;
    bool sampled = false;
    const double step_deg = std::max(1.0, scan_sample_step_deg_);

    for (double offset_deg = -half_width_deg; offset_deg <= half_width_deg; offset_deg += step_deg)
    {
      double range = 0.0;
      if (range_at_angle(scan, deg_to_rad(center_deg + offset_deg), range))
      {
        if (!range_is_usable(scan, range))
        {
          continue;
        }
        min_range = std::min(min_range, sanitize_range(scan, range));
        sampled = true;
      }
    }

    return sampled ? min_range : 0.0;
  }

  bool range_at_angle(
      const sensor_msgs::msg::LaserScan &scan,
      double target_angle_rad,
      double &range) const
  {
    if (scan.ranges.empty() || std::fabs(scan.angle_increment) < 1e-9)
    {
      return false;
    }

    double angle = target_angle_rad;
    while (angle < scan.angle_min)
    {
      angle += kTwoPi;
    }
    while (angle > scan.angle_max)
    {
      angle -= kTwoPi;
    }

    if (angle < scan.angle_min || angle > scan.angle_max)
    {
      return false;
    }

    const int index =
        static_cast<int>(std::lround((angle - scan.angle_min) / scan.angle_increment));
    if (index < 0 || index >= static_cast<int>(scan.ranges.size()))
    {
      return false;
    }

    range = scan.ranges[static_cast<std::size_t>(index)];
    return true;
  }

  double sanitize_range(
      const sensor_msgs::msg::LaserScan &scan,
      double range) const
  {
    if (!std::isfinite(range) || range <= minimum_usable_scan_range(scan))
    {
      return scan_obstacle_max_range_m_;
    }
    if (std::isfinite(scan.range_max) && range > scan.range_max)
    {
      return scan_obstacle_max_range_m_;
    }
    return clamp_value(range, 0.0, scan_obstacle_max_range_m_);
  }

  double choose_recovery_turn(double left, double right)
  {
    const rclcpp::Time now = this->now();

    if (!recovery_active_)
    {
      recovery_active_ = true;
      recovery_started_at_ = now;

      if (std::fabs(left - right) < 0.05)
      {
        recovery_direction_ = prefer_left_recovery_ ? 1 : -1;
      }
      else
      {
        recovery_direction_ = left > right ? 1 : -1;
      }
    }
    else if ((now - recovery_started_at_).seconds() > recovery_flip_seconds_)
    {
      recovery_direction_ *= -1;
      recovery_started_at_ = now;
    }

    return sign_value(static_cast<double>(recovery_direction_)) * recovery_turn_speed_;
  }

  void reset_recovery_state()
  {
    recovery_active_ = false;
  }

  bool sensors_current_locked(const rclcpp::Time &now) const
  {
    const bool imu_current =
        imu_ready_ && (now - latest_imu_time_).seconds() <= imu_timeout_seconds_;
    const bool odom_current =
        odom_ready_ && (now - latest_odom_time_).seconds() <= odom_timeout_seconds_;
    return imu_current && (!require_odom_ || odom_current);
  }

  bool reset_heading_reference_from_current()
  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    if (!imu_ready_)
    {
      return false;
    }

    set_heading_reference_locked();
    RCLCPP_INFO(this->get_logger(), "Gyro yaw reference reset");
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
    blocked_recovery_candidate_active_ = false;
    stuck_recovery_phase_ = StuckRecoveryPhase::IDLE;
    recovery_source_ = RecoverySource::NONE;
    held_narrow_gap_target_ = NarrowGapTarget();
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
    snapshot.odom_pose = odom_pose_;
    snapshot.odom_linear_speed = odom_linear_speed_;
    snapshot.latest_imu_time = latest_imu_time_;
    snapshot.latest_odom_time = latest_odom_time_;
    return snapshot;
  }

  bool safety_data_ready(
      const SafetySnapshot &safety,
      const rclcpp::Time &now,
      std::string &wait_reason) const
  {
    if (!safety.imu_ready)
    {
      wait_reason = "waiting for imu";
      return false;
    }
    if ((now - safety.latest_imu_time).seconds() > imu_timeout_seconds_)
    {
      wait_reason = "imu timeout";
      return false;
    }
    if (require_odom_ && !safety.odom_ready)
    {
      wait_reason = "waiting for odom";
      return false;
    }
    if (require_odom_ && (now - safety.latest_odom_time).seconds() > odom_timeout_seconds_)
    {
      wait_reason = "odom timeout";
      return false;
    }
    return true;
  }

  void annotate_output(DriveOutput &output, const SafetySnapshot &safety) const
  {
    output.enabled_requested = safety.enabled_requested;
    output.imu_ready = safety.imu_ready;
    output.odom_ready = safety.odom_ready;
    output.heading_reference_valid = safety.heading_reference_valid;
    output.tilt_deg = safety.max_tilt_deg;

    if (safety.heading_reference_valid)
    {
      output.heading_error_deg =
          rad_to_deg(normalize_angle_rad(safety.current_heading_rad - safety.heading_reference_rad));
    }
  }

  void apply_reverse_guard(DriveOutput &output, const rclcpp::Time &now)
  {
    double heading_error_rad = 0.0;
    double heading_error_abs_deg = 0.0;
    bool run_recovery = false;
    bool recovery_resumed = false;

    {
      std::lock_guard<std::mutex> lock(data_mutex_);
      if (!heading_reference_valid_)
      {
        return;
      }

      heading_error_rad = normalize_angle_rad(current_heading_rad_ - heading_reference_rad_);
      heading_error_abs_deg = std::fabs(rad_to_deg(heading_error_rad));
      output.heading_error_deg = rad_to_deg(heading_error_rad);

      if (reverse_recovery_active_)
      {
        if (heading_error_abs_deg <= reverse_resume_threshold_deg_)
        {
          reverse_recovery_active_ = false;
          reverse_condition_active_ = false;
          recovery_resumed = true;
        }
        else
        {
          run_recovery = true;
        }
      }
      else if (
          output.linear_x > reverse_linear_threshold_ &&
          heading_error_abs_deg >= reverse_heading_threshold_deg_)
      {
        if (!reverse_condition_active_)
        {
          reverse_condition_active_ = true;
          reverse_condition_started_at_ = now;
        }

        if ((now - reverse_condition_started_at_).seconds() >= reverse_hold_seconds_)
        {
          reverse_recovery_active_ = true;
          run_recovery = true;
        }
        else
        {
          output.mode = "REVERSE_WARN";
        }
      }
      else
      {
        reverse_condition_active_ = false;
      }
    }

    if (recovery_resumed)
    {
      return;
    }

    if (run_recovery)
    {
      const double recovery_direction = heading_error_rad > 0.0 ? -1.0 : 1.0;
      output.linear_x = 0.0;
      output.angular_z = recovery_direction * heading_recovery_turn_speed_;
      output.mode = "REVERSE_RECOVERY";
      output.detail = "turning back to start heading";
      reset_recovery_state();
    }
  }

  void apply_stuck_recovery(
      DriveOutput &output,
      const rclcpp::Time &now,
      const LocalCostmap &costmap)
  {
    if (stuck_recovery_phase_ != StuckRecoveryPhase::IDLE)
    {
      update_stuck_recovery_command(output, now, costmap);
      return;
    }

    const bool blocked_mode = is_blocked_recovery_candidate(output.mode);
    if (!blocked_mode)
    {
      blocked_recovery_candidate_active_ = false;
    }

    if (stuck_recovery_finished_once_ &&
        (now - last_stuck_recovery_finished_at_).seconds() < stuck_cooldown_seconds_)
    {
      return;
    }

    if (blocked_recovery_enabled_ && blocked_mode)
    {
      apply_blocked_recovery_candidate(output, now, costmap);
      return;
    }

    if (!stuck_recovery_enabled_)
    {
      return;
    }

    if (output.linear_x <= stuck_command_linear_threshold_)
    {
      stuck_candidate_active_ = false;
      return;
    }

    bool odom_ready = false;
    double odom_age = 0.0;
    Pose2D odom_pose;
    double odom_linear_speed = 0.0;

    {
      std::lock_guard<std::mutex> lock(data_mutex_);
      odom_ready = odom_ready_;
      odom_age = (now - latest_odom_time_).seconds();
      odom_pose = odom_pose_;
      odom_linear_speed = odom_linear_speed_;
    }

    if (!odom_ready || odom_age > odom_timeout_seconds_)
    {
      stuck_candidate_active_ = false;
      return;
    }

    if (odom_linear_speed > stuck_odom_linear_threshold_)
    {
      stuck_candidate_active_ = false;
      return;
    }

    if (!stuck_candidate_active_)
    {
      stuck_candidate_active_ = true;
      stuck_candidate_started_at_ = now;
      stuck_candidate_x_ = odom_pose.x;
      stuck_candidate_y_ = odom_pose.y;
      return;
    }

    const double dx = odom_pose.x - stuck_candidate_x_;
    const double dy = odom_pose.y - stuck_candidate_y_;
    const double progress = std::sqrt(dx * dx + dy * dy);
    if (progress > stuck_min_progress_m_)
    {
      stuck_candidate_active_ = false;
      return;
    }

    if ((now - stuck_candidate_started_at_).seconds() >= stuck_hold_seconds_)
    {
      start_recovery(output, now, RecoverySource::STUCK);
      update_stuck_recovery_command(output, now, costmap);
    }
  }

  bool is_blocked_recovery_candidate(const std::string &mode) const
  {
    return mode == "BLOCKED_TURN" || mode == "EMERGENCY_TURN";
  }

  void apply_blocked_recovery_candidate(
      DriveOutput &output,
      const rclcpp::Time &now,
      const LocalCostmap &costmap)
  {
    stuck_candidate_active_ = false;

    if (!blocked_recovery_candidate_active_)
    {
      blocked_recovery_candidate_active_ = true;
      blocked_recovery_candidate_started_at_ = now;
      return;
    }

    if ((now - blocked_recovery_candidate_started_at_).seconds() >=
        blocked_recovery_hold_seconds_)
    {
      start_recovery(output, now, RecoverySource::BLOCKED);
      update_stuck_recovery_command(output, now, costmap);
    }
  }

  void start_recovery(
      const DriveOutput &output,
      const rclcpp::Time &now,
      RecoverySource source)
  {
    stuck_candidate_active_ = false;
    blocked_recovery_candidate_active_ = false;
    stuck_recovery_phase_ = StuckRecoveryPhase::BACKUP;
    recovery_source_ = source;
    stuck_phase_started_at_ = now;
    stuck_turn_direction_ = choose_stuck_turn_direction(output.costmap);
    reverse_condition_active_ = false;
    reverse_recovery_active_ = false;
    reset_recovery_state();
  }

  int choose_stuck_turn_direction(const CostmapSummary &summary) const
  {
    if (std::fabs(summary.left_m - summary.right_m) < 0.05)
    {
      return prefer_left_recovery_ ? 1 : -1;
    }
    return summary.left_m > summary.right_m ? 1 : -1;
  }

  void update_stuck_recovery_command(
      DriveOutput &output,
      const rclcpp::Time &now,
      const LocalCostmap &costmap)
  {
    double phase_elapsed = (now - stuck_phase_started_at_).seconds();

    if (stuck_recovery_phase_ == StuckRecoveryPhase::BACKUP &&
        !backup_path_is_clear(costmap))
    {
      stuck_recovery_phase_ = StuckRecoveryPhase::TURN;
      stuck_phase_started_at_ = now;
      phase_elapsed = 0.0;
    }

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
      recovery_source_ = RecoverySource::NONE;
      stuck_recovery_finished_once_ = true;
      last_stuck_recovery_finished_at_ = now;
      return;
    }

    if (stuck_recovery_phase_ == StuckRecoveryPhase::BACKUP)
    {
      output.linear_x = -std::fabs(stuck_backup_speed_);
      output.angular_z = 0.0;
      if (recovery_source_ == RecoverySource::BLOCKED)
      {
        output.mode = "BLOCKED_BACKUP";
        output.detail = "blocked turn persisted, backing up for a wider lidar view";
      }
      else
      {
        output.mode = "STUCK_BACKUP";
        output.detail = "odom shows little progress";
      }
      return;
    }

    if (stuck_recovery_phase_ == StuckRecoveryPhase::TURN)
    {
      output.linear_x = 0.0;
      output.angular_z =
          sign_value(static_cast<double>(stuck_turn_direction_)) * std::fabs(stuck_turn_speed_);
      if (recovery_source_ == RecoverySource::BLOCKED)
      {
        output.mode = "BLOCKED_ESCAPE_TURN";
        output.detail = "turning after blocked backup or unsafe reverse";
      }
      else
      {
        output.mode = "STUCK_TURN";
        output.detail = "turning after backup or unsafe reverse";
      }
    }
  }

  bool backup_path_is_clear(const LocalCostmap &costmap) const
  {
    const double backup_speed = std::fabs(stuck_backup_speed_);
    const double backup_duration = std::max(0.0, stuck_backup_seconds_);
    const double backup_distance = backup_speed * backup_duration;
    if (backup_distance <= 1e-6)
    {
      return false;
    }

    const double step_distance = std::max(0.01, costmap.resolution);
    const int steps =
        std::max(1, static_cast<int>(std::ceil(backup_distance / step_distance)));
    for (int step = 0; step <= steps; ++step)
    {
      const double ratio = static_cast<double>(step) / static_cast<double>(steps);
      const double x = -backup_distance * ratio;
      if (!backup_footprint_is_clear(costmap, x, 0.0))
      {
        return false;
      }
    }
    return true;
  }

  bool backup_footprint_is_clear(
      const LocalCostmap &costmap,
      double x,
      double y) const
  {
    const int perimeter_samples = 12;
    if (!backup_cell_is_clear(costmap, x, y))
    {
      return false;
    }

    for (int i = 0; i < perimeter_samples; ++i)
    {
      const double angle = kTwoPi * static_cast<double>(i) / static_cast<double>(perimeter_samples);
      const double px = x + robot_radius_m_ * std::cos(angle);
      const double py = y + robot_radius_m_ * std::sin(angle);
      if (!backup_cell_is_clear(costmap, px, py))
      {
        return false;
      }
    }
    return true;
  }

  bool backup_cell_is_clear(
      const LocalCostmap &costmap,
      double x,
      double y) const
  {
    int gx = 0;
    int gy = 0;
    if (!world_to_grid(costmap, x, y, gx, gy))
    {
      return false;
    }

    const int index = gy * costmap.width + gx;
    if (index < 0 || index >= static_cast<int>(costmap.occupancy.size()))
    {
      return false;
    }
    if (costmap.occupancy[static_cast<std::size_t>(index)] < 0)
    {
      return false;
    }
    return costmap.cost[static_cast<std::size_t>(index)] < lethal_cost_threshold_;
  }

  void publish_local_costmap(const LocalCostmap &costmap)
  {
    nav_msgs::msg::OccupancyGrid msg;
    msg.header.stamp = this->now();
    msg.header.frame_id = costmap_frame_id_;
    msg.info.resolution = static_cast<float>(costmap.resolution);
    msg.info.width = static_cast<uint32_t>(costmap.width);
    msg.info.height = static_cast<uint32_t>(costmap.height);
    msg.info.origin.position.x = costmap.origin_x;
    msg.info.origin.position.y = costmap.origin_y;
    msg.info.origin.position.z = 0.0;
    msg.info.origin.orientation.w = 1.0;
    msg.data = costmap.occupancy;
    local_costmap_publisher_->publish(msg);
  }

  void publish_trajectory_markers(const DriveOutput &output)
  {
    visualization_msgs::msg::MarkerArray array;

    if (!output.selected_path.empty())
    {
      visualization_msgs::msg::Marker marker;
      marker.header.stamp = this->now();
      marker.header.frame_id = costmap_frame_id_;
      marker.ns = "selected_trajectory";
      marker.id = 1;
      marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
      marker.action = visualization_msgs::msg::Marker::ADD;
      marker.scale.x = 0.05;
      marker.color.r = 0.0f;
      marker.color.g = 0.45f;
      marker.color.b = 1.0f;
      marker.color.a = 1.0f;
      marker.lifetime.sec = 0;
      marker.lifetime.nanosec = 500000000;
      marker.pose.orientation.w = 1.0;
      for (const auto &pose : output.selected_path)
      {
        geometry_msgs::msg::Point point;
        point.x = pose.x;
        point.y = pose.y;
        point.z = 0.08;
        marker.points.push_back(point);
      }
      array.markers.push_back(marker);
    }
    else
    {
      visualization_msgs::msg::Marker delete_marker;
      delete_marker.header.stamp = this->now();
      delete_marker.header.frame_id = costmap_frame_id_;
      delete_marker.ns = "selected_trajectory";
      delete_marker.id = 1;
      delete_marker.action = visualization_msgs::msg::Marker::DELETE;
      array.markers.push_back(delete_marker);
    }

    if (output.costmap.long_range_target.valid)
    {
      visualization_msgs::msg::Marker target_marker;
      target_marker.header.stamp = this->now();
      target_marker.header.frame_id = costmap_frame_id_;
      target_marker.ns = "long_range_target";
      target_marker.id = 2;
      target_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
      target_marker.action = visualization_msgs::msg::Marker::ADD;
      target_marker.scale.x = 0.035;
      target_marker.color.r = 0.0f;
      target_marker.color.g = 1.0f;
      target_marker.color.b = 0.25f;
      target_marker.color.a = 0.9f;
      target_marker.lifetime.sec = 0;
      target_marker.lifetime.nanosec = 500000000;
      target_marker.pose.orientation.w = 1.0;

      geometry_msgs::msg::Point origin;
      origin.x = 0.0;
      origin.y = 0.0;
      origin.z = 0.11;
      target_marker.points.push_back(origin);

      geometry_msgs::msg::Point target;
      target.x = output.costmap.long_range_target.distance_m *
                 std::cos(output.costmap.long_range_target.angle_rad);
      target.y = output.costmap.long_range_target.distance_m *
                 std::sin(output.costmap.long_range_target.angle_rad);
      target.z = 0.11;
      target_marker.points.push_back(target);
      array.markers.push_back(target_marker);
    }
    else
    {
      visualization_msgs::msg::Marker delete_target_marker;
      delete_target_marker.header.stamp = this->now();
      delete_target_marker.header.frame_id = costmap_frame_id_;
      delete_target_marker.ns = "long_range_target";
      delete_target_marker.id = 2;
      delete_target_marker.action = visualization_msgs::msg::Marker::DELETE;
      array.markers.push_back(delete_target_marker);
    }

    if (output.costmap.narrow_gap_target.valid)
    {
      visualization_msgs::msg::Marker gate_marker;
      gate_marker.header.stamp = this->now();
      gate_marker.header.frame_id = costmap_frame_id_;
      gate_marker.ns = "narrow_gap_target";
      gate_marker.id = 3;
      gate_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
      gate_marker.action = visualization_msgs::msg::Marker::ADD;
      gate_marker.scale.x = 0.04;
      gate_marker.color.r = 1.0f;
      gate_marker.color.g = 0.62f;
      gate_marker.color.b = 0.0f;
      gate_marker.color.a = 0.95f;
      gate_marker.lifetime.sec = 0;
      gate_marker.lifetime.nanosec = 500000000;
      gate_marker.pose.orientation.w = 1.0;

      geometry_msgs::msg::Point origin;
      origin.x = 0.0;
      origin.y = 0.0;
      origin.z = 0.14;
      gate_marker.points.push_back(origin);

      geometry_msgs::msg::Point target;
      target.x = output.costmap.narrow_gap_target.center_range_m *
                 std::cos(output.costmap.narrow_gap_target.angle_rad);
      target.y = output.costmap.narrow_gap_target.center_range_m *
                 std::sin(output.costmap.narrow_gap_target.angle_rad);
      target.z = 0.14;
      gate_marker.points.push_back(target);
      array.markers.push_back(gate_marker);

      if (output.costmap.narrow_gap_target.width_measurement.measured)
      {
        visualization_msgs::msg::Marker width_marker;
        width_marker.header.stamp = this->now();
        width_marker.header.frame_id = costmap_frame_id_;
        width_marker.ns = "narrow_gap_width";
        width_marker.id = 4;
        width_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
        width_marker.action = visualization_msgs::msg::Marker::ADD;
        width_marker.scale.x = 0.025;
        width_marker.color.r = 1.0f;
        width_marker.color.g = 0.95f;
        width_marker.color.b = 0.0f;
        width_marker.color.a = 0.9f;
        width_marker.lifetime.sec = 0;
        width_marker.lifetime.nanosec = 500000000;
        width_marker.pose.orientation.w = 1.0;

        geometry_msgs::msg::Point left;
        left.x = output.costmap.narrow_gap_target.width_measurement.left_range_m *
                 std::cos(output.costmap.narrow_gap_target.width_measurement.left_angle_rad);
        left.y = output.costmap.narrow_gap_target.width_measurement.left_range_m *
                 std::sin(output.costmap.narrow_gap_target.width_measurement.left_angle_rad);
        left.z = 0.145;
        width_marker.points.push_back(left);

        geometry_msgs::msg::Point right;
        right.x = output.costmap.narrow_gap_target.width_measurement.right_range_m *
                  std::cos(output.costmap.narrow_gap_target.width_measurement.right_angle_rad);
        right.y = output.costmap.narrow_gap_target.width_measurement.right_range_m *
                  std::sin(output.costmap.narrow_gap_target.width_measurement.right_angle_rad);
        right.z = 0.145;
        width_marker.points.push_back(right);
        array.markers.push_back(width_marker);
      }
    }
    else
    {
      visualization_msgs::msg::Marker delete_gate_marker;
      delete_gate_marker.header.stamp = this->now();
      delete_gate_marker.header.frame_id = costmap_frame_id_;
      delete_gate_marker.ns = "narrow_gap_target";
      delete_gate_marker.id = 3;
      delete_gate_marker.action = visualization_msgs::msg::Marker::DELETE;
      array.markers.push_back(delete_gate_marker);

      visualization_msgs::msg::Marker delete_width_marker;
      delete_width_marker.header.stamp = this->now();
      delete_width_marker.header.frame_id = costmap_frame_id_;
      delete_width_marker.ns = "narrow_gap_width";
      delete_width_marker.id = 4;
      delete_width_marker.action = visualization_msgs::msg::Marker::DELETE;
      array.markers.push_back(delete_width_marker);
    }

    trajectory_marker_publisher_->publish(array);
  }

  void publish_velocity(double linear_x, double angular_z)
  {
    last_command_angular_z_ = angular_z;

    if (cmd_vel_stamped_)
    {
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

  void store_output(const DriveOutput &output)
  {
    std::lock_guard<std::mutex> lock(output_mutex_);
    last_output_ = output;
  }

  int obstacle_memory_size() const
  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    return static_cast<int>(obstacle_memory_.size());
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
        "front=%.2fm left=%.2fm right=%.2fm narrow=%s obs=%d traj=%d/%d "
        "score=%.2f avg_cost=%.1f unknown=%.2f long=%s %.0fdeg/%.2fm "
        "gate=%s%s %.0fdeg/%.0fmm/%.2fm "
        "target_w=%.2f cmd=(%.2f, %.2f) %s",
        output.mode.c_str(),
        output.enabled_requested ? "yes" : "no",
        output.imu_ready ? "yes" : "no",
        output.odom_ready ? "yes" : "no",
        output.heading_reference_valid ? "yes" : "no",
        output.tilt_deg,
        output.heading_error_deg,
        output.costmap.front_m,
        output.costmap.left_m,
        output.costmap.right_m,
        output.costmap.narrow_corridor ? "yes" : "no",
        output.costmap.obstacle_points,
        output.costmap.evaluated_trajectories,
        output.costmap.rejected_trajectories,
        output.costmap.best_score,
        output.costmap.best_avg_cost,
        output.costmap.best_unknown_ratio,
        output.costmap.long_range_target.valid ? "yes" : "no",
        output.costmap.long_range_target.valid
            ? rad_to_deg(output.costmap.long_range_target.angle_rad)
            : 0.0,
        output.costmap.long_range_target.valid
            ? output.costmap.long_range_target.distance_m
            : 0.0,
        output.costmap.narrow_gap_target.valid ? "yes" : "no",
        output.costmap.narrow_gap_target.held ? "*" : "",
        output.costmap.narrow_gap_target.valid
            ? rad_to_deg(output.costmap.narrow_gap_target.angle_rad)
            : 0.0,
        output.costmap.narrow_gap_target.valid
            ? output.costmap.narrow_gap_target.width_m * 1000.0
            : 0.0,
        output.costmap.narrow_gap_target.valid
            ? output.costmap.narrow_gap_target.center_range_m
            : 0.0,
        output.costmap.best_angular_z,
        output.linear_x,
        output.angular_z,
        output.detail.c_str());
  }

  const std::string scan_topic_;
  const std::string imu_topic_;
  const std::string odom_topic_;
  const std::string cmd_vel_topic_;
  const bool cmd_vel_stamped_;
  const std::string local_costmap_topic_;
  const std::string trajectory_marker_topic_;
  const std::string costmap_frame_id_;
  const bool auto_start_;
  const bool reset_heading_on_start_;
  const bool require_odom_;
  const int control_period_ms_;
  const int report_period_ms_;
  const double max_linear_speed_;
  const double min_linear_speed_;
  const double max_angular_speed_;
  const double recovery_turn_speed_;
  const double recovery_flip_seconds_;
  const bool prefer_left_recovery_;
  const double emergency_stop_distance_m_;
  const double front_stop_distance_m_;
  const double front_slow_distance_m_;
  const double side_angle_deg_;
  const double side_window_deg_;
  const double front_window_deg_;
  const double scan_sample_step_deg_;
  const double scan_obstacle_min_range_m_;
  const double scan_obstacle_max_range_m_;
  const double raytrace_max_range_m_;
  const double lost_scan_timeout_seconds_;
  const double grid_resolution_m_;
  const double grid_forward_m_;
  const double grid_back_m_;
  const double grid_half_width_m_;
  const double obstacle_memory_seconds_;
  const int obstacle_memory_max_points_;
  const double robot_radius_m_;
  const double inflation_radius_m_;
  const double lethal_cost_threshold_;
  const double unknown_cell_cost_;
  const bool allow_unknown_trajectory_;
  const double trajectory_horizon_s_;
  const double trajectory_dt_s_;
  const int angular_sample_count_;
  const double turn_slowdown_gain_;
  const double progress_reward_weight_;
  const double cost_obstacle_weight_;
  const double cost_unknown_weight_;
  const double cost_turn_weight_;
  const double cost_smooth_weight_;
  const double cost_lateral_weight_;
  const double heading_alignment_weight_;
  const bool long_range_clearance_enabled_;
  const double long_range_clearance_weight_;
  const double long_range_clearance_max_range_m_;
  const double long_range_clearance_min_range_m_;
  const double long_range_clearance_search_deg_;
  const double long_range_clearance_window_deg_;
  const double long_range_clearance_alignment_deg_;
  const double long_range_clearance_start_bias_;
  const bool narrow_gap_target_enabled_;
  const double narrow_gap_bonus_weight_;
  const double narrow_gap_min_width_m_;
  const double narrow_gap_max_width_m_;
  const double narrow_gap_search_deg_;
  const double narrow_gap_sector_half_width_deg_;
  const double narrow_gap_boundary_search_deg_;
  const double narrow_gap_boundary_obstacle_max_range_m_;
  const double narrow_gap_boundary_drop_m_;
  const double narrow_gap_min_center_distance_m_;
  const double narrow_gap_min_sector_distance_m_;
  const double narrow_gap_min_depth_gain_m_;
  const double narrow_gap_alignment_deg_;
  const double narrow_gap_hold_seconds_;
  const double narrow_gap_hold_max_angle_error_deg_;
  const double narrow_gap_speed_m_s_;
  const bool narrow_corridor_enabled_;
  const double narrow_corridor_width_m_;
  const double narrow_corridor_side_detect_m_;
  const double narrow_corridor_speed_m_s_;
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
  const bool blocked_recovery_enabled_;
  const double blocked_recovery_hold_seconds_;

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_publisher_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr cmd_vel_stamped_publisher_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr local_costmap_publisher_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr trajectory_marker_publisher_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_subscription_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscription_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscription_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr set_enabled_service_;
  rclcpp::TimerBase::SharedPtr control_timer_;
  rclcpp::TimerBase::SharedPtr report_timer_;

  mutable std::mutex data_mutex_;
  sensor_msgs::msg::LaserScan::SharedPtr latest_scan_;
  rclcpp::Time latest_scan_time_;
  rclcpp::Time last_integrated_scan_time_;
  std::deque<ObstaclePoint> obstacle_memory_;
  bool enabled_requested_ = false;
  bool imu_ready_ = false;
  bool odom_ready_ = false;
  bool heading_reference_valid_ = false;
  double roll_rad_ = 0.0;
  double pitch_rad_ = 0.0;
  double max_tilt_deg_ = 0.0;
  double current_heading_rad_ = 0.0;
  double heading_reference_rad_ = 0.0;
  double odom_linear_speed_ = 0.0;
  Pose2D odom_pose_;
  rclcpp::Time latest_imu_time_;
  rclcpp::Time latest_odom_time_;

  bool recovery_active_ = false;
  int recovery_direction_ = 1;
  rclcpp::Time recovery_started_at_;

  bool reverse_condition_active_ = false;
  bool reverse_recovery_active_ = false;
  rclcpp::Time reverse_condition_started_at_;

  bool stuck_candidate_active_ = false;
  rclcpp::Time stuck_candidate_started_at_;
  double stuck_candidate_x_ = 0.0;
  double stuck_candidate_y_ = 0.0;
  bool blocked_recovery_candidate_active_ = false;
  rclcpp::Time blocked_recovery_candidate_started_at_;
  StuckRecoveryPhase stuck_recovery_phase_ = StuckRecoveryPhase::IDLE;
  RecoverySource recovery_source_ = RecoverySource::NONE;
  rclcpp::Time stuck_phase_started_at_;
  int stuck_turn_direction_ = 1;
  bool stuck_recovery_finished_once_ = false;
  rclcpp::Time last_stuck_recovery_finished_at_;

  NarrowGapTarget held_narrow_gap_target_;
  rclcpp::Time held_narrow_gap_target_updated_at_;

  double last_command_angular_z_ = 0.0;

  std::mutex output_mutex_;
  DriveOutput last_output_;
};

int main(int argc, char *argv[])
{
  signal(SIGINT, signal_handler);
  rclcpp::init(argc, argv);

  auto node = std::make_shared<TurtlebotAutonomousCostmapDrive>();
  rclcpp::Rate rate(20.0);
  while (rclcpp::ok() && g_keep_running)
  {
    rclcpp::spin_some(node);
    rate.sleep();
  }

  node->publish_stop_command();
  rclcpp::shutdown();
  return 0;
}
