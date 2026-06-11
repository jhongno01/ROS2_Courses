#include <signal.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <deque>
#include <limits>
#include <memory>
#include <mutex>
#include <string>
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
constexpr double kFree = 0.0;
constexpr double kUnknown = 1.0;
constexpr double kBlocked = 2.0;

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

struct Vector2D
{
  double x = 0.0;
  double y = 0.0;
};

struct Pose2D
{
  double x = 0.0;
  double y = 0.0;
  double yaw = 0.0;
};

struct MemoryPoint
{
  double x = 0.0;
  double y = 0.0;
  rclcpp::Time stamp;
};

struct DeadEndMemory
{
  double x = 0.0;
  double y = 0.0;
  double yaw = 0.0;
  rclcpp::Time stamp;
  int hit_count = 1;
};

struct LocalGrid
{
  int width = 0;
  int height = 0;
  double resolution = 0.02;
  double radius = 2.0;
  double origin_x = -2.0;
  double origin_y = -2.0;
  std::vector<double> cells;
};

struct DirectionCandidate
{
  double angle_rad = 0.0;
  double score = std::numeric_limits<double>::infinity();
  double min_blocked_distance = std::numeric_limits<double>::infinity();
  bool driveable = false;
};

struct GapWidthMeasurement
{
  bool measured = false;
  double left_angle_rad = 0.0;
  double right_angle_rad = 0.0;
  double left_range_m = 0.0;
  double right_range_m = 0.0;
  double width_m = 0.0;
};

struct GapSector
{
  double angle_rad = 0.0;
  double half_width_rad = 0.0;
  double center_range_m = 0.0;
  double width_m = 0.0;
};

struct VectorSummary
{
  Vector2D lowest_cost;
  Vector2D repulsion;
  Vector2D heading_bias;
  Vector2D reverse_bias;
  Vector2D final_vector;
  double best_angle_rad = 0.0;
  double best_score = std::numeric_limits<double>::infinity();
  double front_m = std::numeric_limits<double>::infinity();
  int blocked_directions = 0;
  int sampled_directions = 0;
  bool has_drive_direction = false;
  bool reverse_bias_active = false;
  bool dead_end_memory_enabled = false;
  int dead_end_memory_count = 0;
};

struct DriveOutput
{
  double linear_x = 0.0;
  double angular_z = 0.0;
  std::string mode = "WAIT_SCAN";
  std::string detail;
  bool enabled_requested = false;
  bool imu_ready = false;
  bool odom_ready = false;
  bool heading_reference_valid = false;
  double tilt_deg = 0.0;
  double heading_error_deg = 0.0;
  double path_distance_m = 0.0;
  VectorSummary vectors;
};

struct SafetySnapshot
{
  bool enabled_requested = false;
  bool imu_ready = false;
  bool odom_ready = false;
  bool heading_reference_valid = false;
  rclcpp::Time latest_imu_time;
  rclcpp::Time latest_odom_time;
  Pose2D odom_pose;
  double max_tilt_deg = 0.0;
};

enum class RecoveryPhase
{
  IDLE,
  EMERGENCY_BACKUP,
  STUCK_BACKUP,
  RECOVERY_TURN
};

class TurtlebotAutonomousGridDrive : public rclcpp::Node
{
public:
  TurtlebotAutonomousGridDrive()
      : Node("turtlebot_autonomous_grid_drive"),
        scan_topic_(this->declare_parameter<std::string>("scan_topic", "/scan")),
        imu_topic_(this->declare_parameter<std::string>("imu_topic", "/imu")),
        odom_topic_(this->declare_parameter<std::string>("odom_topic", "/odom")),
        cmd_vel_topic_(this->declare_parameter<std::string>("cmd_vel_topic", "/cmd_vel")),
        cmd_vel_stamped_(this->declare_parameter<bool>("cmd_vel_stamped", false)),
        local_grid_topic_(this->declare_parameter<std::string>("local_grid_topic", "/grid_drive/local_grid")),
        vector_marker_topic_(this->declare_parameter<std::string>("vector_marker_topic", "/grid_drive/vector_markers")),
        grid_frame_id_(this->declare_parameter<std::string>("grid_frame_id", "base_link")),
        auto_start_(this->declare_parameter<bool>("auto_start", false)),
        reset_heading_on_start_(this->declare_parameter<bool>("reset_heading_on_start", true)),
        require_odom_(this->declare_parameter<bool>("require_odom", true)),
        control_period_ms_(this->declare_parameter<int>("control_period_ms", 100)),
        report_period_ms_(this->declare_parameter<int>("report_period_ms", 1000)),
        max_linear_speed_(this->declare_parameter<double>("max_linear_speed", 0.15)),
        min_linear_speed_(this->declare_parameter<double>("min_linear_speed", 0.02)),
        max_angular_speed_(this->declare_parameter<double>("max_angular_speed", 0.85)),
        steering_kp_(this->declare_parameter<double>("steering_kp", 2.0)),
        recovery_turn_speed_(this->declare_parameter<double>("recovery_turn_speed", 0.75)),
        prefer_left_recovery_(this->declare_parameter<bool>("prefer_left_recovery", true)),
        emergency_stop_distance_m_(this->declare_parameter<double>("emergency_stop_distance_m", 0.125)),
        front_stop_distance_m_(this->declare_parameter<double>("front_stop_distance_m", 0.16)),
        front_slow_distance_m_(this->declare_parameter<double>("front_slow_distance_m", 0.70)),
        front_window_deg_(this->declare_parameter<double>("front_window_deg", 28.0)),
        scan_sample_step_deg_(this->declare_parameter<double>("scan_sample_step_deg", 1.0)),
        scan_obstacle_min_range_m_(this->declare_parameter<double>("scan_obstacle_min_range_m", 0.01)),
        scan_obstacle_max_range_m_(this->declare_parameter<double>("scan_obstacle_max_range_m", 2.0)),
        lost_scan_timeout_seconds_(this->declare_parameter<double>("lost_scan_timeout_seconds", 2.0)),
        grid_radius_m_(this->declare_parameter<double>("grid_radius_m", 2.0)),
        grid_resolution_m_(this->declare_parameter<double>("grid_resolution_m", 0.02)),
        grid_memory_seconds_(this->declare_parameter<double>("grid_memory_seconds", 1.2)),
        grid_memory_max_points_(this->declare_parameter<int>("grid_memory_max_points", 30000)),
        obstacle_thickness_m_(this->declare_parameter<double>("obstacle_thickness_m", 0.02)),
        robot_radius_m_(this->declare_parameter<double>("robot_radius_m", 0.105)),
        clearance_margin_m_(this->declare_parameter<double>("clearance_margin_m", 0.02)),
        direction_sample_step_deg_(this->declare_parameter<double>("direction_sample_step_deg", 2.0)),
        corridor_sample_step_m_(this->declare_parameter<double>("corridor_sample_step_m", 0.04)),
        direction_score_tolerance_ratio_(this->declare_parameter<double>("direction_score_tolerance_ratio", 0.10)),
        direction_score_tolerance_abs_(this->declare_parameter<double>("direction_score_tolerance_abs", 0.20)),
        unknown_direction_penalty_(this->declare_parameter<double>("unknown_direction_penalty", 1.0)),
        blocked_direction_penalty_(this->declare_parameter<double>("blocked_direction_penalty", 2.0)),
        min_vector_norm_(this->declare_parameter<double>("min_vector_norm", 0.05)),
        repulsion_range_m_(this->declare_parameter<double>("repulsion_range_m", 0.55)),
        repulsion_weight_(this->declare_parameter<double>("repulsion_weight", 1.0)),
        lowest_cost_weight_(this->declare_parameter<double>("lowest_cost_weight", 1.0)),
        heading_forward_weight_initial_(this->declare_parameter<double>("heading_forward_weight_initial", 1.0)),
        heading_decay_distance_m_(this->declare_parameter<double>("heading_decay_distance_m", 4.0)),
        reverse_heading_threshold_deg_(this->declare_parameter<double>("reverse_heading_threshold_deg", 150.0)),
        reverse_bias_start_m_(this->declare_parameter<double>("reverse_bias_start_m", 0.20)),
        reverse_bias_full_m_(this->declare_parameter<double>("reverse_bias_full_m", 1.00)),
        reverse_bias_max_weight_(this->declare_parameter<double>("reverse_bias_max_weight", 1.2)),
        min_passable_gap_m_(this->declare_parameter<double>("min_passable_gap_m", 0.20)),
        gap_search_deg_(this->declare_parameter<double>("gap_search_deg", 100.0)),
        gap_sector_half_width_deg_(this->declare_parameter<double>("gap_sector_half_width_deg", 10.0)),
        gap_boundary_search_deg_(this->declare_parameter<double>("gap_boundary_search_deg", 45.0)),
        gap_boundary_obstacle_max_range_m_(this->declare_parameter<double>("gap_boundary_obstacle_max_range_m", 0.80)),
        gap_boundary_drop_m_(this->declare_parameter<double>("gap_boundary_drop_m", 0.08)),
        gap_min_center_distance_m_(this->declare_parameter<double>("gap_min_center_distance_m", 0.30)),
        gap_min_depth_gain_m_(this->declare_parameter<double>("gap_min_depth_gain_m", 0.25)),
        tilt_stop_deg_(this->declare_parameter<double>("tilt_stop_deg", 60.0)),
        imu_timeout_seconds_(this->declare_parameter<double>("imu_timeout_seconds", 1.0)),
        odom_timeout_seconds_(this->declare_parameter<double>("odom_timeout_seconds", 1.0)),
        stuck_recovery_enabled_(this->declare_parameter<bool>("stuck_recovery_enabled", true)),
        stuck_command_linear_threshold_(this->declare_parameter<double>("stuck_command_linear_threshold", 0.03)),
        stuck_min_progress_m_(this->declare_parameter<double>("stuck_min_progress_m", 0.03)),
        stuck_hold_seconds_(this->declare_parameter<double>("stuck_hold_seconds", 4.0)),
        stuck_backup_speed_(this->declare_parameter<double>("stuck_backup_speed", 0.04)),
        stuck_backup_seconds_(this->declare_parameter<double>("stuck_backup_seconds", 1.2)),
        stuck_turn_speed_(this->declare_parameter<double>("stuck_turn_speed", 0.60)),
        stuck_turn_seconds_(this->declare_parameter<double>("stuck_turn_seconds", 1.0)),
        stuck_cooldown_seconds_(this->declare_parameter<double>("stuck_cooldown_seconds", 2.0)),
        emergency_backup_speed_(this->declare_parameter<double>("emergency_backup_speed", 0.04)),
        emergency_backup_seconds_(this->declare_parameter<double>("emergency_backup_seconds", 0.8)),
        pivot_hold_seconds_(this->declare_parameter<double>("pivot_hold_seconds", 0.6)),
        dead_end_memory_enabled_default_(this->declare_parameter<bool>("dead_end_memory_enabled", false)),
        dead_end_memory_seconds_(this->declare_parameter<double>("dead_end_memory_seconds", 45.0)),
        dead_end_memory_max_entries_(this->declare_parameter<int>("dead_end_memory_max_entries", 20)),
        dead_end_memory_cell_cost_(this->declare_parameter<double>("dead_end_memory_cell_cost", 1.5)),
        dead_end_record_cooldown_seconds_(this->declare_parameter<double>("dead_end_record_cooldown_seconds", 3.0)),
        dead_end_merge_distance_m_(this->declare_parameter<double>("dead_end_merge_distance_m", 0.35)),
        dead_end_merge_angle_deg_(this->declare_parameter<double>("dead_end_merge_angle_deg", 35.0)),
        dead_end_backtrack_distance_m_(this->declare_parameter<double>("dead_end_backtrack_distance_m", 1.20)),
        dead_end_forward_extension_m_(this->declare_parameter<double>("dead_end_forward_extension_m", 0.20)),
        dead_end_corridor_half_width_m_(this->declare_parameter<double>("dead_end_corridor_half_width_m", 0.28))
  {
    enabled_requested_ = auto_start_;
    dead_end_memory_enabled_runtime_ = dead_end_memory_enabled_default_;
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
    local_grid_publisher_ =
        this->create_publisher<nav_msgs::msg::OccupancyGrid>(local_grid_topic_, 10);
    vector_marker_publisher_ =
        this->create_publisher<visualization_msgs::msg::MarkerArray>(vector_marker_topic_, 10);

    scan_subscription_ =
        this->create_subscription<sensor_msgs::msg::LaserScan>(
            scan_topic_, rclcpp::SensorDataQoS(),
            std::bind(&TurtlebotAutonomousGridDrive::scan_callback, this, _1));
    imu_subscription_ =
        this->create_subscription<sensor_msgs::msg::Imu>(
            imu_topic_, rclcpp::SensorDataQoS(),
            std::bind(&TurtlebotAutonomousGridDrive::imu_callback, this, _1));
    odom_subscription_ =
        this->create_subscription<nav_msgs::msg::Odometry>(
            odom_topic_, 10,
            std::bind(&TurtlebotAutonomousGridDrive::odom_callback, this, _1));
    set_enabled_service_ =
        this->create_service<std_srvs::srv::SetBool>(
            "~/set_enabled",
            std::bind(&TurtlebotAutonomousGridDrive::set_enabled_callback, this, _1, _2));
    set_dead_end_memory_service_ =
        this->create_service<std_srvs::srv::SetBool>(
            "~/set_dead_end_memory",
            std::bind(&TurtlebotAutonomousGridDrive::set_dead_end_memory_callback, this, _1, _2));

    control_timer_ =
        this->create_wall_timer(
            std::chrono::milliseconds(std::max(20, control_period_ms_)),
            std::bind(&TurtlebotAutonomousGridDrive::control_timer_callback, this));
    report_timer_ =
        this->create_wall_timer(
            std::chrono::milliseconds(std::max(100, report_period_ms_)),
            std::bind(&TurtlebotAutonomousGridDrive::report_timer_callback, this));

    RCLCPP_INFO(
        this->get_logger(),
        "Started grid drive: scan=%s odom=%s cmd=%s grid_radius=%.2f resolution=%.3f auto_start=%s dead_end_memory=%s",
        scan_topic_.c_str(),
        odom_topic_.c_str(),
        cmd_vel_topic_.c_str(),
        grid_radius_m_,
        grid_resolution_m_,
        auto_start_ ? "true" : "false",
        dead_end_memory_enabled_runtime_ ? "true" : "false");
  }

private:
  void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
  {
    auto sanitized = std::make_shared<sensor_msgs::msg::LaserScan>(*msg);
    for (auto &range : sanitized->ranges)
    {
      if (!std::isfinite(range) || range <= 0.0)
      {
        range = std::numeric_limits<float>::quiet_NaN();
      }
    }

    std::lock_guard<std::mutex> lock(data_mutex_);
    latest_scan_ = sanitized;
    latest_scan_time_ = this->now();
  }

  void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
  {
    const double x = msg->orientation.x;
    const double y = msg->orientation.y;
    const double z = msg->orientation.z;
    const double w = msg->orientation.w;

    const double sinr_cosp = 2.0 * (w * x + y * z);
    const double cosr_cosp = 1.0 - 2.0 * (x * x + y * y);
    const double roll = std::atan2(sinr_cosp, cosr_cosp);
    const double sinp = 2.0 * (w * y - z * x);
    const double pitch = std::fabs(sinp) >= 1.0
                             ? std::copysign(kPi / 2.0, sinp)
                             : std::asin(sinp);

    std::lock_guard<std::mutex> lock(data_mutex_);
    latest_imu_time_ = this->now();
    imu_ready_ = true;
    max_tilt_deg_ = std::max(std::fabs(rad_to_deg(roll)), std::fabs(rad_to_deg(pitch)));
  }

  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    Pose2D pose;
    pose.x = msg->pose.pose.position.x;
    pose.y = msg->pose.pose.position.y;
    pose.yaw = yaw_from_quaternion(
        msg->pose.pose.orientation.x,
        msg->pose.pose.orientation.y,
        msg->pose.pose.orientation.z,
        msg->pose.pose.orientation.w);

    std::lock_guard<std::mutex> lock(data_mutex_);
    if (odom_ready_ && enabled_requested_)
    {
      const double dx = pose.x - odom_pose_.x;
      const double dy = pose.y - odom_pose_.y;
      const double ds = std::sqrt(dx * dx + dy * dy);
      if (ds < 0.25)
      {
        path_distance_m_ += ds;
      }
    }
    odom_pose_ = pose;
    odom_ready_ = true;
    latest_odom_time_ = this->now();
  }

  void set_enabled_callback(
      const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
      std::shared_ptr<std_srvs::srv::SetBool::Response> response)
  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    enabled_requested_ = request->data;
    if (enabled_requested_)
    {
      if (reset_heading_on_start_ && odom_ready_)
      {
        heading_reference_yaw_ = odom_pose_.yaw;
        heading_reference_valid_ = true;
      }
      path_distance_m_ = 0.0;
      reverse_bias_active_ = false;
      recovery_phase_ = RecoveryPhase::IDLE;
      stuck_watch_active_ = false;
      response->success = true;
      response->message = "Grid drive enabled. Heading reference reset.";
    }
    else
    {
      publish_velocity(0.0, 0.0);
      memory_points_.clear();
      recovery_phase_ = RecoveryPhase::IDLE;
      reverse_bias_active_ = false;
      stuck_watch_active_ = false;
      response->success = true;
      response->message = "Grid drive stopped. Publishing zero velocity.";
    }
  }

  void set_dead_end_memory_callback(
      const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
      std::shared_ptr<std_srvs::srv::SetBool::Response> response)
  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    dead_end_memory_enabled_runtime_ = request->data;
    if (!dead_end_memory_enabled_runtime_)
    {
      dead_end_memories_.clear();
      response->success = true;
      response->message = "Dead-end memory disabled and cleared.";
      return;
    }

    response->success = true;
    response->message = "Dead-end memory enabled.";
  }

  void control_timer_callback()
  {
    if (!g_keep_running)
    {
      rclcpp::shutdown();
      return;
    }

    const rclcpp::Time now = this->now();
    SafetySnapshot safety;
    sensor_msgs::msg::LaserScan::SharedPtr scan;
    rclcpp::Time scan_time;
    {
      std::lock_guard<std::mutex> lock(data_mutex_);
      safety.enabled_requested = enabled_requested_;
      safety.imu_ready = imu_ready_;
      safety.odom_ready = odom_ready_;
      safety.heading_reference_valid = heading_reference_valid_;
      safety.latest_imu_time = latest_imu_time_;
      safety.latest_odom_time = latest_odom_time_;
      safety.odom_pose = odom_pose_;
      safety.max_tilt_deg = max_tilt_deg_;
      scan = latest_scan_;
      scan_time = latest_scan_time_;
    }

    DriveOutput output;
    output.enabled_requested = safety.enabled_requested;
    output.imu_ready = safety.imu_ready;
    output.odom_ready = safety.odom_ready;
    output.heading_reference_valid = safety.heading_reference_valid;
    output.tilt_deg = safety.max_tilt_deg;
    output.path_distance_m = path_distance_snapshot();
    if (safety.heading_reference_valid)
    {
      output.heading_error_deg =
          rad_to_deg(normalize_angle_rad(safety.odom_pose.yaw - heading_reference_yaw_snapshot()));
    }

    if (!safety.enabled_requested)
    {
      output.mode = "STOPPED";
      output.detail = "waiting for set_enabled true";
      publish_velocity(0.0, 0.0);
      store_output(output);
      return;
    }
    if (!sensors_are_ready(safety, now, output.detail))
    {
      output.mode = "WAIT_SENSORS";
      publish_velocity(0.0, 0.0);
      store_output(output);
      return;
    }
    if (!scan || (now - scan_time).seconds() > lost_scan_timeout_seconds_)
    {
      output.mode = scan ? "SCAN_TIMEOUT" : "WAIT_SCAN";
      output.detail = "lidar scan missing or stale";
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

    if (reset_heading_on_start_ && !safety.heading_reference_valid)
    {
      std::lock_guard<std::mutex> lock(data_mutex_);
      heading_reference_yaw_ = safety.odom_pose.yaw;
      heading_reference_valid_ = true;
    }

    integrate_scan_if_new(*scan, scan_time, safety.odom_pose);
    prune_memory(now);
    prune_dead_end_memory(now);
    LocalGrid grid = build_local_grid(*scan, safety.odom_pose);
    publish_local_grid(grid);

    output = compute_grid_drive_output(grid, *scan, safety);
    output.enabled_requested = safety.enabled_requested;
    output.imu_ready = safety.imu_ready;
    output.odom_ready = safety.odom_ready;
    output.heading_reference_valid = heading_reference_valid_snapshot();
    output.tilt_deg = safety.max_tilt_deg;
    output.path_distance_m = path_distance_snapshot();
    output.heading_error_deg =
        rad_to_deg(normalize_angle_rad(safety.odom_pose.yaw - heading_reference_yaw_snapshot()));

    apply_recovery_overrides(output, grid, now, safety.odom_pose);
    publish_vector_markers(output);
    publish_velocity(output.linear_x, output.angular_z);
    store_output(output);
  }

  bool sensors_are_ready(const SafetySnapshot &safety, const rclcpp::Time &now, std::string &reason) const
  {
    if (!safety.imu_ready)
    {
      reason = "waiting for imu";
      return false;
    }
    if ((now - safety.latest_imu_time).seconds() > imu_timeout_seconds_)
    {
      reason = "imu timeout";
      return false;
    }
    if (require_odom_ && !safety.odom_ready)
    {
      reason = "waiting for odom";
      return false;
    }
    if (require_odom_ && (now - safety.latest_odom_time).seconds() > odom_timeout_seconds_)
    {
      reason = "odom timeout";
      return false;
    }
    return true;
  }

  DriveOutput compute_grid_drive_output(
      const LocalGrid &grid,
      const sensor_msgs::msg::LaserScan &scan,
      const SafetySnapshot &safety)
  {
    DriveOutput output;
    output.vectors = compute_vectors(grid, scan, safety);
    const double final_norm = vector_norm(output.vectors.final_vector);
    const double target_angle = std::atan2(output.vectors.final_vector.y, output.vectors.final_vector.x);
    output.vectors.front_m = min_range_in_sector(scan, 0.0, front_window_deg_);

    if (!output.vectors.has_drive_direction || final_norm < min_vector_norm_)
    {
      output.linear_x = 0.0;
      output.angular_z = choose_pivot_turn(grid, recovery_turn_speed_);
      output.mode = "GRID_PIVOT";
      output.detail = "no stable drive vector";
      return output;
    }

    const double front_ratio = clamp_value(
        (output.vectors.front_m - front_stop_distance_m_) /
            std::max(0.01, front_slow_distance_m_ - front_stop_distance_m_),
        0.0,
        1.0);
    const double angle_ratio = clamp_value(std::fabs(target_angle) / (0.5 * kPi), 0.0, 1.0);
    const double angle_speed_scale = clamp_value(1.0 - angle_ratio, 0.25, 1.0);
    double linear = min_linear_speed_ +
                    (max_linear_speed_ - min_linear_speed_) * front_ratio * angle_speed_scale;
    if (output.vectors.front_m <= front_stop_distance_m_)
    {
      linear = 0.0;
    }

    output.linear_x = clamp_value(linear, 0.0, max_linear_speed_);
    output.angular_z = clamp_value(steering_kp_ * target_angle, -max_angular_speed_, max_angular_speed_);
    output.mode = output.linear_x <= min_linear_speed_ + 1e-6 ? "GRID_SLOW" : "GRID_DRIVE";
    output.detail = "vector field steering";
    return output;
  }

  VectorSummary compute_vectors(
      const LocalGrid &grid,
      const sensor_msgs::msg::LaserScan &scan,
      const SafetySnapshot &safety)
  {
    (void)scan;
    VectorSummary summary;
    summary.dead_end_memory_enabled = dead_end_memory_enabled_snapshot();
    summary.dead_end_memory_count = dead_end_memory_count_snapshot();
    std::vector<DirectionCandidate> candidates;
    const double step_deg = std::max(1.0, direction_sample_step_deg_);
    double best_score = std::numeric_limits<double>::infinity();

    for (double angle_deg = -180.0; angle_deg < 180.0; angle_deg += step_deg)
    {
      DirectionCandidate candidate = score_direction(grid, deg_to_rad(angle_deg));
      candidates.push_back(candidate);
      ++summary.sampled_directions;
      if (!candidate.driveable)
      {
        ++summary.blocked_directions;
        continue;
      }
      if (candidate.score < best_score)
      {
        best_score = candidate.score;
      }
    }

    if (!std::isfinite(best_score))
    {
      summary.has_drive_direction = false;
      summary.repulsion = compute_repulsion_vector(grid);
      summary.final_vector = summary.repulsion;
      return summary;
    }

    Vector2D low_sum;
    double weight_sum = 0.0;
    const double tolerance =
        direction_score_tolerance_abs_ + std::fabs(best_score) * direction_score_tolerance_ratio_;
    for (const auto &candidate : candidates)
    {
      if (!candidate.driveable || candidate.score > best_score + tolerance)
      {
        continue;
      }
      const double weight = 1.0 / (1.0 + std::max(0.0, candidate.score));
      low_sum.x += weight * std::cos(candidate.angle_rad);
      low_sum.y += weight * std::sin(candidate.angle_rad);
      weight_sum += weight;
    }
    if (weight_sum > 1e-9)
    {
      low_sum.x /= weight_sum;
      low_sum.y /= weight_sum;
    }
    summary.lowest_cost = limit_vector(normalize_vector(low_sum), lowest_cost_weight_);
    summary.repulsion = compute_repulsion_vector(grid);
    summary.heading_bias = compute_heading_bias(safety);

    Vector2D pre_reverse = add_vectors(add_vectors(summary.lowest_cost, summary.repulsion), summary.heading_bias);
    summary.reverse_bias = compute_reverse_bias(pre_reverse, safety);
    summary.final_vector = add_vectors(pre_reverse, summary.reverse_bias);
    summary.best_score = best_score;
    summary.best_angle_rad = std::atan2(summary.lowest_cost.y, summary.lowest_cost.x);
    summary.has_drive_direction = true;
    summary.reverse_bias_active = reverse_bias_active_;
    return summary;
  }

  DirectionCandidate score_direction(const LocalGrid &grid, double angle_rad) const
  {
    DirectionCandidate candidate;
    candidate.angle_rad = angle_rad;
    candidate.score = 0.0;
    candidate.driveable = true;
    const double corridor_half_width = robot_radius_m_ + clearance_margin_m_;
    const double distance_step = std::max(grid.resolution, corridor_sample_step_m_);
    const double lateral_step = std::max(grid.resolution, corridor_sample_step_m_);
    int samples = 0;

    for (double distance = robot_radius_m_; distance <= grid.radius; distance += distance_step)
    {
      for (double lateral = -corridor_half_width; lateral <= corridor_half_width + 1e-6; lateral += lateral_step)
      {
        const double x = distance * std::cos(angle_rad) - lateral * std::sin(angle_rad);
        const double y = distance * std::sin(angle_rad) + lateral * std::cos(angle_rad);
        double state = kUnknown;
        if (!state_at(grid, x, y, state))
        {
          state = kUnknown;
        }
        const double cell_value = cell_score_value(state);
        candidate.score += cell_value * distance;
        if (state >= kBlocked)
        {
          candidate.min_blocked_distance = std::min(candidate.min_blocked_distance, distance);
        }
        ++samples;
      }
    }

    if (samples == 0)
    {
      candidate.driveable = false;
      return candidate;
    }
    candidate.score /= static_cast<double>(samples);
    if (candidate.min_blocked_distance <= front_stop_distance_m_)
    {
      candidate.driveable = false;
    }
    return candidate;
  }

  double cell_score_value(double state) const
  {
    if (state <= kFree)
    {
      return 0.0;
    }
    if (state >= kBlocked)
    {
      return blocked_direction_penalty_;
    }
    if (state > kUnknown)
    {
      return state;
    }
    return unknown_direction_penalty_;
  }

  Vector2D compute_repulsion_vector(const LocalGrid &grid) const
  {
    Vector2D repulsion;
    for (int gy = 0; gy < grid.height; ++gy)
    {
      for (int gx = 0; gx < grid.width; ++gx)
      {
        const int index = gy * grid.width + gx;
        if (grid.cells[static_cast<std::size_t>(index)] < kBlocked)
        {
          continue;
        }
        const double x = grid.origin_x + (static_cast<double>(gx) + 0.5) * grid.resolution;
        const double y = grid.origin_y + (static_cast<double>(gy) + 0.5) * grid.resolution;
        const double dist = std::sqrt(x * x + y * y);
        if (dist <= 1e-6 || dist > repulsion_range_m_)
        {
          continue;
        }
        const double gain = (repulsion_range_m_ - dist) / repulsion_range_m_;
        repulsion.x += gain * (-x / dist);
        repulsion.y += gain * (-y / dist);
      }
    }
    return limit_vector(repulsion, repulsion_weight_);
  }

  Vector2D compute_heading_bias(const SafetySnapshot &safety) const
  {
    if (!safety.heading_reference_valid)
    {
      return Vector2D();
    }
    const double distance = path_distance_snapshot();
    const double decay = clamp_value(
        1.0 - distance / std::max(0.01, heading_decay_distance_m_),
        0.0,
        1.0);
    const double weight = heading_forward_weight_initial_ * decay;
    const double angle = normalize_angle_rad(heading_reference_yaw_snapshot() - safety.odom_pose.yaw);
    Vector2D bias;
    bias.x = weight * std::cos(angle);
    bias.y = weight * std::sin(angle);
    return bias;
  }

  Vector2D compute_reverse_bias(const Vector2D &pre_reverse_vector, const SafetySnapshot &safety)
  {
    if (!safety.heading_reference_valid)
    {
      reverse_bias_active_ = false;
      return Vector2D();
    }
    const double angle = std::atan2(pre_reverse_vector.y, pre_reverse_vector.x);
    const double heading_angle =
        normalize_angle_rad(heading_reference_yaw_snapshot() - safety.odom_pose.yaw);
    const double heading_error = normalize_angle_rad(angle - heading_angle);
    const bool wants_reverse = std::fabs(heading_error) >= deg_to_rad(reverse_heading_threshold_deg_);
    const double path_distance = path_distance_snapshot();

    if (!wants_reverse)
    {
      reverse_bias_active_ = false;
      return Vector2D();
    }
    if (!reverse_bias_active_)
    {
      reverse_bias_active_ = true;
      reverse_bias_start_distance_m_ = path_distance;
    }

    const double reverse_distance = std::max(0.0, path_distance - reverse_bias_start_distance_m_);
    const double ratio = clamp_value(
        (reverse_distance - reverse_bias_start_m_) /
            std::max(0.01, reverse_bias_full_m_ - reverse_bias_start_m_),
        0.0,
        1.0);
    const double weight = reverse_bias_max_weight_ * ratio;
    Vector2D bias;
    bias.x = weight * std::cos(heading_angle);
    bias.y = weight * std::sin(heading_angle);
    return bias;
  }

  LocalGrid build_local_grid(
      const sensor_msgs::msg::LaserScan &scan,
      const Pose2D &odom_pose)
  {
    LocalGrid grid;
    grid.radius = std::max(0.2, grid_radius_m_);
    grid.resolution = std::max(0.005, grid_resolution_m_);
    grid.origin_x = -grid.radius;
    grid.origin_y = -grid.radius;
    grid.width = static_cast<int>(std::ceil((2.0 * grid.radius) / grid.resolution));
    grid.height = grid.width;
    grid.cells.assign(static_cast<std::size_t>(grid.width * grid.height), kUnknown);

    replay_memory(grid, odom_pose);
    apply_current_scan(grid, scan);
    apply_small_gap_blocks(grid, scan);
    apply_dead_end_memory(grid, odom_pose);
    return grid;
  }

  void replay_memory(LocalGrid &grid, const Pose2D &odom_pose)
  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    const double cos_yaw = std::cos(odom_pose.yaw);
    const double sin_yaw = std::sin(odom_pose.yaw);
    for (const auto &point : memory_points_)
    {
      const double dx = point.x - odom_pose.x;
      const double dy = point.y - odom_pose.y;
      const double local_x = cos_yaw * dx + sin_yaw * dy;
      const double local_y = -sin_yaw * dx + cos_yaw * dy;
      if (local_x * local_x + local_y * local_y <= grid.radius * grid.radius)
      {
        mark_disk(grid, local_x, local_y, obstacle_thickness_m_, kBlocked);
      }
    }
  }

  void apply_current_scan(LocalGrid &grid, const sensor_msgs::msg::LaserScan &scan)
  {
    const int stride = scan_stride(scan);
    const double max_hit_range = std::min(grid.radius, scan_obstacle_max_range_m_);
    for (int i = 0; i < static_cast<int>(scan.ranges.size()); i += stride)
    {
      const double angle = scan.angle_min + static_cast<double>(i) * scan.angle_increment;
      const double range = scan.ranges[static_cast<std::size_t>(i)];
      if (!range_is_usable(scan, range))
      {
        continue;
      }
      const bool hit_inside_grid = range <= max_hit_range;
      const double clear_range = hit_inside_grid
                                     ? std::max(0.0, range - obstacle_thickness_m_)
                                     : max_hit_range;
      raytrace_free(grid, angle, clear_range);
      if (hit_inside_grid)
      {
        mark_disk(
            grid,
            range * std::cos(angle),
            range * std::sin(angle),
            obstacle_thickness_m_,
            kBlocked);
      }
    }
  }

  void apply_small_gap_blocks(LocalGrid &grid, const sensor_msgs::msg::LaserScan &scan)
  {
    const std::vector<GapSector> sectors = compute_small_gap_sectors(scan);
    const double angle_step = deg_to_rad(std::max(1.0, scan_sample_step_deg_));
    for (const auto &sector : sectors)
    {
      for (double angle = sector.angle_rad - sector.half_width_rad;
           angle <= sector.angle_rad + sector.half_width_rad + 1e-6;
           angle += angle_step)
      {
        const double start = std::max(0.0, sector.center_range_m - 0.05);
        for (double distance = start; distance <= grid.radius; distance += grid.resolution)
        {
          set_state(grid, distance * std::cos(angle), distance * std::sin(angle), kBlocked);
        }
      }
    }
  }

  void raytrace_free(LocalGrid &grid, double angle, double range)
  {
    const double step = std::max(0.005, grid.resolution);
    for (double distance = 0.0; distance <= range; distance += step)
    {
      set_state(grid, distance * std::cos(angle), distance * std::sin(angle), kFree);
    }
  }

  void mark_disk(LocalGrid &grid, double x, double y, double radius, double state)
  {
    const double r = std::max(grid.resolution, radius);
    for (double dy = -r; dy <= r + 1e-6; dy += grid.resolution)
    {
      for (double dx = -r; dx <= r + 1e-6; dx += grid.resolution)
      {
        if (dx * dx + dy * dy <= r * r)
        {
          set_state(grid, x + dx, y + dy, state);
        }
      }
    }
  }

  bool set_state(LocalGrid &grid, double x, double y, double state)
  {
    if (x * x + y * y > grid.radius * grid.radius)
    {
      return false;
    }
    int gx = 0;
    int gy = 0;
    if (!world_to_grid(grid, x, y, gx, gy))
    {
      return false;
    }
    grid.cells[static_cast<std::size_t>(gy * grid.width + gx)] = state;
    return true;
  }

  bool set_state_max(LocalGrid &grid, double x, double y, double state)
  {
    if (x * x + y * y > grid.radius * grid.radius)
    {
      return false;
    }
    int gx = 0;
    int gy = 0;
    if (!world_to_grid(grid, x, y, gx, gy))
    {
      return false;
    }
    const std::size_t index = static_cast<std::size_t>(gy * grid.width + gx);
    grid.cells[index] = std::max(grid.cells[index], state);
    return true;
  }

  bool state_at(const LocalGrid &grid, double x, double y, double &state) const
  {
    if (x * x + y * y > grid.radius * grid.radius)
    {
      return false;
    }
    int gx = 0;
    int gy = 0;
    if (!world_to_grid(grid, x, y, gx, gy))
    {
      return false;
    }
    state = grid.cells[static_cast<std::size_t>(gy * grid.width + gx)];
    return true;
  }

  void mark_disk_max(LocalGrid &grid, double x, double y, double radius, double state)
  {
    const double r = std::max(grid.resolution, radius);
    for (double dy = -r; dy <= r + 1e-6; dy += grid.resolution)
    {
      for (double dx = -r; dx <= r + 1e-6; dx += grid.resolution)
      {
        if (dx * dx + dy * dy <= r * r)
        {
          set_state_max(grid, x + dx, y + dy, state);
        }
      }
    }
  }

  void apply_dead_end_memory(LocalGrid &grid, const Pose2D &odom_pose)
  {
    std::vector<DeadEndMemory> memories;
    bool enabled = false;
    {
      std::lock_guard<std::mutex> lock(data_mutex_);
      enabled = dead_end_memory_enabled_runtime_;
      memories.assign(dead_end_memories_.begin(), dead_end_memories_.end());
    }
    if (!enabled || memories.empty())
    {
      return;
    }

    const double cost = clamp_value(dead_end_memory_cell_cost_, kUnknown, kBlocked);
    const double step = std::max(grid.resolution, corridor_sample_step_m_);
    const double cos_robot = std::cos(odom_pose.yaw);
    const double sin_robot = std::sin(odom_pose.yaw);
    const double half_width = std::max(grid.resolution, dead_end_corridor_half_width_m_);

    for (const auto &memory : memories)
    {
      const double cos_bad = std::cos(memory.yaw);
      const double sin_bad = std::sin(memory.yaw);
      for (double distance = -std::max(0.0, dead_end_backtrack_distance_m_);
           distance <= std::max(0.0, dead_end_forward_extension_m_) + 1e-6;
           distance += step)
      {
        const double wx = memory.x + distance * cos_bad;
        const double wy = memory.y + distance * sin_bad;
        const double dx = wx - odom_pose.x;
        const double dy = wy - odom_pose.y;
        const double local_x = cos_robot * dx + sin_robot * dy;
        const double local_y = -sin_robot * dx + cos_robot * dy;
        mark_disk_max(grid, local_x, local_y, half_width, cost);
      }
    }
  }

  bool world_to_grid(const LocalGrid &grid, double x, double y, int &gx, int &gy) const
  {
    gx = static_cast<int>(std::floor((x - grid.origin_x) / grid.resolution));
    gy = static_cast<int>(std::floor((y - grid.origin_y) / grid.resolution));
    return gx >= 0 && gy >= 0 && gx < grid.width && gy < grid.height;
  }

  void integrate_scan_if_new(
      const sensor_msgs::msg::LaserScan &scan,
      const rclcpp::Time &scan_time,
      const Pose2D &odom_pose)
  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    if (last_integrated_scan_time_.nanoseconds() == scan_time.nanoseconds())
    {
      return;
    }
    last_integrated_scan_time_ = scan_time;

    const int stride = scan_stride(scan);
    const double max_range = std::min(grid_radius_m_, scan_obstacle_max_range_m_);
    const double cos_yaw = std::cos(odom_pose.yaw);
    const double sin_yaw = std::sin(odom_pose.yaw);
    for (int i = 0; i < static_cast<int>(scan.ranges.size()); i += stride)
    {
      const double range = scan.ranges[static_cast<std::size_t>(i)];
      if (!range_is_usable(scan, range) || range > max_range)
      {
        continue;
      }
      const double angle = scan.angle_min + static_cast<double>(i) * scan.angle_increment;
      const double lx = range * std::cos(angle);
      const double ly = range * std::sin(angle);
      MemoryPoint point;
      point.x = odom_pose.x + cos_yaw * lx - sin_yaw * ly;
      point.y = odom_pose.y + sin_yaw * lx + cos_yaw * ly;
      point.stamp = scan_time;
      memory_points_.push_back(point);
    }
    while (static_cast<int>(memory_points_.size()) > std::max(1, grid_memory_max_points_))
    {
      memory_points_.pop_front();
    }
  }

  void prune_memory(const rclcpp::Time &now)
  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    while (!memory_points_.empty() &&
           (now - memory_points_.front().stamp).seconds() > grid_memory_seconds_)
    {
      memory_points_.pop_front();
    }
  }

  void prune_dead_end_memory(const rclcpp::Time &now)
  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    if (!dead_end_memory_enabled_runtime_)
    {
      dead_end_memories_.clear();
      return;
    }
    dead_end_memories_.erase(
        std::remove_if(
            dead_end_memories_.begin(),
            dead_end_memories_.end(),
            [&](const DeadEndMemory &memory) {
              return (now - memory.stamp).seconds() > dead_end_memory_seconds_;
            }),
        dead_end_memories_.end());
    while (static_cast<int>(dead_end_memories_.size()) > std::max(1, dead_end_memory_max_entries_))
    {
      dead_end_memories_.pop_front();
    }
  }

  std::vector<GapSector> compute_small_gap_sectors(const sensor_msgs::msg::LaserScan &scan) const
  {
    std::vector<GapSector> sectors;
    if (scan.ranges.empty() || std::fabs(scan.angle_increment) < 1e-9)
    {
      return sectors;
    }

    const double search_limit = std::max(1.0, gap_search_deg_);
    const double step_deg = std::max(1.0, scan_sample_step_deg_);
    bool run_active = false;
    double run_start_deg = 0.0;
    double run_end_deg = 0.0;
    double run_center_range = 0.0;
    double run_min_width = std::numeric_limits<double>::infinity();

    const auto flush_run = [&]() {
      if (!run_active)
      {
        return;
      }
      GapSector sector;
      sector.angle_rad = deg_to_rad(0.5 * (run_start_deg + run_end_deg));
      sector.half_width_rad =
          deg_to_rad(std::max(step_deg, 0.5 * (run_end_deg - run_start_deg) + step_deg));
      sector.center_range_m = run_center_range;
      sector.width_m = std::isfinite(run_min_width) ? run_min_width : 0.0;
      sectors.push_back(sector);
      run_active = false;
      run_center_range = 0.0;
      run_min_width = std::numeric_limits<double>::infinity();
    };

    for (double angle_deg = -search_limit; angle_deg <= search_limit; angle_deg += step_deg)
    {
      double center_range = 0.0;
      if (!range_at_angle(scan, deg_to_rad(angle_deg), center_range) ||
          !range_is_usable(scan, center_range))
      {
        flush_run();
        continue;
      }
      const double sector_clearance =
          min_range_in_sector(scan, angle_deg, gap_sector_half_width_deg_);
      const GapWidthMeasurement measurement =
          measure_gap_width(scan, angle_deg, center_range);
      if (!gap_is_too_small(angle_deg, center_range, sector_clearance, measurement))
      {
        flush_run();
        continue;
      }

      if (!run_active)
      {
        run_active = true;
        run_start_deg = angle_deg;
      }
      run_end_deg = angle_deg;
      run_center_range = std::max(run_center_range, center_range);
      run_min_width = std::min(run_min_width, measurement.width_m);
    }
    flush_run();
    return sectors;
  }

  GapWidthMeasurement measure_gap_width(
      const sensor_msgs::msg::LaserScan &scan,
      double center_deg,
      double center_range_m) const
  {
    GapWidthMeasurement measurement;
    const bool has_left = find_gap_boundary(scan, center_deg, center_range_m, 1.0, measurement.left_angle_rad, measurement.left_range_m);
    const bool has_right = find_gap_boundary(scan, center_deg, center_range_m, -1.0, measurement.right_angle_rad, measurement.right_range_m);
    if (!has_left || !has_right)
    {
      return measurement;
    }

    const double angle_between =
        std::fabs(normalize_angle_rad(measurement.left_angle_rad - measurement.right_angle_rad));
    const double width_squared =
        measurement.left_range_m * measurement.left_range_m +
        measurement.right_range_m * measurement.right_range_m -
        2.0 * measurement.left_range_m * measurement.right_range_m * std::cos(angle_between);
    measurement.width_m = std::sqrt(std::max(0.0, width_squared));
    measurement.measured = true;
    return measurement;
  }

  bool gap_is_too_small(
      double center_deg,
      double center_range_m,
      double sector_clearance_m,
      const GapWidthMeasurement &measurement) const
  {
    if (!measurement.measured || std::fabs(center_deg) > gap_search_deg_)
    {
      return false;
    }
    if (measurement.width_m >= min_passable_gap_m_)
    {
      return false;
    }
    if (center_range_m < gap_min_center_distance_m_ || sector_clearance_m < scan_obstacle_min_range_m_)
    {
      return false;
    }
    const double farther_boundary_range =
        std::max(measurement.left_range_m, measurement.right_range_m);
    return center_range_m >= farther_boundary_range + gap_min_depth_gain_m_;
  }

  bool find_gap_boundary(
      const sensor_msgs::msg::LaserScan &scan,
      double center_deg,
      double center_range_m,
      double direction,
      double &boundary_angle_rad,
      double &boundary_range_m) const
  {
    const double step_deg = std::max(1.0, scan_sample_step_deg_);
    const double max_offset = std::max(step_deg, gap_boundary_search_deg_);
    for (double offset_deg = step_deg; offset_deg <= max_offset; offset_deg += step_deg)
    {
      const double angle_deg = center_deg + sign_value(direction) * offset_deg;
      double range = 0.0;
      if (!range_at_angle(scan, deg_to_rad(angle_deg), range) || !range_is_usable(scan, range))
      {
        continue;
      }
      const bool close_enough = range <= gap_boundary_obstacle_max_range_m_;
      const bool shallower_than_center =
          range <= center_range_m - std::max(0.0, gap_boundary_drop_m_);
      if (close_enough && shallower_than_center)
      {
        boundary_angle_rad = deg_to_rad(angle_deg);
        boundary_range_m = range;
        return true;
      }
    }
    return false;
  }

  bool apply_recovery_overrides(
      DriveOutput &output,
      const LocalGrid &grid,
      const rclcpp::Time &now,
      const Pose2D &odom_pose)
  {
    if (recovery_phase_ != RecoveryPhase::IDLE)
    {
      return run_active_recovery(output, grid, now);
    }

    if (output.vectors.front_m <= emergency_stop_distance_m_)
    {
      record_dead_end_memory(odom_pose, 0.0, now, "emergency");
      if (rear_corridor_is_clear(grid, emergency_backup_speed_ * emergency_backup_seconds_))
      {
        recovery_phase_ = RecoveryPhase::EMERGENCY_BACKUP;
        recovery_started_at_ = now;
        return run_active_recovery(output, grid, now);
      }
      output.linear_x = 0.0;
      output.angular_z = choose_pivot_turn(grid, recovery_turn_speed_);
      output.mode = "EMERGENCY_PIVOT";
      output.detail = "front emergency distance, rear is not clear";
      return true;
    }

    if (stuck_recovery_enabled_)
    {
      update_stuck_recovery(output, grid, now, odom_pose);
      if (recovery_phase_ != RecoveryPhase::IDLE)
      {
        return run_active_recovery(output, grid, now);
      }
    }
    return false;
  }

  bool run_active_recovery(DriveOutput &output, const LocalGrid &grid, const rclcpp::Time &now)
  {
    const double elapsed = (now - recovery_started_at_).seconds();
    if (recovery_phase_ == RecoveryPhase::EMERGENCY_BACKUP)
    {
      if (elapsed <= emergency_backup_seconds_)
      {
        output.linear_x = -std::fabs(emergency_backup_speed_);
        output.angular_z = 0.0;
        output.mode = "EMERGENCY_BACKUP";
        output.detail = "front emergency distance, backing up through clear rear corridor";
        return true;
      }
      recovery_phase_ = RecoveryPhase::IDLE;
      return false;
    }
    if (recovery_phase_ == RecoveryPhase::STUCK_BACKUP)
    {
      if (elapsed <= stuck_backup_seconds_)
      {
        output.linear_x = -std::fabs(stuck_backup_speed_);
        output.angular_z = 0.0;
        output.mode = "STUCK_BACKUP";
        output.detail = "odom progress stuck, backing up through clear rear corridor";
        return true;
      }
      recovery_phase_ = RecoveryPhase::IDLE;
      last_stuck_recovery_finished_at_ = now;
      return false;
    }
    if (recovery_phase_ == RecoveryPhase::RECOVERY_TURN)
    {
      if (elapsed <= stuck_turn_seconds_)
      {
        output.linear_x = 0.0;
        output.angular_z = choose_pivot_turn(grid, stuck_turn_speed_);
        output.mode = "STUCK_TURN";
        output.detail = "odom progress stuck, rear not clear";
        return true;
      }
      recovery_phase_ = RecoveryPhase::IDLE;
      last_stuck_recovery_finished_at_ = now;
      return false;
    }
    return false;
  }

  void update_stuck_recovery(
      const DriveOutput &output,
      const LocalGrid &grid,
      const rclcpp::Time &now,
      const Pose2D &odom_pose)
  {
    if ((now - last_stuck_recovery_finished_at_).seconds() < stuck_cooldown_seconds_)
    {
      return;
    }
    if (output.linear_x <= stuck_command_linear_threshold_)
    {
      stuck_watch_active_ = false;
      return;
    }
    if (!stuck_watch_active_)
    {
      stuck_watch_active_ = true;
      stuck_watch_started_at_ = now;
      stuck_watch_start_pose_ = odom_pose;
      return;
    }

    const double dx = odom_pose.x - stuck_watch_start_pose_.x;
    const double dy = odom_pose.y - stuck_watch_start_pose_.y;
    const double progress = std::sqrt(dx * dx + dy * dy);
    const double elapsed = (now - stuck_watch_started_at_).seconds();
    if (progress >= stuck_min_progress_m_)
    {
      stuck_watch_started_at_ = now;
      stuck_watch_start_pose_ = odom_pose;
      return;
    }
    if (elapsed < stuck_hold_seconds_)
    {
      return;
    }

    stuck_watch_active_ = false;
    record_dead_end_memory(
        odom_pose,
        std::atan2(output.vectors.final_vector.y, output.vectors.final_vector.x),
        now,
        "stuck");
    if (rear_corridor_is_clear(grid, stuck_backup_speed_ * stuck_backup_seconds_))
    {
      recovery_phase_ = RecoveryPhase::STUCK_BACKUP;
    }
    else
    {
      recovery_phase_ = RecoveryPhase::RECOVERY_TURN;
    }
    recovery_started_at_ = now;
  }

  void record_dead_end_memory(
      const Pose2D &odom_pose,
      double bad_local_angle,
      const rclcpp::Time &now,
      const std::string &reason)
  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    if (!dead_end_memory_enabled_runtime_)
    {
      return;
    }
    if (last_dead_end_recorded_at_.nanoseconds() != 0 &&
        (now - last_dead_end_recorded_at_).seconds() < dead_end_record_cooldown_seconds_)
    {
      return;
    }

    const double bad_yaw = normalize_angle_rad(odom_pose.yaw + bad_local_angle);
    for (auto &memory : dead_end_memories_)
    {
      const double dx = memory.x - odom_pose.x;
      const double dy = memory.y - odom_pose.y;
      const double distance = std::sqrt(dx * dx + dy * dy);
      const double yaw_error = std::fabs(normalize_angle_rad(memory.yaw - bad_yaw));
      if (distance <= dead_end_merge_distance_m_ &&
          yaw_error <= deg_to_rad(dead_end_merge_angle_deg_))
      {
        const double old_weight = static_cast<double>(std::max(1, memory.hit_count));
        memory.x = (memory.x * old_weight + odom_pose.x) / (old_weight + 1.0);
        memory.y = (memory.y * old_weight + odom_pose.y) / (old_weight + 1.0);
        memory.yaw = bad_yaw;
        memory.stamp = now;
        memory.hit_count += 1;
        last_dead_end_recorded_at_ = now;
        RCLCPP_INFO(
            this->get_logger(),
            "Updated dead-end memory: reason=%s count=%d total=%zu",
            reason.c_str(),
            memory.hit_count,
            dead_end_memories_.size());
        return;
      }
    }

    DeadEndMemory memory;
    memory.x = odom_pose.x;
    memory.y = odom_pose.y;
    memory.yaw = bad_yaw;
    memory.stamp = now;
    dead_end_memories_.push_back(memory);
    while (static_cast<int>(dead_end_memories_.size()) > std::max(1, dead_end_memory_max_entries_))
    {
      dead_end_memories_.pop_front();
    }
    last_dead_end_recorded_at_ = now;
    RCLCPP_INFO(
        this->get_logger(),
        "Recorded dead-end memory: reason=%s yaw=%.0fdeg total=%zu",
        reason.c_str(),
        rad_to_deg(bad_yaw),
        dead_end_memories_.size());
  }

  bool rear_corridor_is_clear(const LocalGrid &grid, double distance) const
  {
    const double max_distance = std::max(0.0, distance);
    if (max_distance <= 1e-6)
    {
      return false;
    }
    const double half_width = robot_radius_m_ + clearance_margin_m_;
    const double distance_step = std::max(grid.resolution, corridor_sample_step_m_);
    const double lateral_step = std::max(grid.resolution, corridor_sample_step_m_);
    for (double x = 0.0; x >= -max_distance; x -= distance_step)
    {
      for (double y = -half_width; y <= half_width + 1e-6; y += lateral_step)
      {
        double state = kUnknown;
        if (!state_at(grid, x, y, state) ||
            (state > kFree && state <= kUnknown) ||
            state >= kBlocked)
        {
          return false;
        }
      }
    }
    return true;
  }

  double choose_pivot_turn(const LocalGrid &grid, double turn_speed)
  {
    const rclcpp::Time now = this->now();
    if (pivot_hold_active_ && (now - pivot_hold_started_at_).seconds() <= pivot_hold_seconds_)
    {
      return static_cast<double>(pivot_direction_) * std::fabs(turn_speed);
    }

    const double left_score = score_direction(grid, deg_to_rad(90.0)).score;
    const double right_score = score_direction(grid, deg_to_rad(-90.0)).score;
    if (std::fabs(left_score - right_score) < 1e-6)
    {
      pivot_direction_ = prefer_left_recovery_ ? 1 : -1;
    }
    else
    {
      pivot_direction_ = left_score < right_score ? 1 : -1;
    }
    pivot_hold_active_ = true;
    pivot_hold_started_at_ = now;
    return static_cast<double>(pivot_direction_) * std::fabs(turn_speed);
  }

  double min_range_in_sector(
      const sensor_msgs::msg::LaserScan &scan,
      double center_deg,
      double half_width_deg) const
  {
    double best = std::numeric_limits<double>::infinity();
    const double center = deg_to_rad(center_deg);
    const double half = deg_to_rad(std::max(0.0, half_width_deg));
    for (int i = 0; i < static_cast<int>(scan.ranges.size()); ++i)
    {
      const double angle = scan.angle_min + static_cast<double>(i) * scan.angle_increment;
      if (std::fabs(normalize_angle_rad(angle - center)) > half)
      {
        continue;
      }
      const double range = scan.ranges[static_cast<std::size_t>(i)];
      if (range_is_usable(scan, range))
      {
        best = std::min(best, range);
      }
    }
    return std::isfinite(best) ? best : grid_radius_m_;
  }

  bool range_at_angle(
      const sensor_msgs::msg::LaserScan &scan,
      double angle_rad,
      double &range) const
  {
    if (scan.ranges.empty() || std::fabs(scan.angle_increment) < 1e-9)
    {
      return false;
    }
    const int index = static_cast<int>(std::round((angle_rad - scan.angle_min) / scan.angle_increment));
    if (index < 0 || index >= static_cast<int>(scan.ranges.size()))
    {
      return false;
    }
    range = scan.ranges[static_cast<std::size_t>(index)];
    return true;
  }

  bool range_is_usable(const sensor_msgs::msg::LaserScan &scan, double range) const
  {
    if (!std::isfinite(range) || range <= scan_obstacle_min_range_m_)
    {
      return false;
    }
    if (std::isfinite(scan.range_max) && scan.range_max > 0.0 && range > scan.range_max)
    {
      return false;
    }
    return true;
  }

  int scan_stride(const sensor_msgs::msg::LaserScan &scan) const
  {
    if (std::fabs(scan.angle_increment) < 1e-9)
    {
      return 1;
    }
    return std::max(
        1,
        static_cast<int>(std::round(deg_to_rad(std::max(0.1, scan_sample_step_deg_)) /
                                    std::fabs(scan.angle_increment))));
  }

  Vector2D add_vectors(const Vector2D &a, const Vector2D &b) const
  {
    Vector2D result;
    result.x = a.x + b.x;
    result.y = a.y + b.y;
    return result;
  }

  double vector_norm(const Vector2D &v) const
  {
    return std::sqrt(v.x * v.x + v.y * v.y);
  }

  Vector2D normalize_vector(const Vector2D &v) const
  {
    const double norm = vector_norm(v);
    if (norm <= 1e-9)
    {
      return Vector2D();
    }
    Vector2D result;
    result.x = v.x / norm;
    result.y = v.y / norm;
    return result;
  }

  Vector2D limit_vector(const Vector2D &v, double max_norm) const
  {
    const double norm = vector_norm(v);
    if (norm <= 1e-9 || norm <= max_norm)
    {
      return v;
    }
    Vector2D result;
    result.x = v.x * max_norm / norm;
    result.y = v.y * max_norm / norm;
    return result;
  }

  void publish_local_grid(const LocalGrid &grid)
  {
    nav_msgs::msg::OccupancyGrid msg;
    msg.header.stamp = this->now();
    msg.header.frame_id = grid_frame_id_;
    msg.info.resolution = static_cast<float>(grid.resolution);
    msg.info.width = static_cast<std::uint32_t>(grid.width);
    msg.info.height = static_cast<std::uint32_t>(grid.height);
    msg.info.origin.position.x = grid.origin_x;
    msg.info.origin.position.y = grid.origin_y;
    msg.info.origin.position.z = 0.0;
    msg.info.origin.orientation.w = 1.0;
    msg.data.resize(grid.cells.size());
    for (std::size_t i = 0; i < grid.cells.size(); ++i)
    {
      if (grid.cells[i] <= kFree)
      {
        msg.data[i] = 0;
      }
      else if (grid.cells[i] >= kBlocked)
      {
        msg.data[i] = 100;
      }
      else if (grid.cells[i] > kUnknown)
      {
        msg.data[i] = 75;
      }
      else
      {
        msg.data[i] = -1;
      }
    }
    local_grid_publisher_->publish(msg);
  }

  void publish_vector_markers(const DriveOutput &output)
  {
    visualization_msgs::msg::MarkerArray array;
    add_vector_marker(array, "lowest_cost", 1, output.vectors.lowest_cost, 0.0f, 1.0f, 0.2f);
    add_vector_marker(array, "repulsion", 2, output.vectors.repulsion, 1.0f, 0.0f, 1.0f);
    add_vector_marker(array, "heading_bias", 3, output.vectors.heading_bias, 0.0f, 0.3f, 1.0f);
    add_vector_marker(array, "reverse_bias", 4, output.vectors.reverse_bias, 0.0f, 1.0f, 1.0f);
    add_vector_marker(array, "final_vector", 5, output.vectors.final_vector, 1.0f, 0.55f, 0.0f);
    vector_marker_publisher_->publish(array);
  }

  void add_vector_marker(
      visualization_msgs::msg::MarkerArray &array,
      const std::string &ns,
      int id,
      const Vector2D &vector,
      float r,
      float g,
      float b)
  {
    visualization_msgs::msg::Marker marker;
    marker.header.stamp = this->now();
    marker.header.frame_id = grid_frame_id_;
    marker.ns = ns;
    marker.id = id;
    marker.type = visualization_msgs::msg::Marker::ARROW;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.scale.x = 0.035;
    marker.scale.y = 0.07;
    marker.scale.z = 0.08;
    marker.color.r = r;
    marker.color.g = g;
    marker.color.b = b;
    marker.color.a = 0.95f;
    marker.lifetime.sec = 0;
    marker.lifetime.nanosec = 500000000;
    marker.pose.orientation.w = 1.0;

    geometry_msgs::msg::Point start;
    start.x = 0.0;
    start.y = 0.0;
    start.z = 0.10 + 0.02 * static_cast<double>(id);
    marker.points.push_back(start);

    const Vector2D limited = limit_vector(vector, 1.0);
    geometry_msgs::msg::Point end;
    end.x = limited.x;
    end.y = limited.y;
    end.z = start.z;
    marker.points.push_back(end);
    array.markers.push_back(marker);
  }

  void publish_velocity(double linear_x, double angular_z)
  {
    if (cmd_vel_stamped_)
    {
      geometry_msgs::msg::TwistStamped cmd;
      cmd.header.stamp = this->now();
      cmd.header.frame_id = grid_frame_id_;
      cmd.twist.linear.x = linear_x;
      cmd.twist.angular.z = angular_z;
      cmd_vel_stamped_publisher_->publish(cmd);
      return;
    }

    geometry_msgs::msg::Twist cmd;
    cmd.linear.x = linear_x;
    cmd.angular.z = angular_z;
    cmd_vel_publisher_->publish(cmd);
  }

  void store_output(const DriveOutput &output)
  {
    std::lock_guard<std::mutex> lock(output_mutex_);
    last_output_ = output;
  }

  double path_distance_snapshot() const
  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    return path_distance_m_;
  }

  double heading_reference_yaw_snapshot() const
  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    return heading_reference_yaw_;
  }

  bool heading_reference_valid_snapshot() const
  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    return heading_reference_valid_;
  }

  bool dead_end_memory_enabled_snapshot() const
  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    return dead_end_memory_enabled_runtime_;
  }

  int dead_end_memory_count_snapshot() const
  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    return static_cast<int>(dead_end_memories_.size());
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
        "[%s] enabled=%s imu=%s odom=%s ref=%s tilt=%.1fdeg path=%.2fm "
        "front=%.2fm best=%.0fdeg score=%.2f blocked=%d/%d rev_bias=%s dead=%s/%d "
        "cmd=(%.2f, %.2f) %s",
        output.mode.c_str(),
        output.enabled_requested ? "yes" : "no",
        output.imu_ready ? "yes" : "no",
        output.odom_ready ? "yes" : "no",
        output.heading_reference_valid ? "yes" : "no",
        output.tilt_deg,
        output.path_distance_m,
        output.vectors.front_m,
        rad_to_deg(output.vectors.best_angle_rad),
        output.vectors.best_score,
        output.vectors.blocked_directions,
        output.vectors.sampled_directions,
        output.vectors.reverse_bias_active ? "yes" : "no",
        output.vectors.dead_end_memory_enabled ? "yes" : "no",
        output.vectors.dead_end_memory_count,
        output.linear_x,
        output.angular_z,
        output.detail.c_str());
  }

  const std::string scan_topic_;
  const std::string imu_topic_;
  const std::string odom_topic_;
  const std::string cmd_vel_topic_;
  const bool cmd_vel_stamped_;
  const std::string local_grid_topic_;
  const std::string vector_marker_topic_;
  const std::string grid_frame_id_;
  const bool auto_start_;
  const bool reset_heading_on_start_;
  const bool require_odom_;
  const int control_period_ms_;
  const int report_period_ms_;

  const double max_linear_speed_;
  const double min_linear_speed_;
  const double max_angular_speed_;
  const double steering_kp_;
  const double recovery_turn_speed_;
  const bool prefer_left_recovery_;
  const double emergency_stop_distance_m_;
  const double front_stop_distance_m_;
  const double front_slow_distance_m_;
  const double front_window_deg_;
  const double scan_sample_step_deg_;
  const double scan_obstacle_min_range_m_;
  const double scan_obstacle_max_range_m_;
  const double lost_scan_timeout_seconds_;

  const double grid_radius_m_;
  const double grid_resolution_m_;
  const double grid_memory_seconds_;
  const int grid_memory_max_points_;
  const double obstacle_thickness_m_;
  const double robot_radius_m_;
  const double clearance_margin_m_;
  const double direction_sample_step_deg_;
  const double corridor_sample_step_m_;
  const double direction_score_tolerance_ratio_;
  const double direction_score_tolerance_abs_;
  const double unknown_direction_penalty_;
  const double blocked_direction_penalty_;
  const double min_vector_norm_;
  const double repulsion_range_m_;
  const double repulsion_weight_;
  const double lowest_cost_weight_;
  const double heading_forward_weight_initial_;
  const double heading_decay_distance_m_;
  const double reverse_heading_threshold_deg_;
  const double reverse_bias_start_m_;
  const double reverse_bias_full_m_;
  const double reverse_bias_max_weight_;

  const double min_passable_gap_m_;
  const double gap_search_deg_;
  const double gap_sector_half_width_deg_;
  const double gap_boundary_search_deg_;
  const double gap_boundary_obstacle_max_range_m_;
  const double gap_boundary_drop_m_;
  const double gap_min_center_distance_m_;
  const double gap_min_depth_gain_m_;

  const double tilt_stop_deg_;
  const double imu_timeout_seconds_;
  const double odom_timeout_seconds_;
  const bool stuck_recovery_enabled_;
  const double stuck_command_linear_threshold_;
  const double stuck_min_progress_m_;
  const double stuck_hold_seconds_;
  const double stuck_backup_speed_;
  const double stuck_backup_seconds_;
  const double stuck_turn_speed_;
  const double stuck_turn_seconds_;
  const double stuck_cooldown_seconds_;
  const double emergency_backup_speed_;
  const double emergency_backup_seconds_;
  const double pivot_hold_seconds_;
  const bool dead_end_memory_enabled_default_;
  const double dead_end_memory_seconds_;
  const int dead_end_memory_max_entries_;
  const double dead_end_memory_cell_cost_;
  const double dead_end_record_cooldown_seconds_;
  const double dead_end_merge_distance_m_;
  const double dead_end_merge_angle_deg_;
  const double dead_end_backtrack_distance_m_;
  const double dead_end_forward_extension_m_;
  const double dead_end_corridor_half_width_m_;

  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_subscription_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscription_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscription_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_publisher_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr cmd_vel_stamped_publisher_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr local_grid_publisher_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr vector_marker_publisher_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr set_enabled_service_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr set_dead_end_memory_service_;
  rclcpp::TimerBase::SharedPtr control_timer_;
  rclcpp::TimerBase::SharedPtr report_timer_;

  mutable std::mutex data_mutex_;
  mutable std::mutex output_mutex_;
  sensor_msgs::msg::LaserScan::SharedPtr latest_scan_;
  rclcpp::Time latest_scan_time_;
  rclcpp::Time latest_imu_time_;
  rclcpp::Time latest_odom_time_;
  rclcpp::Time last_integrated_scan_time_;
  bool enabled_requested_ = false;
  bool imu_ready_ = false;
  bool odom_ready_ = false;
  bool heading_reference_valid_ = false;
  double heading_reference_yaw_ = 0.0;
  double max_tilt_deg_ = 0.0;
  double path_distance_m_ = 0.0;
  Pose2D odom_pose_;
  std::deque<MemoryPoint> memory_points_;
  std::deque<DeadEndMemory> dead_end_memories_;
  DriveOutput last_output_;

  bool dead_end_memory_enabled_runtime_ = false;
  bool reverse_bias_active_ = false;
  double reverse_bias_start_distance_m_ = 0.0;
  RecoveryPhase recovery_phase_ = RecoveryPhase::IDLE;
  rclcpp::Time recovery_started_at_;
  rclcpp::Time last_stuck_recovery_finished_at_;
  bool stuck_watch_active_ = false;
  rclcpp::Time stuck_watch_started_at_;
  rclcpp::Time last_dead_end_recorded_at_;
  Pose2D stuck_watch_start_pose_;
  bool pivot_hold_active_ = false;
  rclcpp::Time pivot_hold_started_at_;
  int pivot_direction_ = 1;
};

int main(int argc, char **argv)
{
  signal(SIGINT, signal_handler);
  rclcpp::init(argc, argv);
  auto node = std::make_shared<TurtlebotAutonomousGridDrive>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
