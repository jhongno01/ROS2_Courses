#include <cstdint> // for fixed-width integer types int8_t, int16_t, int32_t, int64_t 등을 사용하기 위함
#include <cstring>
#include <memory>
#include <string>

#include "more_interfaces/msg/hw03_abcd.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/image_encodings.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/imu.hpp"

using std::placeholders::_1;

class Hw03SubRgbdGyro : public rclcpp::Node
{
public:
  Hw03SubRgbdGyro()
  : Node("hw03_sub_rgbd_gyro"),
    latest_abcd_(std::make_shared<more_interfaces::msg::Hw03Abcd>()),
    latest_gyro_(std::make_shared<sensor_msgs::msg::Imu>())
  {
    auto sensor_qos = rclcpp::SensorDataQoS();

    abcd_sub_ = this->create_subscription<more_interfaces::msg::Hw03Abcd>(
      "abcd", 10, std::bind(&Hw03SubRgbdGyro::abcd_callback, this, _1));

    depth_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
      "/camera/camera/aligned_depth_to_color/image_raw",
      sensor_qos,
      std::bind(&Hw03SubRgbdGyro::depth_callback, this, _1));

    color_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
      "/camera/camera/color/image_raw",
      sensor_qos,
      std::bind(&Hw03SubRgbdGyro::color_callback, this, _1));

    gyro_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
      "/camera/camera/gyro/sample",
      sensor_qos,
      std::bind(&Hw03SubRgbdGyro::gyro_callback, this, _1));

    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(500),
      std::bind(&Hw03SubRgbdGyro::print_latest_data, this));
  }

private:
  void abcd_callback(const more_interfaces::msg::Hw03Abcd::SharedPtr msg)
  {
    latest_abcd_ = msg;
    have_abcd_ = true;

    RCLCPP_INFO(
      this->get_logger(), "Received abcd: A=%ld B=%ld C=%ld D=%ld",
      msg->a, msg->b, msg->c, msg->d);
  }

  void depth_callback(const sensor_msgs::msg::Image::SharedPtr msg)
  {
    latest_depth_ = msg;
    have_depth_ = true;
  }

  void color_callback(const sensor_msgs::msg::Image::SharedPtr msg)
  {
    latest_color_ = msg;
    have_color_ = true;
  }

  void gyro_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
  {
    latest_gyro_ = msg;
    have_gyro_ = true;
  }

  bool all_inputs_ready() const
  {
    return have_abcd_ && have_depth_ && have_color_ && have_gyro_;
  }

  bool pixel_in_bounds(const sensor_msgs::msg::Image::SharedPtr & image, int x, int y) const
  {
    return image && x >= 0 && y >= 0 &&
           x < static_cast<int>(image->width) &&
           y < static_cast<int>(image->height);
  }

  bool read_depth_at_pixel(int x, int y, std::string & depth_text) const
  {
    if (!pixel_in_bounds(latest_depth_, x, y)) {
      return false;
    }

    const auto & image = *latest_depth_;
    const std::size_t offset = static_cast<std::size_t>(y) * image.step;

    if (image.encoding == sensor_msgs::image_encodings::TYPE_16UC1) {
      const std::size_t index = offset + static_cast<std::size_t>(x) * sizeof(std::uint16_t);
      if (index + sizeof(std::uint16_t) > image.data.size()) {
        return false;
      }

      std::uint16_t depth_mm = 0;
      std::memcpy(&depth_mm, &image.data[index], sizeof(depth_mm));
      depth_text = std::to_string(depth_mm) + " mm";
      return true;
    }

    if (image.encoding == sensor_msgs::image_encodings::TYPE_32FC1) {
      const std::size_t index = offset + static_cast<std::size_t>(x) * sizeof(float);
      if (index + sizeof(float) > image.data.size()) {
        return false;
      }

      float depth_m = 0.0f;
      std::memcpy(&depth_m, &image.data[index], sizeof(depth_m));
      depth_text = std::to_string(depth_m) + " m";
      return true;
    }

    depth_text = "unsupported depth encoding: " + image.encoding;
    return true;
  }

  bool read_rgb_at_pixel(int x, int y, int & r, int & g, int & b) const
  {
    if (!pixel_in_bounds(latest_color_, x, y)) {
      return false;
    }

    const auto & image = *latest_color_;
    const std::size_t index =
      static_cast<std::size_t>(y) * image.step + static_cast<std::size_t>(x) * image.step / image.width;

    if (image.encoding == sensor_msgs::image_encodings::RGB8) {
      if (index + 2 >= image.data.size()) {
        return false;
      }
      r = image.data[index];
      g = image.data[index + 1];
      b = image.data[index + 2];
      return true;
    }

    if (image.encoding == sensor_msgs::image_encodings::BGR8) {
      if (index + 2 >= image.data.size()) {
        return false;
      }
      b = image.data[index];
      g = image.data[index + 1];
      r = image.data[index + 2];
      return true;
    }

    if (image.encoding == sensor_msgs::image_encodings::RGBA8) {
      if (index + 3 >= image.data.size()) {
        return false;
      }
      r = image.data[index];
      g = image.data[index + 1];
      b = image.data[index + 2];
      return true;
    }

    if (image.encoding == sensor_msgs::image_encodings::BGRA8) {
      if (index + 3 >= image.data.size()) {
        return false;
      }
      b = image.data[index];
      g = image.data[index + 1];
      r = image.data[index + 2];
      return true;
    }

    return false;
  }

  void print_latest_data()
  {
    if (!all_inputs_ready()) {
      RCLCPP_INFO_THROTTLE(
        this->get_logger(), *this->get_clock(), 3000,
        "Waiting for abcd, depth image, color image, and gyro data...");
      return;
    }

    const int x = static_cast<int>(latest_abcd_->a * latest_abcd_->b);
    const int y = static_cast<int>(latest_abcd_->c * latest_abcd_->d);

    std::string depth_text;
    if (!read_depth_at_pixel(x, y, depth_text)) {
      RCLCPP_WARN(
        this->get_logger(),
        "Depth pixel (%d, %d) is out of bounds. Depth image size: %u x %u",
        x, y, latest_depth_->width, latest_depth_->height);
      return;
    }

    int r = 0;
    int g = 0;
    int b = 0;
    if (!read_rgb_at_pixel(x, y, r, g, b)) {
      RCLCPP_WARN(
        this->get_logger(),
        "Color pixel (%d, %d) could not be read. Encoding: %s, image size: %u x %u",
        x, y, latest_color_->encoding.c_str(), latest_color_->width, latest_color_->height);
      return;
    }

    RCLCPP_INFO(
      this->get_logger(),
      "\n(A*B, C*D) = (%d, %d)\nDepth: %s\nRGB: r=%d g=%d b=%d\nGyro: x=%.6f y=%.6f z=%.6f",
      x, y, depth_text.c_str(), r, g, b,
      latest_gyro_->angular_velocity.x,
      latest_gyro_->angular_velocity.y,
      latest_gyro_->angular_velocity.z);
  }

  rclcpp::Subscription<more_interfaces::msg::Hw03Abcd>::SharedPtr abcd_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr color_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr gyro_sub_;
  rclcpp::TimerBase::SharedPtr timer_;

  more_interfaces::msg::Hw03Abcd::SharedPtr latest_abcd_;
  sensor_msgs::msg::Image::SharedPtr latest_depth_;
  sensor_msgs::msg::Image::SharedPtr latest_color_;
  sensor_msgs::msg::Imu::SharedPtr latest_gyro_;

  bool have_abcd_ = false;
  bool have_depth_ = false;
  bool have_color_ = false;
  bool have_gyro_ = false;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<Hw03SubRgbdGyro>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
