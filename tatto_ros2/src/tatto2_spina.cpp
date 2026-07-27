#include <memory>
#include <iostream>
#include <cmath>
#include <algorithm>
#include <sstream>
#include <iomanip>
#include <cstdlib>
#include <rclcpp/rclcpp.hpp>
#include <opencv2/opencv.hpp>

#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <tf2_ros/buffer.hpp>
#include <tf2_ros/transform_broadcaster.hpp>
#include <tf2_ros/transform_listener.hpp>
#include <tf2/exceptions.hpp>
#include <tatto_ros2_msgs/msg/sensor_array.hpp>
#include <std_msgs/msg/string.hpp>

class PhotosensorMarkerHSVNode : public rclcpp::Node
{
public:
  PhotosensorMarkerHSVNode()
  : Node("photosensor_marker_hsv_node")
  {
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    sub_ = create_subscription<tatto_ros2_msgs::msg::SensorArray>(
      "/tatto/sensor_values",
      rclcpp::SensorDataQoS(),
      std::bind(&PhotosensorMarkerHSVNode::callback, this, std::placeholders::_1));

    pub_ = this->create_publisher<std_msgs::msg::String>(
      "/angle_cmd", rclcpp::QoS(10).reliable());
  }

private:
  int apply_rate_limit(int target, int last) 
  {
    int diff = target - last;
    if (diff >  max_step_deg_) diff =  max_step_deg_;
    if (diff < -max_step_deg_) diff = -max_step_deg_;
    return last + diff;
  }

  void spina_angle(double x, double y, double z)
  {
    const double eps = 1e-6;
    const double norm = std::sqrt(x*x + y*y + z*z);
    if (norm < eps) return;

    // unit vector
    const double nx = x / norm;
    const double ny = y / norm;
    const double nz = z / norm;

    // pitch / roll
    double roll  = std::atan2(ny, nz);
    double pitch = std::atan2(nx, std::sqrt(ny*ny + nz*nz));

    int pitch_deg = static_cast<int>(std::round(pitch * 180.0 / M_PI));
    int roll_deg  = static_cast<int>(std::round(roll  * 180.0 / M_PI));

    int pitch_tgt = static_cast<int>(std::round(kp_ * pitch_deg));
    int roll_tgt  = static_cast<int>(std::round(kp_ * roll_deg));

    if (std::abs(pitch_tgt) < deadband_deg_) pitch_tgt = 0;
    if (std::abs(roll_tgt)  < deadband_deg_) roll_tgt  = 0;

    int pitch_cmd = apply_rate_limit(pitch_tgt, last_pitch_cmd_);
    int roll_cmd  = apply_rate_limit(roll_tgt,  last_roll_cmd_);

    publish_value("A0p", pitch_cmd);
    publish_value("A0r", roll_cmd);

    last_pitch_cmd_ = pitch_cmd;
    last_roll_cmd_  = roll_cmd;
  }

  int clamp_deg(int v) const
  {
    if (v > max_deg_) return max_deg_;
    if (v < min_deg_) return min_deg_;
    return v;
  }

  std::string format_cmd(const std::string &prefix, int value) const
  {
    value = clamp_deg(value);

    std::ostringstream ss;
    ss << prefix
       << (value < 0 ? '-' : '+')
       << std::setw(3)
       << std::setfill('0')
       << std::abs(value);
    return ss.str();
  }

  void publish_value(const std::string &prefix, int value)
  {
    std_msgs::msg::String out;
    out.data = format_cmd(prefix, value);
    pub_->publish(out);
  }

  void callback(const tatto_ros2_msgs::msg::SensorArray::SharedPtr msg)
  {
    const auto &values = msg->data.data;
    if (values.empty()) return;

    auto max_it = std::max_element(values.begin(), values.end());
    size_t index = std::distance(values.begin(), max_it);
    float value = *max_it;

    std::string frame = "photosensor_" + std::to_string(index);

    try {
      auto tf = tf_buffer_->lookupTransform("module4_upper_link", frame, tf2::TimePointZero);

      spina_angle(
        tf.transform.translation.x,
        tf.transform.translation.y,
        tf.transform.translation.z);

      RCLCPP_INFO(
        get_logger(),
        "max sensor: %zu (%.1f) position = (%.3f, %.3f, %.3f)",
        index, value,
        tf.transform.translation.x,
        tf.transform.translation.y,
        tf.transform.translation.z);

      auto tf_tatto = tf_buffer_->lookupTransform("tatto_link", frame, tf2::TimePointZero);

      RCLCPP_INFO(
        get_logger(),
        "tatto: %zu (%.1f) position = (%.3f, %.3f, %.3f)",
        index, value,
        tf_tatto.transform.translation.x,
        tf_tatto.transform.translation.y,
        tf_tatto.transform.translation.z);

    } catch (const tf2::TransformException &ex) {
      RCLCPP_WARN(get_logger(), "%s", ex.what());
    }
  }

  rclcpp::Subscription<tatto_ros2_msgs::msg::SensorArray>::SharedPtr sub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  int min_deg_ = -90;
  int max_deg_ = 90;
  double kp_ = 0.8;
  int deadband_deg_ = 3;
  int max_step_deg_ = 4;
  int last_pitch_cmd_ = 0;
  int last_roll_cmd_ = 0;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PhotosensorMarkerHSVNode>());
  rclcpp::shutdown();
  return 0;
}