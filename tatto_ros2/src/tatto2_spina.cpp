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
  void spina_angle(double x, double y, double z)
  {
    double pitch = std::atan2(x, std::sqrt(y * y + z * z));
    double yaw   = std::atan2(y, std::sqrt(x * x + z * z));

    int pitch_deg = static_cast<int>(pitch * 180.0 / M_PI);
    int yaw_deg   = static_cast<int>(yaw   * 180.0 / M_PI);

    publish_value("A0p", pitch_deg);
    publish_value("A0r", yaw_deg);
  }

  int clamp_deg(int v) const
  {
    if (v > max_deg_) return max_deg_;
    if (v < min_deg_) return min_deg_;
    return v;
  }

  // "A0p-090" / "A0r045" 形式（正値は符号なし、3桁ゼロ埋め）
  std::string format_cmd(const std::string &prefix, int value) const
  {
    value = clamp_deg(value);

    std::ostringstream ss;
    ss << prefix;
    if (value < 0) {
      ss << '-';
    }
    ss << std::setw(3) << std::setfill('0') << std::abs(value);
    return ss.str();
  }

  void publish_value(const char *prefix, int value)
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
        "max sensor: %zu (%.1f)\nposition = (%.3f, %.3f, %.3f)",
        index, value,
        tf.transform.translation.x,
        tf.transform.translation.y,
        tf.transform.translation.z);

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
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PhotosensorMarkerHSVNode>());
  rclcpp::shutdown();
  return 0;
}