#include <algorithm>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <std_msgs/msg/color_rgba.hpp>

#include <tatto_ros2_msgs/msg/sensor_array.hpp>

class SensorArrayToMarkerNode : public rclcpp::Node
{
public:
  SensorArrayToMarkerNode()
  : Node("sensor_array_to_marker_node")
  {
    input_topic_  = declare_parameter<std::string>("input_topic", "/tatto/sensor_values_raw");
    marker_topic_ = declare_parameter<std::string>("marker_topic", "/tatto/sensor_markers");
    frame_id_fallback_ = declare_parameter<std::string>("frame_id_fallback", "tatto_link");

    vmin_ = declare_parameter<double>("vmin", 150.0);
    vmax_ = declare_parameter<double>("vmax", 210.0);

    cols_ = declare_parameter<int>("cols", 8);   // 32ch想定で8x4
    spacing_ = declare_parameter<double>("spacing", 0.012);

    scale_x_ = declare_parameter<double>("scale_x", 0.01);
    scale_y_ = declare_parameter<double>("scale_y", 0.01);
    scale_z_ = declare_parameter<double>("scale_z", 0.002);

    marker_pub_ = create_publisher<visualization_msgs::msg::Marker>(marker_topic_, 10);

    sub_ = create_subscription<tatto_ros2_msgs::msg::SensorArray>(
      input_topic_, 10,
      std::bind(&SensorArrayToMarkerNode::on_msg, this, std::placeholders::_1));

    RCLCPP_INFO(get_logger(), "Subscribed: %s", input_topic_.c_str());
    RCLCPP_INFO(get_logger(), "Publishing: %s", marker_topic_.c_str());
  }

private:
  double normalize(double v) const
  {
    if (vmax_ <= vmin_) return 0.0;
    double x = (v - vmin_) / (vmax_ - vmin_);
    return std::clamp(x, 0.0, 1.0);
  }

  void on_msg(const tatto_ros2_msgs::msg::SensorArray::SharedPtr msg)
  {
    const auto & values = msg->data.data;  // ★ここが重要（Float32MultiArrayの実データ）

    visualization_msgs::msg::Marker marker;
    marker.header.stamp = now();
    marker.header.frame_id = msg->header.frame_id.empty() ? frame_id_fallback_ : msg->header.frame_id;

    marker.ns = "tatto_sensor";
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::CUBE_LIST;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.orientation.w = 1.0;

    marker.scale.x = scale_x_;
    marker.scale.y = scale_y_;
    marker.scale.z = scale_z_;

    marker.points.reserve(values.size());
    marker.colors.reserve(values.size());

    for (size_t i = 0; i < values.size(); ++i) {
      const int row = static_cast<int>(i) / cols_;
      const int col = static_cast<int>(i) % cols_;

      geometry_msgs::msg::Point p;
      p.x = static_cast<double>(col) * spacing_;
      p.y = static_cast<double>(row) * spacing_;
      p.z = 0.0;
      marker.points.push_back(p);

      const float g = static_cast<float>(normalize(static_cast<double>(values[i])));
      std_msgs::msg::ColorRGBA c;
      c.r = 0.0f;
      c.g = g;
      c.b = 0.0f;
      c.a = 1.0f;
      marker.colors.push_back(c);
    }

    marker_pub_->publish(marker);
  }

  rclcpp::Subscription<tatto_ros2_msgs::msg::SensorArray>::SharedPtr sub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;

  std::string input_topic_;
  std::string marker_topic_;
  std::string frame_id_fallback_;

  double vmin_, vmax_;
  int cols_;
  double spacing_;
  double scale_x_, scale_y_, scale_z_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SensorArrayToMarkerNode>());
  rclcpp::shutdown();
  return 0;
}