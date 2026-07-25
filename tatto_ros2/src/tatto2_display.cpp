#include <algorithm>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <std_msgs/msg/color_rgba.hpp>

#include <tatto_ros2_msgs/msg/sensor_array.hpp>

class SensorArrayToMarkerArrayNode : public rclcpp::Node
{
public:
  SensorArrayToMarkerArrayNode()
  : Node("sensor_array_to_marker_array_node")
  {
    input_topic_  = declare_parameter<std::string>("input_topic", "/tatto/sensor_values_raw");
    marker_topic_ = declare_parameter<std::string>("marker_topic", "/tatto/sensor_markers_array");
    fallback_frame_ = declare_parameter<std::string>("fallback_frame", "tatto_link");

    vmin_ = declare_parameter<double>("vmin", 150.0);
    vmax_ = declare_parameter<double>("vmax", 210.0);

    sx_ = declare_parameter<double>("scale_x", 0.01);
    sy_ = declare_parameter<double>("scale_y", 0.01);
    sz_ = declare_parameter<double>("scale_z", 0.002);

    // 例: ["tatto_sensor_0","tatto_sensor_1",...]
    sensor_frames_ = declare_parameter<std::vector<std::string>>(
      "sensor_frames", std::vector<std::string>{});

    pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(marker_topic_, 10);
    sub_ = create_subscription<tatto_ros2_msgs::msg::SensorArray>(
      input_topic_, 10,
      std::bind(&SensorArrayToMarkerArrayNode::on_msg, this, std::placeholders::_1));
  }

private:
  double norm(double v) const
  {
    if (vmax_ <= vmin_) return 0.0;
    return std::clamp((v - vmin_) / (vmax_ - vmin_), 0.0, 1.0);
  }

  void on_msg(const tatto_ros2_msgs::msg::SensorArray::SharedPtr msg)
  {
    const auto & values = msg->data.data;
    visualization_msgs::msg::MarkerArray arr;
    const auto stamp = now();

    for (size_t i = 0; i < values.size(); ++i) {
      visualization_msgs::msg::Marker m;
      m.header.stamp = stamp;

      if (i < sensor_frames_.size() && !sensor_frames_[i].empty()) {
        m.header.frame_id = sensor_frames_[i];          // 各マーカ個別frame
      } else if (!msg->header.frame_id.empty()) {
        m.header.frame_id = msg->header.frame_id;
      } else {
        m.header.frame_id = fallback_frame_;
      }

      m.ns = "tatto_sensor_each_frame";
      m.id = static_cast<int>(i);
      m.type = visualization_msgs::msg::Marker::CUBE;
      m.action = visualization_msgs::msg::Marker::ADD;

      // 各frame原点に表示（=そのセンサ座標系に追従）
      m.pose.position.x = 0.0;
      m.pose.position.y = 0.0;
      m.pose.position.z = 0.0;
      m.pose.orientation.w = 1.0;

      m.scale.x = sx_;
      m.scale.y = sy_;
      m.scale.z = sz_;

      const float g = static_cast<float>(norm(values[i]));
      m.color.r = 0.0f;
      m.color.g = g;
      m.color.b = 0.0f;
      m.color.a = 1.0f;

      arr.markers.push_back(std::move(m));
    }

    pub_->publish(arr);
  }

  rclcpp::Subscription<tatto_ros2_msgs::msg::SensorArray>::SharedPtr sub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_;

  std::string input_topic_, marker_topic_, fallback_frame_;
  double vmin_, vmax_, sx_, sy_, sz_;
  std::vector<std::string> sensor_frames_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SensorArrayToMarkerArrayNode>());
  rclcpp::shutdown();
  return 0;
}