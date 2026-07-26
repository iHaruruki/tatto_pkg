#include <algorithm>
#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <opencv2/opencv.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <tatto_ros2_msgs/msg/sensor_array.hpp>

class SensorArrayToPhotosensorMarkersNode : public rclcpp::Node
{
public:
  SensorArrayToPhotosensorMarkersNode()
  : Node("sensor_array_to_photosensor_markers_node")
  {
    input_topic_  = declare_parameter<std::string>("input_topic", "/tatto/sensor_values");
    output_topic_ = declare_parameter<std::string>("output_topic", "/tatto/markers/photosensor");

    // 値→緑強度の正規化レンジ
    vmin_ = declare_parameter<double>("vmin", 160.0);
    vmax_ = declare_parameter<double>("vmax", 350.0);
    // vmin_ = declare_parameter<double>("vmin", 250.0);
    // vmax_ = declare_parameter<double>("vmax", 320.0);

    // マーカサイズ
    sx_ = declare_parameter<double>("scale_x", 0.004);
    sy_ = declare_parameter<double>("scale_y", 0.004);
    sz_ = declare_parameter<double>("scale_z", 0.003);

    // photosensor_N の開始インデックス
    sensor_index_offset_ = declare_parameter<int>("sensor_index_offset", 0);

    sub_ = create_subscription<tatto_ros2_msgs::msg::SensorArray>(
      input_topic_, rclcpp::SensorDataQoS(),
      std::bind(&SensorArrayToPhotosensorMarkersNode::onMsg, this, std::placeholders::_1));

    pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(output_topic_, 10);

    RCLCPP_INFO(get_logger(), "sub: %s", input_topic_.c_str());
    RCLCPP_INFO(get_logger(), "pub: %s", output_topic_.c_str());
  }

private:
  double normalize(double v) const
  {
    if (vmax_ <= vmin_) return 0.0;
    return std::clamp((v - vmin_) / (vmax_ - vmin_), 0.0, 1.0);
  }

  void onMsg(const tatto_ros2_msgs::msg::SensorArray::SharedPtr msg)
  {
    const auto & values = msg->data.data;
    visualization_msgs::msg::MarkerArray arr;
    const auto stamp = now();

    // {
    //   visualization_msgs::msg::Marker del;
    //   del.action = visualization_msgs::msg::Marker::DELETEALL;
    //   arr.markers.push_back(del);
    // }

    for (size_t i = 0; i < values.size(); ++i) {
      visualization_msgs::msg::Marker m;
      m.header.stamp = stamp;
      m.header.frame_id = "photosensor_" + std::to_string(sensor_index_offset_ + static_cast<int>(i));

      m.ns = "photosensor_values";
      m.id = static_cast<int>(i);
      // m.type = visualization_msgs::msg::Marker::CUBE;
      m.type = visualization_msgs::msg::Marker::SPHERE;
      m.action = visualization_msgs::msg::Marker::ADD;

      // 各photosensor frame原点に表示
      m.pose.position.x = 0.0;
      m.pose.position.y = 0.0;
      m.pose.position.z = 0.0;
      m.pose.orientation.w = 1.0;

      m.scale.x = sx_;
      m.scale.y = sy_;
      m.scale.z = sz_;

      // HSV
      float h = 120.0f;   // 0～360°
      float s = static_cast<float>(normalize(values[i]));
      float v = 1.0f;
      // const float g = static_cast<float>(normalize(values[i]));

      cv::Mat hsv(1, 1, CV_8UC3);
      hsv.at<cv::Vec3b>(0,0) = cv::Vec3b(
      static_cast<uchar>(h / 2),
      static_cast<uchar>(s * 255),
      static_cast<uchar>(v * 255));

      cv::Mat rgb;
      cv::cvtColor(hsv, rgb, cv::COLOR_HSV2RGB);
      cv::Vec3b c = rgb.at<cv::Vec3b>(0,0);

      m.color.r = c[0] / 255.0f;
      m.color.g = c[1] / 255.0f;
      m.color.b = c[2] / 255.0f;
      m.color.a = 1.0f;

      // 少しだけ寿命を持たせる（更新が止まったら消える）
      m.lifetime = rclcpp::Duration::from_seconds(0.3);

      arr.markers.push_back(std::move(m));
    }

    pub_->publish(arr);
  }

  rclcpp::Subscription<tatto_ros2_msgs::msg::SensorArray>::SharedPtr sub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_;

  std::string input_topic_;
  std::string output_topic_;
  double vmin_, vmax_;
  double sx_, sy_, sz_;
  int sensor_index_offset_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SensorArrayToPhotosensorMarkersNode>());
  rclcpp::shutdown();
  return 0;
}