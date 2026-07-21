#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>
#include <vector>
#include <stdexcept>

#include <rclcpp/rclcpp.hpp>
#include <tatto_ros2_msgs/msg/sensor_array.hpp>

class SensorNormalizerNode : public rclcpp::Node
{
public:
  SensorNormalizerNode()
  : Node("sensor_normalizer_node")
  {
    sensor_mins_ = this->declare_parameter<std::vector<double>>(
      "sensor_mins", std::vector<double>{310, 580, 545, 510, 575, 535, 490, 0, 565});
    sensor_maxs_ = this->declare_parameter<std::vector<double>>(
      "sensor_maxs", std::vector<double>{1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023});

    if (sensor_mins_.size() != sensor_maxs_.size()) {
      RCLCPP_FATAL(this->get_logger(),
        "sensor_mins and sensor_maxs size mismatch: %zu vs %zu",
        sensor_mins_.size(), sensor_maxs_.size());
      throw std::runtime_error("parameter size mismatch");
    }

    sub_ = this->create_subscription<tatto_ros2_msgs::msg::SensorArray>(
      "/tatto/sensor_values", 10,
      std::bind(&SensorNormalizerNode::topic_callback, this, std::placeholders::_1));

    pub_ = this->create_publisher<tatto_ros2_msgs::msg::SensorArray>(
      "/tatto/sensor_values_normalized", 10);

    RCLCPP_INFO(this->get_logger(),
      "sensor_normalizer_node started. channels=%zu", sensor_mins_.size());
  }

private:
  void topic_callback(const tatto_ros2_msgs::msg::SensorArray::SharedPtr msg)
  {
    // std_msgs/Header header
    // std_msgs/Float32MultiArray data

    if (msg->data.data.empty()) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "Received empty data.");
      return;
    }

    if (msg->data.data.size() != sensor_mins_.size()) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
        "Input size (%zu) != parameter size (%zu). Skip.",
        msg->data.data.size(), sensor_mins_.size());
      return;
    }

    tatto_ros2_msgs::msg::SensorArray out;
    out.header = msg->header;        // keep timestamp/frame
    out.data.layout = msg->data.layout;
    out.data.data.resize(msg->data.data.size());

    for (size_t i = 0; i < msg->data.data.size(); ++i) {
      const float x = msg->data.data[i];
      const float min_v = static_cast<float>(sensor_mins_[i]);
      const float max_v = static_cast<float>(sensor_maxs_[i]);
      const float denom = max_v - min_v;

      float norm = 0.0f;
      if (std::abs(denom) > std::numeric_limits<float>::epsilon()) {
        norm = (x - min_v) / denom;
      }

      out.data.data[i] = std::clamp(norm, 0.0f, 1.0f);
    }

    pub_->publish(out);
  }

  rclcpp::Subscription<tatto_ros2_msgs::msg::SensorArray>::SharedPtr sub_;
  rclcpp::Publisher<tatto_ros2_msgs::msg::SensorArray>::SharedPtr pub_;

  std::vector<double> sensor_mins_;
  std::vector<double> sensor_maxs_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SensorNormalizerNode>());
  rclcpp::shutdown();
  return 0;
}