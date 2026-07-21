#include <rclcpp/rclcpp.hpp>
#include <tatto_ros2_msgs/msg/sensor_array.hpp>
#include <opencv2/opencv.hpp>
#include <vector>
#include <algorithm>

using std::placeholders::_1;

class SensorDisplayNode : public rclcpp::Node {
public:
  SensorDisplayNode()
  : Node("sensor_display_node"),
    image_side_(1000)
  {
    this->declare_parameter<int>("image_side", 1000);
    image_side_ = this->get_parameter("image_side").as_int();

    img_ = cv::Mat(image_side_, image_side_, CV_8UC3, cv::Scalar(255,255,255));
    cv::namedWindow("Display Node", cv::WINDOW_NORMAL);   // ← リサイズ可能
    cv::resizeWindow("Display Node", image_side_, image_side_); // 初期サイズ

    sub_vals_ = this->create_subscription<tatto_ros2_msgs::msg::SensorArray>(
      "/tatto/sensor_values", 10, std::bind(&SensorDisplayNode::on_values, this, _1));
  }

  ~SensorDisplayNode() override {
    cv::destroyAllWindows();
  }

private:
  void on_values(const tatto_ros2_msgs::msg::SensorArray::SharedPtr msg) {

    if (msg->data.data.size() != 9) return;

    const auto msg_time = rclcpp::Time(msg->header.stamp);
    const std::vector<float> &bset_s = msg->data.data;

    const auto current_time = this->get_clock()->now();
    const auto time_lag = (current_time - msg_time).seconds();
    RCLCPP_DEBUG_STREAM(this->get_logger(),
        "stamp: " << msg->header.stamp.sec << "." << msg->header.stamp.nanosec
        << ", frame_id: " << msg->header.frame_id
        << ", lag: " << time_lag << " s");

    if (time_lag > 0.1) return;  // 100 ms

    const float low = 30.0f;
    const float high = 2000.0f;
    bool above_border = true;
    for (int k = 0; k < 9; ++k) {
      if (bset_s[k] <= low || bset_s[k] > high) {
        above_border = false;
        break;
      }
    }
    if (!above_border) return;

    const float fixed_min = 0.0f;
    const float scor = 1000.0f;
    std::vector<float> s(9, 0.0f);

    for (int k = 0; k < 9; ++k) {
      float mh = bset_s[k] - fixed_min;
      float denom = std::max(1.0f, scor - fixed_min);
      float normalized = mh / denom;
      s[k] = std::clamp(normalized, 0.0f, 1.0f);
    }

    // ch7 special (index 7)
    {
      float mh = bset_s[7] - fixed_min;
      float denom = std::max(1.0f, 250.0f - fixed_min);
      float normalized = mh / denom;
      s[7] = std::clamp(normalized, 0.0f, 1.0f);
      s[7] = 0.5f;
    }

    // ch0 boost
    s[0] += 0.2f;
    if (s[0] > 1.0f) s[0] = 1.0f;

    img_ = cv::Scalar(255,255,255);
    const float sc = 0.7071f;
    const int ab = image_side_ / 2;
    const int image_scale = image_side_ / 500;

    std::vector<cv::Point> positions = {
      {static_cast<int>(-125*image_scale*sc+ab), static_cast<int>( 125*image_scale*sc+ab)},
      { 250*image_scale, 125*image_scale},
      {static_cast<int>( 125*image_scale*sc+ab), static_cast<int>( 125*image_scale*sc+ab)},

      { 125*image_scale, 250*image_scale},
      { 250*image_scale, 250*image_scale},
      { 375*image_scale, 250*image_scale},

      {static_cast<int>(-125*image_scale*sc+ab), static_cast<int>(-125*image_scale*sc+ab)},
      { 250*image_scale, 375*image_scale},
      {static_cast<int>( 125*image_scale*sc+ab), static_cast<int>(-125*image_scale*sc+ab)}
    };

    for (int k = 0; k < 9; ++k) {
      int radius = static_cast<int>(15*image_scale + (65*image_scale * s[k]));
      cv::circle(img_, positions[k], radius, cv::Scalar(255, 0, 0), 2);
    }

    float sumw = 0.f, wx = 0.f, wy = 0.f;
    for (int k = 0; k < 9; ++k) {
      wx += s[k] * positions[k].x;
      wy += s[k] * positions[k].y;
      sumw += s[k];
    }

    int cx = 250*image_scale, cy = 250*image_scale;
    if (sumw > 0.f) {
      cx = static_cast<int>(wx / sumw);
      cy = static_cast<int>(wy / sumw);
    }

    cv::circle(img_, {(cx-ab)*6+ab, (cy-ab)*6+ab}, 10, cv::Scalar(0,0,255), -1);

    cv::imshow("Display Node", img_);
    cv::waitKey(1);
  }

  rclcpp::Subscription<tatto_ros2_msgs::msg::SensorArray>::SharedPtr sub_vals_;
  int image_side_;
  cv::Mat img_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SensorDisplayNode>());
  rclcpp::shutdown();
  return 0;
}