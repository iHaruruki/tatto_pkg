#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/u_int16_multi_array.hpp>
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

    // 並び替え済みの sensor_values のみ購読
    sub_vals_ = this->create_subscription<std_msgs::msg::UInt16MultiArray>(
      "sensor_values", 10, std::bind(&SensorDisplayNode::on_values, this, _1));

    cv::namedWindow("img", cv::WINDOW_AUTOSIZE);
  }

  ~SensorDisplayNode() override {
    cv::destroyAllWindows();
  }

private:
  void on_values(const std_msgs::msg::UInt16MultiArray::SharedPtr msg) {
    if (msg->data.size() != 9) return;
    const std::vector<uint16_t> &bset_s = msg->data;

    // 全センサが閾値 border より大きいときだけ描画（元コード準拠）
    const int border = 30;
    bool above_border = true;
    for (int k = 0; k < 9; ++k) {
      if (static_cast<int>(bset_s[k]) <= border) { above_border = false; break; }
    }
    if (!above_border) return;

    // --- 固定の最小/最大で正規化（要求どおり） ---
    // 最小は 0、通常の最大 (scor) = 1000、ただし ch7 (index 7) は最大 250 の特例
    const float fixed_min = 0.0f;
    const float scor = 1000.0f;
    std::vector<float> s(9, 0.0f);

    for (int k = 0; k < 9; ++k) {
      float mh  = static_cast<float>(bset_s[k]) - fixed_min;
      float denom = std::max(1.0f, scor - fixed_min);
      float normalized = mh / denom;
      s[k] = std::clamp(normalized, 0.0f, 1.0f);
    }
    // ch7 の特例（index 7）
    {
      float mh  = static_cast<float>(bset_s[7]) - fixed_min;
      float denom = std::max(1.0f, 250.0f - fixed_min);
      float normalized = mh / denom;
      s[7] = std::clamp(normalized, 0.0f, 1.0f);
    }
    // -----------------------------------------------

    // 描画
    img_ = cv::Scalar(255,255,255);
    const float sc = 0.7071f;
    const int ab   = image_side_ / 2;
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
      int radius = static_cast<int>(35*image_scale + (65*image_scale * s[k]));
      cv::circle(img_, positions[k], radius, cv::Scalar(255, 0, 0), 2);
    }

    // 重心
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

    cv::circle(img_, {250*image_scale,250*image_scale}, 150*image_scale, cv::Scalar(255,0,0), 3);
    cv::circle(img_, {250*image_scale,250*image_scale},  75*image_scale, cv::Scalar(100,100,0), 3);
    cv::circle(img_, {cx, cy}, 10, cv::Scalar(0,0,255), -1);

    cv::imshow("img", img_);
    cv::waitKey(1);
  }

  rclcpp::Subscription<std_msgs::msg::UInt16MultiArray>::SharedPtr sub_vals_;

  int image_side_;
  cv::Mat img_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SensorDisplayNode>());
  rclcpp::shutdown();
  return 0;
}
