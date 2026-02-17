#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/u_int16_multi_array.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/bool.hpp>
#include <numeric>

class SensorThresholdNode : public rclcpp::Node
{
public:
    SensorThresholdNode() : Node("sensor_threshold_node")
    {
        // パラメータ宣言
        this->declare_parameter<int>("threshold", 600);
        this->declare_parameter<int>("sensor_count", 9);
        
        // パラメータ取得
        threshold_ = this->get_parameter("threshold").as_int();
        sensor_count_ = this->get_parameter("sensor_count").as_int();
        
        // QoS設定
        auto qos = rclcpp::QoS(rclcpp::KeepLast(1))
            .reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);
        
        // サブスクライバー
        sensor_sub_ = this->create_subscription<std_msgs::msg::UInt16MultiArray>(
            "/sensor_values",
            qos,
            std::bind(&SensorThresholdNode::sensorCallback, this, std::placeholders::_1)
        );
        
        // パブリッシャー
        average_pub_ = this->create_publisher<std_msgs::msg::Float64>("/sensor_average", 10);
        threshold_pub_ = this->create_publisher<std_msgs::msg::Bool>("/sensor_threshold_exceeded", 10);
        
        RCLCPP_INFO(this->get_logger(), "sensor_threshold_node started (threshold: %d)", threshold_);
    }

private:
    void sensorCallback(const std_msgs::msg::UInt16MultiArray::SharedPtr msg)
    {
        const auto& data = msg->data;
        
        // センサー数のチェック
        if (static_cast<int>(data.size()) != sensor_count_) {
            return;
        }
        
        // 平均値を計算
        double average = std::accumulate(data.begin(), data.end(), 0.0) / data.size();
        
        // しきい値判定
        bool exceeded = average >= threshold_;
        
        // 平均値をパブリッシュ
        auto avg_msg = std_msgs::msg::Float64();
        avg_msg.data = average;
        average_pub_->publish(avg_msg);
        
        // しきい値超過ステータスをパブリッシュ
        auto status_msg = std_msgs::msg::Bool();
        status_msg.data = exceeded;
        threshold_pub_->publish(status_msg);
    }
    
    // メンバ変数
    rclcpp::Subscription<std_msgs::msg::UInt16MultiArray>::SharedPtr sensor_sub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr average_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr threshold_pub_;
    
    int threshold_;
    int sensor_count_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SensorThresholdNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
