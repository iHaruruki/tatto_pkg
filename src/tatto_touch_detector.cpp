#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/u_int16_multi_array.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/bool.hpp>
#include <numeric>
#include <algorithm>

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
        
        RCLCPP_INFO(this->get_logger(), "sensor_threshold_node を起動しました。");
        RCLCPP_INFO(this->get_logger(), "しきい値: %d, ��ンサー数: %d", threshold_, sensor_count_);
    }

private:
    void sensorCallback(const std_msgs::msg::UInt16MultiArray::SharedPtr msg)
    {
        const auto& data = msg->data;
        
        // センサー数のチェック
        if (static_cast<int>(data.size()) != sensor_count_) {
            RCLCPP_WARN(this->get_logger(), 
                "センサー数が一致しません。期待: %d, 受信: %zu", 
                sensor_count_, data.size());
            return;
        }
        
        // 平均値を計算
        double average = std::accumulate(data.begin(), data.end(), 0.0) / data.size();
        
        // しきい値判定
        bool exceeded = average >= threshold_;
        
        // 統計更新
        total_count_++;
        if (exceeded) {
            exceed_count_++;
        }
        
        // 最小値・最大値を計算
        uint16_t min_val = *std::min_element(data.begin(), data.end());
        uint16_t max_val = *std::max_element(data.begin(), data.end());
        
        // 平均値をパブリッシュ
        auto avg_msg = std_msgs::msg::Float64();
        avg_msg.data = average;
        average_pub_->publish(avg_msg);
        
        // しきい値超過ステータスをパブリッシュ
        auto status_msg = std_msgs::msg::Bool();
        status_msg.data = exceeded;
        threshold_pub_->publish(status_msg);
        
        // ログ出力（しきい値超過時のみ）
        if (exceeded) {
            double exceed_rate = (total_count_ > 0) 
                ? static_cast<double>(exceed_count_) / total_count_ * 100.0
                : 0.0;
            
            RCLCPP_INFO(this->get_logger(), 
                "しきい値超過！ 平均: %.2f >= %d (Min: %d, Max: %d) [超過率: %.1f%% (%d/%d)]",
                average, threshold_, min_val, max_val, exceed_rate, exceed_count_, total_count_);
        } else {
            RCLCPP_DEBUG(this->get_logger(), 
                "正常範囲: 平均: %.2f < %d (Min: %d, Max: %d)", 
                average, threshold_, min_val, max_val);
        }
    }
    
    // メンバ変数
    rclcpp::Subscription<std_msgs::msg::UInt16MultiArray>::SharedPtr sensor_sub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr average_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr threshold_pub_;
    
    int threshold_;
    int sensor_count_;
    int total_count_ = 0;
    int exceed_count_ = 0;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SensorThresholdNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
