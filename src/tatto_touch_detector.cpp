#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/u_int16_multi_array.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>
#include <numeric>
#include <string>

class SensorThresholdNode : public rclcpp::Node
{
public:
    SensorThresholdNode() : Node("sensor_threshold_node"), 
                            prev_exceeded_(false),
                            last_tts_time_(this->now() - rclcpp::Duration(15, 0))  // 起動時は10秒前に初期化
    {
        // パラメータ宣言
        this->declare_parameter<int>("threshold", 800);
        this->declare_parameter<int>("sensor_count", 9);
        this->declare_parameter<double>("tts_interval", 8.0);  // TTS送信間隔（秒）
        
        // パラメータ取得
        threshold_ = this->get_parameter("threshold").as_int();
        sensor_count_ = this->get_parameter("sensor_count").as_int();
        tts_interval_ = this->get_parameter("tts_interval").as_double();
        
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
        tts_pub_ = this->create_publisher<std_msgs::msg::String>("/voicevox_tts_text", 30);
        
        RCLCPP_INFO(this->get_logger(), 
                    "sensor_threshold_node started (threshold: %d, tts_interval: %.1f sec)", 
                    threshold_, tts_interval_);
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
        bool exceeded = (average >= threshold_);
        
        // しきい値を超えた瞬間だけTTSを送信
        if(exceeded && !prev_exceeded_)
        {
            // 前回のTTS送信から指定秒数以上経過しているかチェック
            rclcpp::Time current_time = this->now();
            rclcpp::Duration elapsed = current_time - last_tts_time_;
            
            if (elapsed.seconds() >= tts_interval_)
            {
                std::string speech_text = "タットを触ってくれてありがとう！";

                // Publish tts text
                auto tts_msg = std_msgs::msg::String();
                tts_msg.data = speech_text;
                tts_pub_->publish(tts_msg);
                
                // 送信時刻を更新
                last_tts_time_ = current_time;
                
                RCLCPP_INFO(this->get_logger(), 
                           "Threshold exceeded! TTS sent. (average: %.2f)", average);
            }
            else
            {
                RCLCPP_DEBUG(this->get_logger(), 
                            "TTS skipped (%.1f sec since last TTS)", elapsed.seconds());
            }
        }
        
        // 前回の状態を更新
        prev_exceeded_ = exceeded;
        
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
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr tts_pub_;
    
    int threshold_;
    int sensor_count_;
    double tts_interval_;           // TTS送信間隔（秒）
    bool prev_exceeded_;            // 前回のしきい値超過状態
    rclcpp::Time last_tts_time_;    // 最後にTTSを送信した時刻
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SensorThresholdNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}