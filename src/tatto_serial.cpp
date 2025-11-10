#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/u_int16_multi_array.hpp>
#include <chrono>
#include <vector>
#include <string>
#include <cstring>
#include <fcntl.h>
#include <errno.h>
#include <termios.h>
#include <unistd.h>
#include <sys/ioctl.h>

using namespace std::chrono_literals;

class SensorReaderNode : public rclcpp::Node {
public:
  SensorReaderNode()
  : Node("sensor_reader_node"),
    payload_size_(9 * 2),
    serial_port_(-1),
    calibrated_(false)
  {
    this->declare_parameter<std::string>("port", "/dev/ttyUSB0");
    this->declare_parameter<int>("baud", 115200);
    port_ = this->get_parameter("port").as_string();
    baud_ = this->get_parameter("baud").as_int();

    pub_raw_       = this->create_publisher<std_msgs::msg::UInt16MultiArray>("sensor_values_raw", 10);
    pub_reordered_ = this->create_publisher<std_msgs::msg::UInt16MultiArray>("sensor_values", 10);

    bset_.assign(9, 0);
    bset_prev_.assign(9, 0);
    minv_.assign(9, 0);

    if (!init_serial()) {
      RCLCPP_ERROR(get_logger(), "Serial init failed. Node will run but publish nothing.");
    }

    timer_ = this->create_wall_timer(50ms, std::bind(&SensorReaderNode::on_timer, this));
  }

  ~SensorReaderNode() override {
    if (serial_port_ >= 0) close(serial_port_);
  }

private:
  bool init_serial() {
    serial_port_ = open(port_.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (serial_port_ < 0) {
      RCLCPP_ERROR(get_logger(), "open(%s) failed: %s", port_.c_str(), strerror(errno));
      return false;
    }

    termios tty{};
    if (tcgetattr(serial_port_, &tty) != 0) {
      RCLCPP_ERROR(get_logger(), "tcgetattr: %s", strerror(errno));
      return false;
    }

    // ボーレート
    speed_t spd = B115200;
    if (baud_ == 9600) spd = B9600;
    else if (baud_ == 19200) spd = B19200;
    else if (baud_ == 57600) spd = B57600;
    else spd = B115200;
    cfsetospeed(&tty, spd);
    cfsetispeed(&tty, spd);

    tty.c_cflag &= ~PARENB;
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;
    tty.c_cflag &= ~CRTSCTS;
    tty.c_cflag |= CREAD | CLOCAL;

    tty.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    tty.c_iflag &= ~(IXON | IXOFF | IXANY | IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL);
    tty.c_oflag &= ~(OPOST | ONLCR);

    tty.c_cc[VTIME] = 1;  // 0.1s
    tty.c_cc[VMIN]  = 0;

    if (tcsetattr(serial_port_, TCSANOW, &tty) != 0) {
      RCLCPP_ERROR(get_logger(), "tcsetattr: %s", strerror(errno));
      return false;
    }

    RCLCPP_INFO(get_logger(), "Serial opened: %s @ %d", port_.c_str(), baud_);
    return true;
  }

  void on_timer() {
    if (serial_port_ < 0) return;

    int byteswaiting = 0;
    ioctl(serial_port_, FIONREAD, &byteswaiting);

    const int min_packet_len = 2 + payload_size_ + 2; // 0xFF 0xFF + 18 + 0xFF 0xFF
    if (byteswaiting < min_packet_len) return;

    std::vector<uint8_t> buf(byteswaiting + 1, 0);
    ssize_t r = read(serial_port_, buf.data(), buf.size());
    if (r <= 0) return;

    int head_idx = -1;
    for (size_t k = 0; k + min_packet_len <= buf.size(); ++k) {
      if (buf[k] == 0xFF && buf[k+1] == 0xFF) {
        size_t tail_pos = k + 2 + payload_size_;
        if (tail_pos + 1 < buf.size() && buf[tail_pos] == 0xFF && buf[tail_pos+1] == 0xFF) {
          head_idx = static_cast<int>(k + 2);
          break;
        }
      }
    }
    if (head_idx < 0) return;

    // シフト
    for (int i = 0; i < 9; ++i) bset_prev_[i] = bset_[i];

    // 9chデコード（Big-endian）
    for (int i = 0; i < 9; ++i) {
      uint16_t hi = buf[head_idx + 2*i];
      uint16_t lo = buf[head_idx + 2*i + 1];
      bset_[i] = static_cast<uint16_t>((hi << 8) | lo);
    }

    // Publish: raw
    {
      std_msgs::msg::UInt16MultiArray msg;
      msg.data.assign(bset_.begin(), bset_.end());
      pub_raw_->publish(msg);
    }

    // 初回キャリブレーション
    if (!calibrated_) {
      bool all_set = true;
      for (int i = 0; i < 9; ++i) {
        if (minv_[i] == 0) {
          if (bset_[i] != 0 && (int)bset_[i] - (int)bset_prev_[i] >= 0) {
            minv_[i] = bset_[i];
          } else {
            all_set = false;
          }
        }
      }
      if (all_set) {
        calibrated_ = true;
        RCLCPP_INFO(get_logger(), "Calibration complete for 9 sensors.");
      }
    }

    // 並べ替え
    // センサのレイアウトに合わせて並べ替え
    //前：bset_ = [A0, A1, A2, A3, A4, A5, A6, A7, A8]
    //並べ替え後：bset_s = [A5, A2, A7, A6, A3, A8, A0, A4, A1]
    std::vector<uint16_t> bset_s(9);
    bset_s[0] = bset_[5];
    bset_s[1] = bset_[2];
    bset_s[2] = bset_[7];
    bset_s[3] = bset_[6];
    bset_s[4] = bset_[3];
    bset_s[5] = bset_[8];
    bset_s[6] = bset_[0];
    bset_s[7] = bset_[4];
    bset_s[8] = bset_[1];

    // Publish: 並び替え後
    {
      std_msgs::msg::UInt16MultiArray msg;
      msg.data.assign(bset_s.begin(), bset_s.end());
      pub_reordered_->publish(msg);
    }

  }

  // Params
  std::string port_;
  int baud_;

  // Serial
  int serial_port_;
  const int payload_size_;

  // State
  std::vector<uint16_t> bset_, bset_prev_;
  std::vector<uint16_t> minv_;
  bool calibrated_;
  rclcpp::TimerBase::SharedPtr timer_;

  // Publishers
  rclcpp::Publisher<std_msgs::msg::UInt16MultiArray>::SharedPtr pub_raw_;
  rclcpp::Publisher<std_msgs::msg::UInt16MultiArray>::SharedPtr pub_reordered_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SensorReaderNode>());
  rclcpp::shutdown();
  return 0;
}
