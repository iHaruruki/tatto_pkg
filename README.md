# tatto_pkg
[![ROS 2 Distro - Humble](https://img.shields.io/badge/ros2-Humble-blue)](https://docs.ros.org/en/humble/)
[![ROS2 Distro - Jazzy](https://img.shields.io/badge/ros2-Jazzy-blue)](https://docs.ros.org/en/jazzy/)

## 🚀 Overview
This is Tatto repository.

## 🧩 Nodes & Topics
<!-- ![rosgraph](media/rosgraph.png) -->

```mermaid
flowchart LR
    %% Nodes
    S([/sensor_reader_node])
    D([/sensor_display_node])

    %% Topics
    V["/sensor_values<br/>(std_msgs::msg::UInt16MultiArray)"]

    S --> V
    V --> D
```

## 🛠️ Setup
> [!NOTE]
> Is your ROS2 environment ready?  
> [ros2 install](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html)


Get `tatto_pkg` package.
```bash
cd ~/ros2_ws/src
git clone https://github.com/iHaruruki/tatto_pkg.git
```
Build
```bash
cd ~/ros2_ws
colcon build --packages-select tatto_pkg
source install/setup.bash
```
## 🎮 How to use
### Launch Tatto / Tattoを起動する
Changes the permissions on the device file.
```bash
sudo chmod 666 /dev/ttyUSB0
```
Run `sensor_reader_node` / シリアル通信を開始
```bash
ros2 run tatto_pkg tatto_serial_node
```
Run `sensor_display_node` / ディスプレイに表示
```bash
ros2 run tatto_pkg tatto_display_node
```
topic echo / センサの値を見る
```bash
ros2 topic echo /sensor_values
```
Sensor placement / センサの配置位置  
<img src="media/IMG_3870.jpg" alt="sensor placement" style="width:30%;height:auto;">

### Record sensor values / センサデータを記録する
ros2 bag record / センサの値を記録する
```bash
cd ~/ros2_ws/rosbag
ros2 bag record -a
```
ros2 bag play / 記録したものを再生する
```bash
# ros2 bag play <file name　ここにディレクトリのパスを書く>
ros2 bag play $HOME/ros2_ws/rosbag/rosbag2_2025_11_10-17_46_24/
```
topic echo / センサの値を見る
```bash
ros2 topic echo /sensor_values
```
Run `sensor_display_node` / ディスプレイに表示
```bash
ros2 run tatto_pkg tatto_display_node
```


## :ghost: ROS 2 のプログラムを書いてみる！ 
### Node & Topic / NodeとTopicとは？
- What is Node? / Nodeとは？  
Role: The smallest unit of a ROS 2 application (an object within a process). Each node handles a single responsibility such as sensor reading, inference, or control.  
役割：ROS 2 アプリの最小単位（プロセス内のオブジェクト）．センサ読み取り，推論，制御などの単機能を担当．  
Official documentation: [Understanding nodes](https://docs.ros.org/en/jazzy/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Nodes/Understanding-ROS2-Nodes.html)

- What is Topic? / Topicとは？  
Role: A communication channel for stream-type messages between nodes in a loosely coupled fashion.  
役割：ノード間で非同期で情報のやりとりを行うメッセージ  
Official documentation: [Understanding topics](https://docs.ros.org/en/jazzy/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Topics/Understanding-ROS2-Topics.html)

### tattoのプログラムを書いてみる
#### `/sensor_values` topicを受信して，topicの値を表示するプログラムを作成する
- ソースコードは`src`ディレクトリ内に置く  
`subscribe_topic.cpp`
```cpp
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/u_int16_multi_array.hpp>
#include <vector>
#include <iostream>

using namespace std::chrono_literals;

class SensorDisplayNode : public rclcpp::Node {
public:
  SensorDisplayNode()
  : Node("sensor_display_node")
  {
    // Subscribe to the /sensor_values topic to receive data
    sub_ = this->create_subscription<std_msgs::msg::UInt16MultiArray>(
      "/sensor_values", 10,
      std::bind(&SensorDisplayNode::on_sensor_values_received, this, std::placeholders::_1));
  }

private:
  // Callback function to process the received sensor values
  void on_sensor_values_received(const std_msgs::msg::UInt16MultiArray::SharedPtr msg)
  {
    // Print each value from the received message
    std::cout << "Received sensor values: ";
    for (const auto& value : msg->data) {
      std::cout << value << " ";
    }
    std::cout << std::endl;
  }

  // Subscriber to /sensor_values topic
  rclcpp::Subscription<std_msgs::msg::UInt16MultiArray>::SharedPtr sub_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SensorDisplayNode>());
  rclcpp::shutdown();
  return 0;
}
```
- ビルドのためにCMakeLists.txtファイルを編集する  
`CMakeLists.txt`
```cmake
cmake_minimum_required(VERSION 3.8)
project(tatto_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(OpenCV REQUIRED)

add_executable(tatto_serial_node src/tatto_serial.cpp)
ament_target_dependencies(tatto_serial_node
  rclcpp
  std_msgs
  geometry_msgs
)

add_executable(tatto_display_node src/tatto_display.cpp)
ament_target_dependencies(tatto_display_node
  rclcpp
  std_msgs
  geometry_msgs
)
target_link_libraries(tatto_display_node ${OpenCV_LIBS})

##### --------------新しく加える------------------------ #####
add_executable(topic_sub_node src/subscribe_topic.cpp)
ament_target_dependencies(subscribe_topic.cpp
  rclcpp
  std_msgs
  geometry_msgs
)
##### -------------------------------------------------- #####

### "install"内の`topic_sub_node`を新しく加える ###
install(TARGETS
  tatto_serial_node tatto_display_node topic_sub_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
```

- Buildする  
```bash
cd ~/ros2_ws
colcon build --packages-select tatto_pkg
source install/setup.bash
```
- 実行する  
```bash
ros2 run tatto_pkg topic_sub_node
```

## 📚 Reference
ROS 2 Official documentation
- [ROS 2-jazzy](https://docs.ros.org/en/jazzy/index.html)