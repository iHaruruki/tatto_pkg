# Tattto
## Setup
```shell
cd ~/ros2_ws/src
git clone ttps://github.com/iHaruruki/tatto_pkg.git
cd ~/ros2_ws
colcon build --packages-select tatto_pkg
```
## How to use
**Startup Lucia**
1. Power on Lucia and NUC21
2. Power on [Lucia-04-Green-01-Main]
3. (Wi-Fi settings) Connect to [lucia-g-router]
4. Release the emergency stop button
5. Switch Lucia's mode to [Remote Movement] (`remote`モードに切り替える)
6. Launch ROS2 Node<bar>

**Start ROS2 Node**
Run motor and encoder nodes
```shell
ros2 launch lucia_controller bringup.launch.py 
```
**Run joystick controller node**
```shell

```
**Run Spina node**
```shell
sudo chmod 777 /dev/ttyUSB0
```
うまくいかない場合

**Run Tatto node**
```shell
ros2 run tatto_pkg tatto_serial_node
```
**ros2 bag (センサの値のログを残す)**
```shell
cd ~/ros2_ws/ros2_bag
ros2 bag record -a --compression-mode file --compression-format zstd
```
Option
```
# サイズで分割（例: 2GB ごと）
ros2 bag record -a -b 2147483648 \
  --compression-mode file --compression-format zstd

# 時間で分割（例: 15分ごと）
ros2 bag record -a -d 900 \
  --compression-mode file --compression-format zstd
```
