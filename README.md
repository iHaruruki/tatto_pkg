# tatto_pkg
## 🚀 Overview
This is Tatto repository.

## 🧩 Nodes & Topics
![rosgraph](media/rosgraph.png)

## 🛠️ Setup
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
### Launch Tatto / 起動する
Changes the permissions on the device file.
```bash
sudo chmod 666 /dev/ttyUSB0
```
Run serial connection / シリアル通信を開始
```bash
ros2 run tatto_pkg tatto_serial_node
```
Run display / ディスプレイに表示
```bash
ros2 run tatto_pkg tatto_display_node
```
topic echo / センサの値を見る
```bash
ros2 topic echo /sensor_values
```
Sensor placement / センサの配置位置
<img src="media/IMG_3870.jpg" alt="sensor placement" style="width:30%;height:auto;">

### Record sensor values / 録画する
ros2 bag record / センサの値を録画する
```bash
ros2 bag record -a
```
ros2 bag play / 録画したものを再生する
```bash
cd ~/ros2_ws/rosbag
# ros2 bag play <file name　ここにディレクトリのパスを書く>
ros2 bag play $HOME/ros2_ws/rosbag/rosbag2_2025_11_10-17_46_24/
```
topic echo / 録画したセンサの値を見る
```bash
ros2 topic echo /sensor_values
```
Run display / 録画した値をディスプレイに表示
```bash
ros2 run tatto_pkg tatto_display_node
```
