# tatto_pkg
## Setup
Get `tatto_pkg` package.
```bash
cd ~/ros2_ws/src
git clone https://github.com/iHaruruki/tatto_pkg.git
```
Build
```bash
cd ~/ros2_ws
colcon build --packages-select tatto_pkg
```
## How to use
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
ros2 bag record / センサの値を録画する
```bash
ros2 bag record -a
```
ros2 bag play / 録画したものを再生する
```bash
# ros2 bag play <file name>
ros2 bag play 
```
topic echo / 録画したセンサの値を見る
```bash
ros2 topic echo /sensor_values
```
Run display / 録画した値をディスプレイに表示
```bash
ros2 run tatto_pkg tatto_display_node
```
