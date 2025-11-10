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
Run serial connection
```bash
ros2 run tatto_pkg tatto_serial_node
```
Run display 
```bash
ros2 run tatto_pkg tatto_display_node
```
