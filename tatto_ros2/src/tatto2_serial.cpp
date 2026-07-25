#include <rclcpp/rclcpp.hpp>
#include <tatto_ros2_msgs/msg/sensor_array.hpp>
#include <chrono>
#include <vector>
#include <string>
#include <cstring>
#include <fcntl.h>
#include <errno.h>
#include <termios.h>
#include <unistd.h>
#include <sys/ioctl.h>

