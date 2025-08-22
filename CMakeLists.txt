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

target_link_libraries(tatto_serial_node ${OpenCV_LIBS})

install(TARGETS
  tatto_serial_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
