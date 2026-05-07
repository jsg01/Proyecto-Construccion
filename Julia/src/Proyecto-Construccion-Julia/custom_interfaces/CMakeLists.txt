cmake_minimum_required(VERSION 3.8)
project(custom_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)

# custom messages
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/CustomColorDetector.srv"
  "srv/AdjustRobotSpeed.srv"
  "srv/MoveJoints.srv"
  "srv/MoveLinear.srv"
  "srv/MovePreset.srv"

  DEPENDENCIES sensor_msgs geometry_msgs std_msgs
)

ament_package()