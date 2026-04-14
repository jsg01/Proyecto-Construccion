cmake_minimum_required(VERSION 3.8)
project(robot_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs  REQUIRED)
find_package(custom_interfaces  REQUIRED)
find_package(std_srvs REQUIRED)

add_executable(robot_controller src/robot_controller.cpp)

target_include_directories(robot_controller PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

target_compile_features(robot_controller PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

ament_target_dependencies(robot_controller
  moveit_ros_planning_interface
  rclcpp
  geometry_msgs
  std_srvs
  custom_interfaces
)

install(TARGETS robot_controller
DESTINATION lib/${PROJECT_NAME})




ament_package()
