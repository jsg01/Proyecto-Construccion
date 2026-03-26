cmake_minimum_required(VERSION 3.8)
project(robot_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(tf2 REQUIRED)


add_executable(move_to_point src/move_to_point.cpp)

ament_target_dependencies(move_to_point
  rclcpp
  geometry_msgs
  moveit_ros_planning_interface
)
add_executable(move_pilz src/move_pilz.cpp)

ament_target_dependencies(move_pilz
  rclcpp
  geometry_msgs
  moveit_ros_planning_interface
)


add_executable(pick_place src/pick_place.cpp)

ament_target_dependencies(pick_place
  rclcpp
  geometry_msgs
  moveit_ros_planning_interface
)

add_executable(nodo90grados src/nodo90grados.cpp)

ament_target_dependencies(nodo90grados
  rclcpp
  geometry_msgs
  moveit_ros_planning_interface
)

add_executable(move_to_pose src/move_to_pose.cpp)

ament_target_dependencies(move_to_pose
  rclcpp
  geometry_msgs
  moveit_ros_planning_interface
  tf2
)

install(TARGETS
  move_to_point
  move_pilz
  pick_place
  nodo90grados
  move_to_pose
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
