cmake_minimum_required(VERSION 3.8)
project(robot_control_julia)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(tf2 REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(shape_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(construccion_interfaces REQUIRED)


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

add_executable(move_to_point_ok src/move_to_point_ok.cpp)

ament_target_dependencies(move_to_point_ok
  rclcpp
  geometry_msgs
  moveit_ros_planning_interface
)

add_executable(move_to_point_ok_orientacion src/move_to_point_ok_orientacion.cpp)

ament_target_dependencies(move_to_point_ok_orientacion
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



add_executable(movimiento_robot src/movimiento_robot.cpp)

ament_target_dependencies(movimiento_robot
  rclcpp
  moveit_ros_planning_interface
  geometry_msgs
  moveit_msgs
  shape_msgs
  std_msgs
  tf2
  tf2_geometry_msgs
  construccion_interfaces
)

install(TARGETS
  move_to_point
  move_pilz
  pick_place
  nodo90grados
  move_to_pose
  move_to_point_ok
  move_to_point_ok_orientacion
  movimiento_robot
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
