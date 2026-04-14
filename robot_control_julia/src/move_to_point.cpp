#include <cstdlib>
#include <memory>
#include <iostream>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <moveit/move_group_interface/move_group_interface.h>

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  if (argc != 4) {
    std::cout << "Uso: ros2 run robot_control move_to_point x y z\n";
    rclcpp::shutdown();
    return 1;
  }

  double x = std::atof(argv[1]);
  double y = std::atof(argv[2]);
  double z = std::atof(argv[3]);

  auto node = std::make_shared<rclcpp::Node>(
    "move_to_point",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
  );

  auto logger = rclcpp::get_logger("move_to_point");

  using moveit::planning_interface::MoveGroupInterface;
  MoveGroupInterface move_group(node, "ur_manipulator");

  geometry_msgs::msg::Pose target_pose;
  target_pose.orientation.w = 1.0;
  target_pose.position.x = x;
  target_pose.position.y = y;
  target_pose.position.z = z;

  move_group.setPoseTarget(target_pose);

  MoveGroupInterface::Plan plan;
  bool success = static_cast<bool>(move_group.plan(plan));

  if (success) {
    RCLCPP_INFO(logger, "Plan encontrado. Ejecutando...");
    move_group.execute(plan);
  } else {
    RCLCPP_ERROR(logger, "Fallo al planificar");
  }

  rclcpp::shutdown();
  return 0;
}
