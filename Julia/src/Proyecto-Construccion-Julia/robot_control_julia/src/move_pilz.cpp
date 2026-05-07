#include <cstdlib>
#include <memory>
#include <iostream>
#include <vector>
#include <string>
#include <thread>
#include <chrono>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/executors/single_threaded_executor.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <moveit/move_group_interface/move_group_interface.h>

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  std::vector<std::string> args = rclcpp::remove_ros_arguments(argc, argv);

  if (args.size() != 4) {
    std::cout << "Uso: ros2 run robot_control move_pilz x y z\n";
    rclcpp::shutdown();
    return 1;
  }

  const double x = std::atof(args[1].c_str());
  const double y = std::atof(args[2].c_str());
  const double z = std::atof(args[3].c_str());

  auto node = std::make_shared<rclcpp::Node>(
    "move_pilz",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
  );

  // Executor para que funcionen bien las subscripciones y callbacks de MoveIt
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  std::thread spinner([&executor]() { executor.spin(); });

  auto logger = rclcpp::get_logger("move_pilz");

  RCLCPP_INFO(logger, "========================================");
  RCLCPP_INFO(logger, "Nodo move_pilz arrancado");
  RCLCPP_INFO(logger, "Punto recibido: x=%.3f  y=%.3f  z=%.3f", x, y, z);

  using moveit::planning_interface::MoveGroupInterface;
  MoveGroupInterface move_group(node, "ur_manipulator");

  RCLCPP_INFO(logger, "MoveGroupInterface creado para grupo: ur_manipulator");

  move_group.setMaxVelocityScalingFactor(0.1);
  move_group.setMaxAccelerationScalingFactor(0.1);

  RCLCPP_INFO(logger, "Velocity scaling: 0.1");
  RCLCPP_INFO(logger, "Acceleration scaling: 0.1");

  geometry_msgs::msg::Pose pre_pose;
  pre_pose.position.x = x;
  pre_pose.position.y = y;
  pre_pose.position.z = z + 0.10;

  pre_pose.orientation.x = 0.0;
  pre_pose.orientation.y = 1.0;
  pre_pose.orientation.z = 0.0;
  pre_pose.orientation.w = 0.0;

  geometry_msgs::msg::Pose target_pose = pre_pose;
  target_pose.position.z = z;

  RCLCPP_INFO(logger, "Prepose:");
  RCLCPP_INFO(
    logger,
    "  pos=(%.3f, %.3f, %.3f)  ori=(%.3f, %.3f, %.3f, %.3f)",
    pre_pose.position.x, pre_pose.position.y, pre_pose.position.z,
    pre_pose.orientation.x, pre_pose.orientation.y,
    pre_pose.orientation.z, pre_pose.orientation.w
  );

  RCLCPP_INFO(logger, "Target pose:");
  RCLCPP_INFO(
    logger,
    "  pos=(%.3f, %.3f, %.3f)  ori=(%.3f, %.3f, %.3f, %.3f)",
    target_pose.position.x, target_pose.position.y, target_pose.position.z,
    target_pose.orientation.x, target_pose.orientation.y,
    target_pose.orientation.z, target_pose.orientation.w
  );

  // -------- PTP a prepose --------
  RCLCPP_INFO(logger, "----------------------------------------");
  RCLCPP_INFO(logger, "FASE 1: Movimiento PTP a prepose");

  move_group.clearPoseTargets();
  move_group.setStartStateToCurrentState();
  move_group.setPlannerId("PTP");
  move_group.setPoseTarget(pre_pose);

  RCLCPP_INFO(logger, "Planner seleccionado: PTP");
  RCLCPP_INFO(logger, "Planificando PTP...");

  MoveGroupInterface::Plan ptp_plan;
  bool ptp_success = static_cast<bool>(move_group.plan(ptp_plan));

  if (!ptp_success) {
    RCLCPP_ERROR(logger, "Falló la planificación PTP a la prepose");
    executor.cancel();
    if (spinner.joinable()) spinner.join();
    rclcpp::shutdown();
    return 1;
  }

  RCLCPP_INFO(logger, "Plan PTP encontrado correctamente");
  RCLCPP_INFO(logger, "Ejecutando movimiento PTP...");

  if (!static_cast<bool>(move_group.execute(ptp_plan))) {
    RCLCPP_ERROR(logger, "Falló la ejecución del movimiento PTP");
    executor.cancel();
    if (spinner.joinable()) spinner.join();
    rclcpp::shutdown();
    return 1;
  }

  RCLCPP_INFO(logger, "Movimiento PTP completado");
  RCLCPP_INFO(logger, "Esperando 1 segundo para estabilizar el estado del robot...");
  std::this_thread::sleep_for(std::chrono::seconds(1));

  // -------- LIN al objetivo --------
  RCLCPP_INFO(logger, "----------------------------------------");
  RCLCPP_INFO(logger, "FASE 2: Movimiento LIN al objetivo");

  move_group.clearPoseTargets();
  move_group.setStartStateToCurrentState();

  auto current_state = move_group.getCurrentState(2.0);
  if (!current_state) {
    RCLCPP_ERROR(logger, "No se pudo obtener el estado actual del robot antes del LIN");
    executor.cancel();
    if (spinner.joinable()) spinner.join();
    rclcpp::shutdown();
    return 1;
  }

  RCLCPP_INFO(logger, "Estado actual leído correctamente antes del LIN");

  move_group.setPlannerId("LIN");
  move_group.setPoseTarget(target_pose);

  RCLCPP_INFO(logger, "Planner seleccionado: LIN");
  RCLCPP_INFO(logger, "Planificando LIN...");

  MoveGroupInterface::Plan lin_plan;
  bool lin_success = static_cast<bool>(move_group.plan(lin_plan));

  if (!lin_success) {
    RCLCPP_ERROR(logger, "Falló la planificación LIN al objetivo");
    executor.cancel();
    if (spinner.joinable()) spinner.join();
    rclcpp::shutdown();
    return 1;
  }

  RCLCPP_INFO(logger, "Plan LIN encontrado correctamente");
  RCLCPP_INFO(logger, "Ejecutando movimiento LIN...");

  if (!static_cast<bool>(move_group.execute(lin_plan))) {
    RCLCPP_ERROR(logger, "Falló la ejecución del movimiento LIN");
    executor.cancel();
    if (spinner.joinable()) spinner.join();
    rclcpp::shutdown();
    return 1;
  }

  RCLCPP_INFO(logger, "Movimiento LIN completado");
  RCLCPP_INFO(logger, "Movimiento completo finalizado con éxito");
  RCLCPP_INFO(logger, "========================================");

  executor.cancel();
  if (spinner.joinable()) {
    spinner.join();
  }
  rclcpp::shutdown();
  return 0;
}
