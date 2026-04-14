#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <geometry_msgs/msg/pose.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/msg/collision_object.hpp>
#include <moveit_msgs/msg/robot_trajectory.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>

#include <thread>
#include <chrono>
#include <cmath>
#include <vector>

using namespace std::chrono_literals;

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);

    auto node = rclcpp::Node::make_shared("move_to_point_ok");

    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    std::thread spin_thread([&executor]() { executor.spin(); });

    std::this_thread::sleep_for(2s);

    moveit::planning_interface::MoveGroupInterface move_group(node, "ur3e");

    move_group.setMaxVelocityScalingFactor(0.10);
    move_group.setMaxAccelerationScalingFactor(0.08);
    move_group.setPlanningTime(10.0);
    move_group.setNumPlanningAttempts(10);
    move_group.allowReplanning(false);

    move_group.setPlannerId("RRTConnectkConfigDefault");
    move_group.setGoalPositionTolerance(0.005);
    move_group.setGoalOrientationTolerance(0.01);

    move_group.setStartStateToCurrentState();
    
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

    // Crear objeto mesa
    moveit_msgs::msg::CollisionObject collision_object;
    collision_object.header.frame_id = "base_link";
    collision_object.id = "mesa";

    shape_msgs::msg::SolidPrimitive primitive;
    primitive.type = primitive.BOX;
    primitive.dimensions = {1.0, 1.0, 0.05};

    geometry_msgs::msg::Pose table_pose;
    table_pose.orientation.w = 1.0;
    table_pose.position.x = 0.0;
    table_pose.position.y = 0.0;
    table_pose.position.z = -0.025;

    collision_object.primitives.push_back(primitive);
    collision_object.primitive_poses.push_back(table_pose);
    collision_object.operation = collision_object.ADD;

    planning_scene_interface.applyCollisionObject(collision_object);
    rclcpp::sleep_for(std::chrono::milliseconds(500));

    // parámetros
    double x = node->declare_parameter("x", 0.2);
    double y = node->declare_parameter("y", 0.0);
    double z = node->declare_parameter("z", 0.1);

    RCLCPP_INFO(node->get_logger(),
        "Objetivo recibido: x=%.3f y=%.3f z=%.3f", x, y, z);

    // pose objetivo
    geometry_msgs::msg::Pose target_pose;
    target_pose.position.x = x;
    target_pose.position.y = y;
    target_pose.position.z = z;

    target_pose.orientation.x = 1.0;
    target_pose.orientation.y = 0.0;
    target_pose.orientation.z = 0.0;
    target_pose.orientation.w = 0.0;

    auto current_pose = move_group.getCurrentPose().pose;

    double dx = target_pose.position.x - current_pose.position.x;
    double dy = target_pose.position.y - current_pose.position.y;
    double dz = std::abs(target_pose.position.z - current_pose.position.z);
    double dist = std::sqrt(dx * dx + dy * dy + dz * dz);

    // Si el movimiento es pequeño, usar cartesiano
    if (dist < 0.3 || dz < 0.02)
    {
        RCLCPP_INFO(node->get_logger(), "Movimiento CARTESIANO");

        std::vector<geometry_msgs::msg::Pose> waypoints;
        waypoints.push_back(target_pose);

        moveit_msgs::msg::RobotTrajectory trajectory;

        double fraction = move_group.computeCartesianPath(
            waypoints,
            0.01,   // eef_step
            0.0,    // jump_threshold
            trajectory
        );

        if (fraction < 0.9) {
            RCLCPP_WARN(node->get_logger(),
                "Trayectoria cartesiana incompleta (fraction=%.2f). Probando con movimiento JOINT...", fraction);

            move_group.setPoseTarget(target_pose);

            moveit::planning_interface::MoveGroupInterface::Plan plan;
            bool ok = (move_group.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);

            if (!ok) {
                RCLCPP_ERROR(node->get_logger(), "Punto NO alcanzable");
                rclcpp::shutdown();
                spin_thread.join();
                return 1;
            }

            RCLCPP_INFO(node->get_logger(), "Punto alcanzable con JOINT, ejecutando...");

            auto exec_ok = move_group.execute(plan);

            if (exec_ok == moveit::core::MoveItErrorCode::SUCCESS) {
                RCLCPP_INFO(node->get_logger(), "Movimiento JOINT ejecutado correctamente");
            } else {
                RCLCPP_ERROR(node->get_logger(), "Error ejecutando movimiento JOINT");
            }

            rclcpp::shutdown();
            spin_thread.join();
            return 0;
        }

        moveit::planning_interface::MoveGroupInterface::Plan plan;
        plan.trajectory_ = trajectory;

        auto exec_ok = move_group.execute(plan);

        if (exec_ok == moveit::core::MoveItErrorCode::SUCCESS) {
            RCLCPP_INFO(node->get_logger(), "Movimiento cartesiano ejecutado correctamente");
        } else {
            RCLCPP_ERROR(node->get_logger(), "Error ejecutando movimiento cartesiano");
        }
    }
    else
    {
        RCLCPP_INFO(node->get_logger(), "Movimiento JOINT (planificador)");

        move_group.setPoseTarget(target_pose);

        moveit::planning_interface::MoveGroupInterface::Plan plan;
        bool ok = (move_group.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);

        if (!ok) {
            RCLCPP_ERROR(node->get_logger(), "Punto NO alcanzable");
            rclcpp::shutdown();
            spin_thread.join();
            return 1;
        }

        RCLCPP_INFO(node->get_logger(), "Punto alcanzable, ejecutando...");

        auto exec_ok = move_group.execute(plan);

        if (exec_ok == moveit::core::MoveItErrorCode::SUCCESS) {
            RCLCPP_INFO(node->get_logger(), "Movimiento ejecutado correctamente");
        } else {
            RCLCPP_ERROR(node->get_logger(), "Error ejecutando movimiento");
        }
    }

    rclcpp::shutdown();
    spin_thread.join();
    return 0;
}
