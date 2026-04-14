#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <geometry_msgs/msg/pose.hpp>
#include <tf2/LinearMath/Quaternion.h>

#include <thread>
#include <chrono>
#include <cmath>
#include <sstream>
#include <iomanip>

using namespace std::chrono_literals;

std::string poseToString(const geometry_msgs::msg::Pose& pose)
{
    std::ostringstream oss;
    oss << std::fixed << std::setprecision(4)
        << "pos[x=" << pose.position.x
        << ", y=" << pose.position.y
        << ", z=" << pose.position.z
        << "] "
        << "quat[x=" << pose.orientation.x
        << ", y=" << pose.orientation.y
        << ", z=" << pose.orientation.z
        << ", w=" << pose.orientation.w << "]";
    return oss.str();
}

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);

    auto node = rclcpp::Node::make_shared("move_to_pose");

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
    move_group.setStartStateToCurrentState();

    geometry_msgs::msg::Pose target_pose;
    target_pose.position.x = 0.40;
    target_pose.position.y = 0.10;
    target_pose.position.z = 0.25;

    tf2::Quaternion q;
    q.setRPY(M_PI, 0.0, 0.0);
    q.normalize();

    target_pose.orientation.x = q.x();
    target_pose.orientation.y = q.y();
    target_pose.orientation.z = q.z();
    target_pose.orientation.w = q.w();

    const double X_MIN = 0.20;
    const double X_MAX = 0.60;
    const double Y_MIN = -0.30;
    const double Y_MAX = 0.30;
    const double Z_MIN = 0.12;
    const double Z_MAX = 0.60;

    if (target_pose.position.x < X_MIN || target_pose.position.x > X_MAX ||
        target_pose.position.y < Y_MIN || target_pose.position.y > Y_MAX ||
        target_pose.position.z < Z_MIN || target_pose.position.z > Z_MAX)
    {
        RCLCPP_ERROR(node->get_logger(), "Pose objetivo fuera de la zona segura");
        RCLCPP_ERROR(node->get_logger(), "Pose pedida: %s", poseToString(target_pose).c_str());
        rclcpp::shutdown();
        spin_thread.join();
        return 1;
    }

    auto current_pose = move_group.getCurrentPose().pose;

    RCLCPP_INFO(node->get_logger(), "Pose actual:");
    RCLCPP_INFO(node->get_logger(), "%s", poseToString(current_pose).c_str());

    RCLCPP_INFO(node->get_logger(), "Pose objetivo:");
    RCLCPP_INFO(node->get_logger(), "%s", poseToString(target_pose).c_str());

    double dx_before = target_pose.position.x - current_pose.position.x;
    double dy_before = target_pose.position.y - current_pose.position.y;
    double dz_before = target_pose.position.z - current_pose.position.z;
    double dist_before = std::sqrt(dx_before * dx_before + dy_before * dy_before + dz_before * dz_before);

    RCLCPP_INFO(
        node->get_logger(),
        "Distancia inicial al objetivo: dx=%.4f, dy=%.4f, dz=%.4f, norm=%.4f m",
        dx_before, dy_before, dz_before, dist_before
    );

    move_group.setPoseTarget(target_pose);

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    bool ok = (move_group.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);

    if (!ok) {
        RCLCPP_ERROR(node->get_logger(), "Error planificando movimiento cartesiano");
        rclcpp::shutdown();
        spin_thread.join();
        return 1;
    }

    RCLCPP_INFO(node->get_logger(), "Plan encontrado. Ejecutando movimiento...");

    auto exec_ok = move_group.execute(plan);
    if (exec_ok == moveit::core::MoveItErrorCode::SUCCESS) {
        RCLCPP_INFO(node->get_logger(), "Movimiento ejecutado correctamente");
    } else {
        RCLCPP_ERROR(node->get_logger(), "Error ejecutando movimiento");
        rclcpp::shutdown();
        spin_thread.join();
        return 1;
    }

    move_group.setStartStateToCurrentState();
    auto final_pose = move_group.getCurrentPose().pose;

    RCLCPP_INFO(node->get_logger(), "Pose final:");
    RCLCPP_INFO(node->get_logger(), "%s", poseToString(final_pose).c_str());

    double dx_after = final_pose.position.x - target_pose.position.x;
    double dy_after = final_pose.position.y - target_pose.position.y;
    double dz_after = final_pose.position.z - target_pose.position.z;
    double position_error = std::sqrt(dx_after * dx_after + dy_after * dy_after + dz_after * dz_after);

    RCLCPP_INFO(
        node->get_logger(),
        "Error posicion final: dx=%.4f, dy=%.4f, dz=%.4f, norm=%.4f m",
        dx_after, dy_after, dz_after, position_error
    );

    const double POSITION_TOLERANCE = 0.02;

    if (position_error < POSITION_TOLERANCE) {
        RCLCPP_INFO(node->get_logger(), "Movimiento correcto dentro de tolerancia");
    } else {
        RCLCPP_WARN(node->get_logger(), "Movimiento fuera de tolerancia");
    }

    rclcpp::shutdown();
    spin_thread.join();
    return 0;
}
