#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>

#include <thread>
#include <chrono>
#include <cmath>
#include <map>
#include <string>
#include <vector>

using namespace std::chrono_literals;

constexpr double DEG2RAD = M_PI / 180.0;
constexpr double JOINT_TOLERANCE_RAD = 1.0 * DEG2RAD;

const std::map<std::string, double> ORIENT_90_JOINTS = {
    {"shoulder_pan_joint",  -5.044},  // -289°
    {"shoulder_lift_joint", -1.134},  // -65°
    {"elbow_joint",          0.733},  // 42°
    {"wrist_1_joint",       -1.187},  // -68°
    {"wrist_2_joint",       -1.571},  // -90°
    {"wrist_3_joint",       -0.332}   // -19°
};

double normalize_angle(double a)
{
    while (a > M_PI)  a -= 2.0 * M_PI;
    while (a < -M_PI) a += 2.0 * M_PI;
    return a;
}

double shortest_angular_distance(double from, double to)
{
    return normalize_angle(to - from);
}

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);

    auto node = rclcpp::Node::make_shared("nodo90grados");

    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    std::thread spin_thread([&executor]() { executor.spin(); });

    std::this_thread::sleep_for(2s);

    moveit::planning_interface::MoveGroupInterface move_group(node, "ur_manipulator");

    move_group.setMaxVelocityScalingFactor(0.10);
    move_group.setMaxAccelerationScalingFactor(0.08);
    move_group.setPlanningTime(10.0);
    move_group.setNumPlanningAttempts(10);
    move_group.allowReplanning(false);

    move_group.setStartStateToCurrentState();

    auto joint_names = move_group.getJointNames();
    auto current_values = move_group.getCurrentJointValues();

    if (joint_names.size() != current_values.size()) {
        RCLCPP_ERROR(node->get_logger(), "Mismatch entre joint names y current values.");
        rclcpp::shutdown();
        spin_thread.join();
        return 1;
    }

    std::vector<double> target_vector = current_values;
    bool all_within_tolerance = true;

    for (size_t i = 0; i < joint_names.size(); ++i) {
        const std::string & name = joint_names[i];

        auto it = ORIENT_90_JOINTS.find(name);
        if (it == ORIENT_90_JOINTS.end()) {
            continue;
        }

        const double current = current_values[i];
        const double desired = it->second;

        const double diff = shortest_angular_distance(current, desired);
        double nearest_target = current + diff;

        if (std::abs(diff) <= JOINT_TOLERANCE_RAD) {
            nearest_target = current;
        } else {
            all_within_tolerance = false;
        }

        target_vector[i] = nearest_target;

        RCLCPP_INFO(
            node->get_logger(),
            "[%s] current=%.3f desired=%.3f nearest=%.3f diff=%.3f",
            name.c_str(), current, desired, nearest_target, diff);
    }

    if (all_within_tolerance) {
        RCLCPP_INFO(node->get_logger(), "Ya esta en la postura objetivo. No se planifica trayectoria.");
        rclcpp::shutdown();
        spin_thread.join();
        return 0;
    }

    move_group.setJointValueTarget(target_vector);

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    bool ok = (move_group.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);

    if (!ok) {
        RCLCPP_ERROR(node->get_logger(), "Error planificando movimiento");
        rclcpp::shutdown();
        spin_thread.join();
        return 1;
    }

    auto exec_ok = move_group.execute(plan);
    if (exec_ok == moveit::core::MoveItErrorCode::SUCCESS) {
        RCLCPP_INFO(node->get_logger(), "Movimiento ejecutado correctamente");
    } else {
        RCLCPP_ERROR(node->get_logger(), "Error ejecutando movimiento");
    }

    rclcpp::shutdown();
    spin_thread.join();
    return 0;
}
