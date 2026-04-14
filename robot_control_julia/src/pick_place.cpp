// pick_and_place.cpp
// UR3e Pick and Place con MoveIt 2
// ROS 2 Humble

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit_msgs/msg/robot_trajectory.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include <thread>
#include <chrono>
#include <cmath>
#include <map>
#include <string>
#include <vector>

using namespace std::chrono_literals;

// ============================================================
// CONSTANTES
// ============================================================

constexpr double DEG2RAD = M_PI / 180.0;
constexpr double JOINT_TOLERANCE_RAD = 1.0 * DEG2RAD;   // 1 grado
constexpr double POS_TOLERANCE_M = 0.005;               // 5 mm

// ============================================================
// JOINT TARGETS
// ============================================================

const std::map<std::string, double> HOME_JOINTS = {
    {"shoulder_pan_joint",   0.0},
    {"shoulder_lift_joint", -1.57},
    {"elbow_joint",          1.57},
    {"wrist_1_joint",       -1.57},
    {"wrist_2_joint",       -1.57},
    {"wrist_3_joint",        0.0}
};

const std::map<std::string, double> PRE_PICK_JOINTS = {
    {"shoulder_pan_joint",   -0.977},
    {"shoulder_lift_joint",  -2.094},
    {"elbow_joint",          -0.977},
    {"wrist_1_joint",        -1.571},
    {"wrist_2_joint",         1.571},
    {"wrist_3_joint",         0.0}
};

const std::map<std::string, double> PRE_PLACE_JOINTS = {
    {"shoulder_pan_joint",   -0.995},
    {"shoulder_lift_joint",  -2.269},
    {"elbow_joint",          -1.361},
    {"wrist_1_joint",        -1.012},
    {"wrist_2_joint",         1.571},
    {"wrist_3_joint",         0.0}
};

// ============================================================
// AUX
// ============================================================

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

// ============================================================
// MAIN NODE
// ============================================================

class PickAndPlace : public rclcpp::Node
{
public:
    PickAndPlace()
    : Node("pick_and_place")
    {
    }

    void init()
    {
        move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
            shared_from_this(), "ur_manipulator");

        move_group_->setMaxVelocityScalingFactor(0.10);
        move_group_->setMaxAccelerationScalingFactor(0.08);
        move_group_->setPlanningTime(5.0);
        move_group_->setNumPlanningAttempts(10);
        move_group_->allowReplanning(false);

        RCLCPP_INFO(get_logger(), "PickAndPlace listo.");
    }

    void wait_for_real_state()
    {
        for (int i = 0; i < 50; ++i) {
            move_group_->setStartStateToCurrentState();
            auto current = move_group_->getCurrentJointValues();

            bool valid = false;
            for (double v : current) {
                if (std::abs(v) > 0.01) {
                    valid = true;
                    break;
                }
            }

            if (valid) {
                return;
            }

            std::this_thread::sleep_for(100ms);
        }

        RCLCPP_WARN(get_logger(), "Timeout esperando estado real.");
    }

    bool go_to_joints_shortest(const std::map<std::string, double> & joint_map,
                               const std::string & planner = "PTP")
    {
        wait_for_real_state();

        move_group_->stop();
        move_group_->clearPoseTargets();
        move_group_->setStartStateToCurrentState();

        auto current_values = move_group_->getCurrentJointValues();
        auto joint_names = move_group_->getJointNames();

        if (current_values.size() != joint_names.size()) {
            RCLCPP_ERROR(get_logger(), "Mismatch entre joint names y current values.");
            return false;
        }

        std::vector<double> target_vector = current_values;
        bool all_within_tolerance = true;

        for (size_t i = 0; i < joint_names.size(); ++i) {
            const std::string & name = joint_names[i];

            auto it = joint_map.find(name);
            if (it == joint_map.end()) {
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
                get_logger(),
                "[%s] current=%.3f desired=%.3f nearest=%.3f diff=%.3f",
                name.c_str(), current, desired, nearest_target, diff);
        }

        // CLAVE: si ya está en el objetivo, NO planificar nada
        if (all_within_tolerance) {
            RCLCPP_INFO(get_logger(), "Ya esta en la posicion articular objetivo. No se planifica trayectoria.");
            return true;
        }

        move_group_->setPlannerId(planner);
        move_group_->setJointValueTarget(target_vector);

        moveit::planning_interface::MoveGroupInterface::Plan plan;
        bool ok = (move_group_->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);

        if (!ok) {
            RCLCPP_ERROR(get_logger(), "Fallo planificando movimiento articular.");
            return false;
        }

        auto exec_result = move_group_->execute(plan);
        move_group_->stop();
        move_group_->clearPoseTargets();

        return (exec_result == moveit::core::MoveItErrorCode::SUCCESS);
    }

    bool move_vertical_cartesian(double z_target)
    {
        wait_for_real_state();

        geometry_msgs::msg::Pose start_pose = move_group_->getCurrentPose().pose;

        // CLAVE: si ya está en esa Z, NO planificar nada
        if (std::fabs(start_pose.position.z - z_target) <= POS_TOLERANCE_M) {
            RCLCPP_INFO(get_logger(), "Ya esta en la altura objetivo. No se planifica trayectoria vertical.");
            return true;
        }

        geometry_msgs::msg::Pose target_pose = start_pose;
        target_pose.position.z = z_target;

        std::vector<geometry_msgs::msg::Pose> waypoints;
        waypoints.push_back(target_pose);

        moveit_msgs::msg::RobotTrajectory trajectory;
        const double eef_step = 0.003;
        const double jump_threshold = 0.0;

        double fraction = move_group_->computeCartesianPath(
            waypoints, eef_step, jump_threshold, trajectory);

        RCLCPP_INFO(get_logger(), "Cartesian path fraction: %.3f", fraction);

        if (fraction < 0.99) {
            RCLCPP_ERROR(get_logger(), "Trayectoria cartesiana incompleta.");
            return false;
        }

        moveit::planning_interface::MoveGroupInterface::Plan plan;
        plan.trajectory_ = trajectory;

        auto exec_result = move_group_->execute(plan);
        move_group_->stop();

        return (exec_result == moveit::core::MoveItErrorCode::SUCCESS);
    }

    bool pick(double z_pick)
    {
        const double z_pre = z_pick + 0.12;

        RCLCPP_INFO(get_logger(), "PTP -> pre-pick joints");
        if (!go_to_joints_shortest(PRE_PICK_JOINTS)) {
            RCLCPP_ERROR(get_logger(), "Fallo PTP pre-pick joints");
            return false;
        }
        std::this_thread::sleep_for(300ms);

        RCLCPP_INFO(get_logger(), "Cartesian -> bajar a pick");
        if (!move_vertical_cartesian(z_pick)) {
            RCLCPP_ERROR(get_logger(), "Fallo bajada cartesiana pick");
            return false;
        }

        RCLCPP_INFO(get_logger(), "Cerrando gripper...");
        std::this_thread::sleep_for(500ms);

        RCLCPP_INFO(get_logger(), "Cartesian -> subir post-pick");
        if (!move_vertical_cartesian(z_pre)) {
            RCLCPP_ERROR(get_logger(), "Fallo subida cartesiana pick");
            return false;
        }

        return true;
    }

    bool place(double z_place)
    {
        const double z_pre = z_place + 0.12;

        RCLCPP_INFO(get_logger(), "PTP -> pre-place joints");
        if (!go_to_joints_shortest(PRE_PLACE_JOINTS)) {
            RCLCPP_ERROR(get_logger(), "Fallo PTP pre-place joints");
            return false;
        }
        std::this_thread::sleep_for(300ms);

        RCLCPP_INFO(get_logger(), "Cartesian -> bajar a place");
        if (!move_vertical_cartesian(z_place)) {
            RCLCPP_ERROR(get_logger(), "Fallo bajada cartesiana place");
            return false;
        }

        RCLCPP_INFO(get_logger(), "Abriendo gripper...");
        std::this_thread::sleep_for(500ms);

        RCLCPP_INFO(get_logger(), "Cartesian -> subir post-place");
        if (!move_vertical_cartesian(z_pre)) {
            RCLCPP_ERROR(get_logger(), "Fallo subida cartesiana place");
            return false;
        }

        return true;
    }

    bool full_pick_and_place(double pick_z, double place_z)
    {
        RCLCPP_INFO(get_logger(), "Yendo a HOME...");
        if (!go_to_joints_shortest(HOME_JOINTS)) {
            return false;
        }

        if (!pick(pick_z)) {
            return false;
        }

        if (!place(place_z)) {
            return false;
        }

        RCLCPP_INFO(get_logger(), "Volviendo a HOME...");
        return go_to_joints_shortest(HOME_JOINTS);
    }

private:
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
};

// ============================================================
// MAIN
// ============================================================

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);

    rclcpp::executors::MultiThreadedExecutor executor;
    auto node = std::make_shared<PickAndPlace>();
    executor.add_node(node);

    std::thread spin_thread([&executor]() { executor.spin(); });

    std::this_thread::sleep_for(2s);
    node->init();
    std::this_thread::sleep_for(1s);

    node->full_pick_and_place(
        0.30,  // z pick
        0.30   // z place
    );

    executor.cancel();
    spin_thread.join();
    rclcpp::shutdown();
    return 0;
}
