// pick_and_place.cpp
// UR3e Pick and Place con MoveIt 2 + Pilz Planner
// ROS 2 Humble

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <thread>
#include <chrono>
#include <cmath>
#include <map>
#include <string>
#include <vector>

using namespace std::chrono_literals;

// ─────────────────────────────────────────────────────────────
// CONSTANTES ARTICULARES (calibrar con RViz / joint_states)
// Usamos map<string,double> → el orden del topic no importa
// ─────────────────────────────────────────────────────────────

const std::map<std::string, double> HOME_JOINTS = {
    {"shoulder_pan_joint",   0.0},
    {"shoulder_lift_joint", -1.57},
    {"elbow_joint",          1.57},
    {"wrist_1_joint",       -1.57},
    {"wrist_2_joint",       -1.57},
    {"wrist_3_joint",        0.0}
};

const std::map<std::string, double> PRE_PICK_JOINTS = {
    {"shoulder_pan_joint",   0.3},
    {"shoulder_lift_joint", -1.8},
    {"elbow_joint",          2.0},
    {"wrist_1_joint",       -1.7},
    {"wrist_2_joint",       -1.57},
    {"wrist_3_joint",        0.0}
};

const std::map<std::string, double> PRE_PLACE_JOINTS = {
    {"shoulder_pan_joint",  -0.3},
    {"shoulder_lift_joint", -1.8},
    {"elbow_joint",          2.0},
    {"wrist_1_joint",       -1.7},
    {"wrist_2_joint",       -1.57},
    {"wrist_3_joint",        0.0}
};

// ─────────────────────────────────────────────────────────────
// FUNCIÓN AUXILIAR: construir PoseStamped
// ─────────────────────────────────────────────────────────────

geometry_msgs::msg::PoseStamped make_pose(
    double x, double y, double z,
    double qx, double qy, double qz, double qw,
    const std::string & frame = "base_link")
{
    geometry_msgs::msg::PoseStamped pose;
    pose.header.frame_id    = frame;
    pose.pose.position.x    = x;
    pose.pose.position.y    = y;
    pose.pose.position.z    = z;
    pose.pose.orientation.x = qx;
    pose.pose.orientation.y = qy;
    pose.pose.orientation.z = qz;
    pose.pose.orientation.w = qw;
    return pose;
}

// ─────────────────────────────────────────────────────────────
// NODO PRINCIPAL
// ─────────────────────────────────────────────────────────────

class PickAndPlace : public rclcpp::Node
{
public:
    PickAndPlace()
    : Node("pick_and_place")
    {
        // Orientación TCP apuntando hacia abajo para UR3e estándar.
        // Si el robot sigue retorciéndose durante LIN, ejecuta:
        //   ros2 run tf2_ros tf2_echo base_link tool0
        // con el robot en posición de trabajo y usa ese quaternion.
        tcp_down_ = {-0.707, 0.707, 0.0, 0.0};  // qx, qy, qz, qw
    }

    // Llamar después de construir el nodo (necesita shared_ptr listo)
    void init()
    {
        move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
            shared_from_this(), "ur_manipulator");

        move_group_->setMaxVelocityScalingFactor(0.15);
        move_group_->setMaxAccelerationScalingFactor(0.10);

        RCLCPP_INFO(get_logger(), "PickAndPlace listo.");
    }

    // ── Espera activa hasta recibir estado real del robot ─────
    // ARREGLO PROBLEMA 1: evita planificar desde estado todo-ceros
    void wait_for_real_state()
    {
        RCLCPP_INFO(get_logger(), "Esperando estado real del robot...");
        for (int i = 0; i < 50; ++i) {   // máximo 5 segundos
            move_group_->setStartStateToCurrentState();
            auto current = move_group_->getCurrentJointValues();

            // Si al menos un joint tiene valor no nulo, el estado llegó
            bool valid = false;
            for (auto v : current) {
                if (std::abs(v) > 0.01) { valid = true; break; }
            }
            if (valid) {
                RCLCPP_INFO(get_logger(),
                    "Estado recibido → pan: %.3f  lift: %.3f  elbow: %.3f",
                    current[0], current[1], current[2]);
                return;
            }
            std::this_thread::sleep_for(100ms);
        }
        RCLCPP_WARN(get_logger(), "Timeout esperando estado. Se continúa de todas formas.");
    }

    // ── Movimiento articular PTP (determinista) ───────────────
    bool go_to_joints(const std::map<std::string, double> & joint_map,
                      const std::string & planner = "PTP")
    {
        wait_for_real_state();
        move_group_->setPlannerId(planner);
        move_group_->setJointValueTarget(joint_map);

        moveit::planning_interface::MoveGroupInterface::Plan plan;
        bool ok = (move_group_->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);
        if (ok) move_group_->execute(plan);
        move_group_->stop();
        return ok;
    }

    // ── Movimiento lineal cartesiano LIN ─────────────────────
    bool go_lin(const geometry_msgs::msg::PoseStamped & pose)
    {
        wait_for_real_state();
        move_group_->setPlannerId("LIN");
        move_group_->setPoseTarget(pose);

        moveit::planning_interface::MoveGroupInterface::Plan plan;
        bool ok = (move_group_->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);
        if (ok) move_group_->execute(plan);
        move_group_->stop();
        return ok;
    }

    // ── Secuencia de PICK ─────────────────────────────────────
    bool pick(double x, double y, double z_pick)
    {
        double qx = tcp_down_[0], qy = tcp_down_[1],
               qz = tcp_down_[2], qw = tcp_down_[3];
        double z_pre = z_pick + 0.12;  // 12 cm sobre el punto de agarre

        // 1. PTP articular a pre-pick (sin IK ambiguo)
        RCLCPP_INFO(get_logger(), "PTP → pre-pick joints");
        if (!go_to_joints(PRE_PICK_JOINTS)) {
            RCLCPP_ERROR(get_logger(), "Fallo PTP pre-pick");
            return false;
        }
        std::this_thread::sleep_for(300ms);

        // 2. LIN bajada al punto de agarre
        RCLCPP_INFO(get_logger(), "LIN ↓ pick  (%.3f, %.3f, %.3f)", x, y, z_pick);
        if (!go_lin(make_pose(x, y, z_pick, qx, qy, qz, qw))) {
            RCLCPP_ERROR(get_logger(), "Fallo LIN pick");
            return false;
        }

        // 3. Cerrar gripper (placeholder — añadir control real aquí)
        RCLCPP_INFO(get_logger(), "Cerrando gripper...");
        std::this_thread::sleep_for(500ms);

        // 4. LIN subida vertical
        RCLCPP_INFO(get_logger(), "LIN ↑ post-pick");
        if (!go_lin(make_pose(x, y, z_pre, qx, qy, qz, qw))) {
            RCLCPP_ERROR(get_logger(), "Fallo LIN subida pick");
            return false;
        }

        return true;
    }

    // ── Secuencia de PLACE ────────────────────────────────────
    bool place(double x, double y, double z_place)
    {
        double qx = tcp_down_[0], qy = tcp_down_[1],
               qz = tcp_down_[2], qw = tcp_down_[3];
        double z_pre = z_place + 0.12;

        // 1. PTP articular a pre-place
        RCLCPP_INFO(get_logger(), "PTP → pre-place joints");
        if (!go_to_joints(PRE_PLACE_JOINTS)) {
            RCLCPP_ERROR(get_logger(), "Fallo PTP pre-place");
            return false;
        }
        std::this_thread::sleep_for(300ms);

        // 2. LIN bajada al punto de colocación
        RCLCPP_INFO(get_logger(), "LIN ↓ place  (%.3f, %.3f, %.3f)", x, y, z_place);
        if (!go_lin(make_pose(x, y, z_place, qx, qy, qz, qw))) {
            RCLCPP_ERROR(get_logger(), "Fallo LIN place");
            return false;
        }

        // 3. Abrir gripper (placeholder)
        RCLCPP_INFO(get_logger(), "Abriendo gripper...");
        std::this_thread::sleep_for(500ms);

        // 4. LIN subida
        RCLCPP_INFO(get_logger(), "LIN ↑ post-place");
        if (!go_lin(make_pose(x, y, z_pre, qx, qy, qz, qw))) {
            RCLCPP_ERROR(get_logger(), "Fallo LIN subida place");
            return false;
        }

        return true;
    }

    // ── Ciclo completo pick → place ───────────────────────────
    bool full_pick_and_place(
        double pick_x,  double pick_y,  double pick_z,
        double place_x, double place_y, double place_z)
    {
        RCLCPP_INFO(get_logger(), "Yendo a HOME...");
        if (!go_to_joints(HOME_JOINTS))          return false;

        if (!pick(pick_x, pick_y, pick_z))       return false;
        if (!place(place_x, place_y, place_z))   return false;

        RCLCPP_INFO(get_logger(), "Volviendo a HOME...");
        return go_to_joints(HOME_JOINTS);
    }

private:
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
    std::vector<double> tcp_down_;  // quaternion {qx, qy, qz, qw}
};

// ─────────────────────────────────────────────────────────────
// MAIN
// ─────────────────────────────────────────────────────────────

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);

    rclcpp::executors::MultiThreadedExecutor executor;
    auto node = std::make_shared<PickAndPlace>();
    executor.add_node(node);

    // Spin en hilo separado
    std::thread spin_thread([&executor]() { executor.spin(); });

    // Espera inicial para que MoveGroupInterface se suscriba a los topics
    std::this_thread::sleep_for(2s);

    node->init();

    // Espera adicional antes del primer movimiento
    std::this_thread::sleep_for(1s);

    // ── Ejecutar ciclo ────────────────────────────────────────
    node->full_pick_and_place(
        0.25,  0.00, 0.30,   // pick:  x, y, z
        0.20,  0.10, 0.30    // place: x, y, z
    );

    executor.cancel();
    spin_thread.join();
    rclcpp::shutdown();
    return 0;
}
