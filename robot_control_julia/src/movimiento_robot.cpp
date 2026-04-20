#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <moveit_msgs/msg/collision_object.hpp>
#include <moveit_msgs/msg/robot_trajectory.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>
#include <std_msgs/msg/string.hpp>

#include <chrono>
#include <cmath>
#include <condition_variable>
#include <deque>
#include <map>
#include <memory>
#include <mutex>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

using namespace std::chrono_literals;

struct NamedJointPose
{
  std::vector<double> joints;
};

struct Command
{
  enum class Type
  {
    NAMED_POSE,
    SIMPLE_POINT,
    POSE_STAMPED
  };

  Type type;
  std::string name;
  double x{0.0};
  double y{0.0};
  double z{0.0};
  double yaw{0.0};
  std::string modo;
  geometry_msgs::msg::PoseStamped pose_stamped;
};

class MovimientoRobot : public rclcpp::Node
{
public:
  MovimientoRobot()
  : Node("movimiento_robot")
  {
    feedback_pub_ =
      this->create_publisher<std_msgs::msg::String>("/robot_feedback", 10);
      
    reached_pose_pub_ = this->create_publisher<std_msgs::msg::String>("/robot_pose_alcanzada", 10);

    named_pose_sub_ = this->create_subscription<std_msgs::msg::String>(
      "/ir_a_pose_guardada",
      10,
      std::bind(&MovimientoRobot::namedPoseCallback, this, std::placeholders::_1));

    simple_point_sub_ = this->create_subscription<std_msgs::msg::String>(
      "/ir_a_punto_simple",
      10,
      std::bind(&MovimientoRobot::simplePointCallback, this, std::placeholders::_1));

    pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      "/ir_a_punto_pose",
      10,
      std::bind(&MovimientoRobot::poseCallback, this, std::placeholders::_1));

    loadNamedPoses();

    worker_thread_ = std::thread(&MovimientoRobot::workerLoop, this);

    RCLCPP_INFO(this->get_logger(), "movimiento_robot listo para recibir comandos");
    publishFeedback("movimiento_robot listo");
  }

  ~MovimientoRobot() override
  {
    {
      std::lock_guard<std::mutex> lock(queue_mutex_);
      stop_worker_ = true;
    }
    queue_cv_.notify_all();

    if (worker_thread_.joinable()) {
      worker_thread_.join();
    }
  }

private:
  void loadNamedPoses()
  {
    named_poses_["PoseIntermedia"] = {{-3.12,-1.7160,-0.9334,-2.0651,1.5688,0.9521}};
    named_poses_["DetectaTorre"] = {{0.7788,-0.0245,0.4485,-3.9326,0.8652,-0.1735}};
    named_poses_["DetectaPiezasSueltas"] = {{-0.617,-1.7169,-0.9337,-2.0650,1.5690,0.9522}};

  }

  geometry_msgs::msg::Pose buildPose(double x, double y, double z, const std::string & modo, double yaw)
  {
    geometry_msgs::msg::Pose target_pose;
    target_pose.position.x = x;
    target_pose.position.y = y;
    target_pose.position.z = z;

    if (modo == "down")
    {
      tf2::Quaternion q;
      q.setRPY(M_PI, 0.0, yaw);

      target_pose.orientation = tf2::toMsg(q);
    }
    else if (modo == "front")
    {
      target_pose.orientation.x = 0.0;
      target_pose.orientation.y = 0.707;
      target_pose.orientation.z = 0.0;
      target_pose.orientation.w = -0.707;
    }
    else
    {
      RCLCPP_WARN(
        this->get_logger(),
        "Modo desconocido (%s). Usando 'down'.",
        modo.c_str());

      target_pose.orientation.x = 1.0;
      target_pose.orientation.y = 0.0;
      target_pose.orientation.z = 0.0;
      target_pose.orientation.w = 0.0;
    }

    return target_pose;
  }

  void addStaticEnvironment(moveit::planning_interface::PlanningSceneInterface & planning_scene_interface)
  {
    moveit_msgs::msg::CollisionObject mesa;
    mesa.header.frame_id = "base_link";
    mesa.id = "mesa";

    shape_msgs::msg::SolidPrimitive mesa_primitive;
    mesa_primitive.type = mesa_primitive.BOX;
    mesa_primitive.dimensions = {0.7, 1.2, 0.05};

    geometry_msgs::msg::Pose mesa_pose;
    mesa_pose.orientation.w = 1.0;
    mesa_pose.position.x = 0.0;
    mesa_pose.position.y = 0.0;
    mesa_pose.position.z = -0.025;

    mesa.primitives.push_back(mesa_primitive);
    mesa.primitive_poses.push_back(mesa_pose);
    mesa.operation = mesa.ADD;

    moveit_msgs::msg::CollisionObject pared;
    pared.header.frame_id = "base_link";
    pared.id = "pared_delante";

    shape_msgs::msg::SolidPrimitive pared_primitive;
    pared_primitive.type = pared_primitive.BOX;
    pared_primitive.dimensions = {0.02, 1.2, 0.80};

    geometry_msgs::msg::Pose pared_pose;
    pared_pose.orientation.w = 1.0;
    pared_pose.position.x = -0.36;
    pared_pose.position.y = 0.0;
    pared_pose.position.z = 0.40;

    pared.primitives.push_back(pared_primitive);
    pared.primitive_poses.push_back(pared_pose);
    pared.operation = pared.ADD;

    planning_scene_interface.applyCollisionObject(mesa);
    planning_scene_interface.applyCollisionObject(pared);

    rclcpp::sleep_for(500ms);
  }

  bool executeLikeWorkingNode(const geometry_msgs::msg::Pose & target_pose)
  {
    auto internal_node = rclcpp::Node::make_shared("movimiento_robot_executor_internal");

    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(internal_node);

    std::thread spin_thread([&executor]() { executor.spin(); });

    std::this_thread::sleep_for(2s);

    moveit::planning_interface::MoveGroupInterface move_group(internal_node, "ur3e");

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
    addStaticEnvironment(planning_scene_interface);

    auto current_pose = move_group.getCurrentPose().pose;

    double dx = target_pose.position.x - current_pose.position.x;
    double dy = target_pose.position.y - current_pose.position.y;
    double dz = std::abs(target_pose.position.z - current_pose.position.z);
    double dist = std::sqrt(dx * dx + dy * dy + dz * dz);

    RCLCPP_INFO(
      this->get_logger(),
      "Current pose -> x=%.3f y=%.3f z=%.3f",
      current_pose.position.x,
      current_pose.position.y,
      current_pose.position.z);

    RCLCPP_INFO(
      this->get_logger(),
      "Target pose  -> x=%.3f y=%.3f z=%.3f ori=(%.3f %.3f %.3f %.3f)",
      target_pose.position.x,
      target_pose.position.y,
      target_pose.position.z,
      target_pose.orientation.x,
      target_pose.orientation.y,
      target_pose.orientation.z,
      target_pose.orientation.w);

    bool success = false;

    if (dist < 0.3 || dz < 0.02)
    {
      RCLCPP_INFO(this->get_logger(), "Movimiento CARTESIANO");

      std::vector<geometry_msgs::msg::Pose> waypoints;
      waypoints.push_back(target_pose);

      moveit_msgs::msg::RobotTrajectory trajectory;

      double fraction = move_group.computeCartesianPath(
        waypoints,
        0.01,
        0.0,
        trajectory
      );

      if (fraction < 0.9)
      {
        RCLCPP_WARN(
          this->get_logger(),
          "Cartesiano incompleto (fraction=%.2f). Fallback a JOINT...",
          fraction);

        move_group.setPoseTarget(target_pose);

        moveit::planning_interface::MoveGroupInterface::Plan plan;
        bool ok = (move_group.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);

        if (!ok) {
          RCLCPP_ERROR(this->get_logger(), "Punto NO alcanzable");
          publishFeedback("error: punto no alcanzable");
          success = false;
        } else {
          auto exec_ok = move_group.execute(plan);

          if (exec_ok == moveit::core::MoveItErrorCode::SUCCESS) {
            RCLCPP_INFO(this->get_logger(), "Movimiento JOINT ejecutado correctamente");
            publishFeedback("ok: movimiento joint ejecutado");
            success = true;
          } else {
            RCLCPP_ERROR(this->get_logger(), "Error ejecutando movimiento JOINT");
            publishFeedback("error: fallo ejecutando movimiento joint");
            success = false;
          }
        }
      }
      else
      {
        moveit::planning_interface::MoveGroupInterface::Plan plan;
        plan.trajectory_ = trajectory;

        auto exec_ok = move_group.execute(plan);

        if (exec_ok == moveit::core::MoveItErrorCode::SUCCESS) {
          RCLCPP_INFO(this->get_logger(), "Movimiento cartesiano ejecutado correctamente");
          publishFeedback("ok: movimiento cartesiano ejecutado");
          success = true;
        } else {
          RCLCPP_ERROR(this->get_logger(), "Error ejecutando movimiento cartesiano");
          publishFeedback("error: fallo ejecutando movimiento cartesiano");
          success = false;
        }
      }
    }
    else
    {
      RCLCPP_INFO(this->get_logger(), "Movimiento JOINT");

      move_group.setPoseTarget(target_pose);

      moveit::planning_interface::MoveGroupInterface::Plan plan;
      bool ok = (move_group.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);

      if (!ok) {
        RCLCPP_ERROR(this->get_logger(), "Punto NO alcanzable");
        publishFeedback("error: punto no alcanzable");
        success = false;
      } else {
        auto exec_ok = move_group.execute(plan);

        if (exec_ok == moveit::core::MoveItErrorCode::SUCCESS) {
          RCLCPP_INFO(this->get_logger(), "Movimiento ejecutado correctamente");
          publishFeedback("ok: movimiento ejecutado");
          success = true;
        } else {
          RCLCPP_ERROR(this->get_logger(), "Error ejecutando movimiento");
          publishFeedback("error: fallo ejecutando movimiento");
          success = false;
        }
      }
    }

    executor.cancel();
    if (spin_thread.joinable()) {
      spin_thread.join();
    }

    return success;
  }

  bool executeNamedJointLikeWorkingNode(const std::vector<double>& joints)
  {
    auto internal_node = rclcpp::Node::make_shared("movimiento_robot_executor_internal_joint");

    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(internal_node);

    std::thread spin_thread([&executor]() { executor.spin(); });

    std::this_thread::sleep_for(2s);

    moveit::planning_interface::MoveGroupInterface move_group(internal_node, "ur3e");

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
    addStaticEnvironment(planning_scene_interface);

    bool success = false;

    move_group.setJointValueTarget(joints);

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    bool ok = (move_group.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);

    if (!ok) {
      RCLCPP_ERROR(this->get_logger(), "Pose predefinida NO alcanzable");
      publishFeedback("error: pose predefinida no alcanzable");
      success = false;
    } else {
      auto exec_ok = move_group.execute(plan);

      if (exec_ok == moveit::core::MoveItErrorCode::SUCCESS) {
        RCLCPP_INFO(this->get_logger(), "Pose predefinida ejecutada correctamente");
        publishFeedback("ok: pose predefinida ejecutada");
        success = true;
      } else {
        RCLCPP_ERROR(this->get_logger(), "Error ejecutando pose predefinida");
        publishFeedback("error: fallo ejecutando pose predefinida");
        success = false;
      }
    }

    executor.cancel();
    if (spin_thread.joinable()) {
      spin_thread.join();
    }

    return success;
  }

  void enqueueCommand(const Command & cmd)
  {
    {
      std::lock_guard<std::mutex> lock(queue_mutex_);
      command_queue_.push_back(cmd);
    }
    queue_cv_.notify_one();
  }

  void workerLoop()
  {
    while (rclcpp::ok()) {
      Command cmd;
      {
        std::unique_lock<std::mutex> lock(queue_mutex_);
        queue_cv_.wait(lock, [this]() {
          return stop_worker_ || !command_queue_.empty();
        });

        if (stop_worker_) {
          return;
        }

        cmd = command_queue_.front();
        command_queue_.pop_front();
      }

      switch (cmd.type)
      {
        case Command::Type::NAMED_POSE:
        {
          auto it = named_poses_.find(cmd.name);
          if (it == named_poses_.end()) {
            RCLCPP_ERROR(this->get_logger(), "Pose preguardada desconocida: %s", cmd.name.c_str());
            publishFeedback("error: pose preguardada desconocida");
            break;
          }

          bool ok = executeNamedJointLikeWorkingNode(it->second.joints);
  	  if (ok) {
            publishReachedPose(cmd.name);
          }
          break;
        }

        case Command::Type::SIMPLE_POINT:
        {
          const auto target_pose = buildPose(cmd.x, cmd.y, cmd.z, cmd.modo, cmd.yaw);
          executeLikeWorkingNode(target_pose);
          break;
        }

        case Command::Type::POSE_STAMPED:
        {
          executeLikeWorkingNode(cmd.pose_stamped.pose);
          break;
        }
      }
    }
  }

  void namedPoseCallback(const std_msgs::msg::String::SharedPtr msg)
  {
    RCLCPP_INFO(
      this->get_logger(),
      "Comando recibido /ir_a_pose_guardada: %s",
      msg->data.c_str());

    Command cmd;
    cmd.type = Command::Type::NAMED_POSE;
    cmd.name = msg->data;
    enqueueCommand(cmd);
  }

  void simplePointCallback(const std_msgs::msg::String::SharedPtr msg)
  {
    std::istringstream iss(msg->data);

    double x, y, z, yaw;
    std::string modo;

    if (!(iss >> x >> y >> z >> modo >> yaw))
    {
      RCLCPP_ERROR(
        this->get_logger(),
        "Formato inválido en /ir_a_punto_simple. Usa: \"x y z modo yaw\"");
      publishFeedback("error: formato invalido en /ir_a_punto_simple");
      return;
    }

    RCLCPP_INFO(
      this->get_logger(),
      "Comando recibido /ir_a_punto_simple: x=%.3f y=%.3f z=%.3f modo=%s",
      x, y, z, modo.c_str());

    Command cmd;
    cmd.type = Command::Type::SIMPLE_POINT;
    cmd.x = x;
    cmd.y = y;
    cmd.z = z;
    cmd.yaw = yaw;
    cmd.modo = modo;
    enqueueCommand(cmd);
  }

  void poseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    RCLCPP_INFO(
      this->get_logger(),
      "Comando recibido /ir_a_punto_pose frame=%s",
      msg->header.frame_id.c_str());

    Command cmd;
    cmd.type = Command::Type::POSE_STAMPED;
    cmd.pose_stamped = *msg;
    enqueueCommand(cmd);
  }

  void publishFeedback(const std::string & text)
  {
    std_msgs::msg::String msg;
    msg.data = text;
    feedback_pub_->publish(msg);
  }
  
  void publishReachedPose(const std::string & pose_name)
  {
    std_msgs::msg::String msg;
    msg.data = pose_name;
    reached_pose_pub_->publish(msg);
  }
  
private:
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr named_pose_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr simple_point_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr feedback_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr reached_pose_pub_;

  std::map<std::string, NamedJointPose> named_poses_;

  std::deque<Command> command_queue_;
  std::mutex queue_mutex_;
  std::condition_variable queue_cv_;
  bool stop_worker_{false};
  std::thread worker_thread_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<MovimientoRobot>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
