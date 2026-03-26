#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_srvs/srv/trigger.hpp>

#include "custom_interfaces/srv/move_linear.hpp"
#include "custom_interfaces/srv/move_joints.hpp"
#include "custom_interfaces/srv/move_preset.hpp"
#include "custom_interfaces/srv/adjust_robot_speed.hpp"

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>

#include <moveit_msgs/msg/collision_object.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>

#include <memory>
#include <map>
#include <vector>
#include <utility>
#include <chrono>

using std::placeholders::_1;
using std::placeholders::_2;
using namespace std::chrono_literals;

class MoveItLux : public rclcpp::Node
{
public:
  MoveItLux()
  : Node("moveit_lux_node",
         rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)),
    logger_(this->get_logger())
  {
    // Callback groups
    pause_cb_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    move_cb_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    move_cb_group_reentrant_ = create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    save_joints_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    move_linear_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

    // Services
    move_linear_interface_ = create_service<custom_interfaces::srv::MoveLinear>(
      "/moveit_lux/move_linear",
      std::bind(&MoveItLux::MoveLinearService, this, _1, _2),
      rmw_qos_profile_services_default,
      move_linear_group_);

    move_preset_interface_ = create_service<custom_interfaces::srv::MovePreset>(
      "/moveit_lux/move_preset",
      std::bind(&MoveItLux::MovePresetService, this, _1, _2),
      rmw_qos_profile_services_default,
      move_cb_group_reentrant_);

    pause_robot_srv_ = create_service<std_srvs::srv::Trigger>(
      "/moveit_lux/stop_robot_movement",
      std::bind(&MoveItLux::PauseService, this, _1, _2),
      rmw_qos_profile_services_default,
      pause_cb_group_);

    adjust_robot_speed_srv_ = create_service<custom_interfaces::srv::AdjustRobotSpeed>(
      "/moveit_lux/adjust_robot_speed",
      std::bind(&MoveItLux::AdjustRobotSpeedService, this, _1, _2),
      rmw_qos_profile_services_default,
      move_cb_group_);

    save_joints_srv_ = create_service<std_srvs::srv::Trigger>(
      "/moveit_lux/save_joints",
      std::bind(&MoveItLux::SaveJointState, this, _1, _2),
      rmw_qos_profile_services_default,
      save_joints_group_);

    move_joints_srv_ = create_service<std_srvs::srv::Trigger>(
      "/moveit_lux/move_joints",
      std::bind(&MoveItLux::JointsService, this, _1, _2),
      rmw_qos_profile_services_default,
      save_joints_group_);

    // Delay MoveIt initialization until node is fully created and spinning
    init_timer_ = create_wall_timer(
      1000ms,
      std::bind(&MoveItLux::InitializeMoveIt, this));
  }

private:
  void InitializeMoveIt()
  {
    if (move_group_interface_) {
      return;
    }

    init_timer_->cancel();

    try {
      move_group_interface_ =
        std::make_shared<moveit::planning_interface::MoveGroupInterface>(
          shared_from_this(), robot_group);

      move_group_interface_->startStateMonitor();

      RCLCPP_INFO(logger_, "MoveGroupInterface initialized for group: %s", robot_group.c_str());

      // Give the state monitor a little time
      rclcpp::sleep_for(2s);

      SetupMoveit();
    } catch (const std::exception &e) {
      RCLCPP_ERROR(logger_, "Failed to initialize MoveIt: %s", e.what());
    }
  }

  bool EnsureMoveItReady()
  {
    if (!move_group_interface_) {
      RCLCPP_ERROR(logger_, "MoveIt is not initialized yet.");
      return false;
    }
    return true;
  }

  moveit::core::RobotStatePtr GetRobotState(double timeout_sec = 5.0)
  {
    if (!EnsureMoveItReady()) {
      return nullptr;
    }

    auto state = move_group_interface_->getCurrentState(timeout_sec);
    if (!state) {
      RCLCPP_ERROR(logger_, "Failed to get current robot state.");
    }
    return state;
  }

  bool GetCurrentJointPositions(std::vector<double> &joint_group_positions)
  {
    auto current_state = GetRobotState(5.0);
    if (!current_state) {
      return false;
    }

    const moveit::core::JointModelGroup *joint_model_group =
      current_state->getJointModelGroup(robot_group);

    if (!joint_model_group) {
      RCLCPP_ERROR(logger_, "Joint model group '%s' not found.", robot_group.c_str());
      return false;
    }

    current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);

    if (joint_group_positions.empty()) {
      RCLCPP_ERROR(logger_, "Joint group positions are empty.");
      return false;
    }

    RCLCPP_INFO(logger_, "number of joints: %zu", joint_group_positions.size());
    for (size_t i = 0; i < joint_group_positions.size(); ++i) {
      RCLCPP_INFO(logger_, "Joint %zu current: %f", i + 1, joint_group_positions[i]);
    }

    return true;
  }

  void SaveJointState(
    std_srvs::srv::Trigger::Request::SharedPtr /*request*/,
    std_srvs::srv::Trigger::Response::SharedPtr response)
  {
    std::vector<double> joint_group_positions;
    if (!GetCurrentJointPositions(joint_group_positions)) {
      response->success = false;
      response->message = "Could not read current joint state";
      return;
    }

    current_joint_states = joint_group_positions;
    response->success = true;
    response->message = "Joint positions saved";
  }

  void PauseService(
    std_srvs::srv::Trigger::Request::SharedPtr /*request*/,
    std_srvs::srv::Trigger::Response::SharedPtr response)
  {
    if (!EnsureMoveItReady()) {
      response->success = false;
      response->message = "MoveIt not initialized";
      return;
    }

    RCLCPP_INFO(logger_, "call moveit pause service");
    move_group_interface_->stop();
    response->success = true;
    response->message = "Robot movement stopped";
  }

  void MoveLinearService(
    const std::shared_ptr<custom_interfaces::srv::MoveLinear::Request> request,
    std::shared_ptr<custom_interfaces::srv::MoveLinear::Response> response)
  {
    if (!EnsureMoveItReady()) {
      response->success = false;
      return;
    }

    std::vector<double> joints;
    if (!GetCurrentJointPositions(joints)) {
      response->success = false;
      return;
    }

    RCLCPP_INFO(logger_, "start move linear");

    const auto target_pose = request->pose;

    move_group_interface_->setPlanningPipelineId("ompl");
    move_group_interface_->setGoalTolerance(0.0005);
    move_group_interface_->setGoalOrientationTolerance(0.0005);
    move_group_interface_->setPlanningTime(5.0);
    move_group_interface_->setMaxVelocityScalingFactor(0.25);
    move_group_interface_->setPoseTarget(target_pose, end_effector);

    RCLCPP_INFO(
      logger_,
      "set target pose to: %f, %f, %f",
      target_pose.position.x,
      target_pose.position.y,
      target_pose.position.z);

    auto [success, plan] = planToTargetPose();

    if (success) {
      RCLCPP_INFO(logger_, "success in creating the plan, move robot.");
      auto exec_result = move_group_interface_->execute(plan);
      response->success = (exec_result == moveit::core::MoveItErrorCode::SUCCESS);
    } else {
      RCLCPP_ERROR(logger_, "Planning failed!");
      response->success = false;
    }
  }

  void JointsService(
    std_srvs::srv::Trigger::Request::SharedPtr /*request*/,
    std_srvs::srv::Trigger::Response::SharedPtr response)
  {
    if (!EnsureMoveItReady()) {
      response->success = false;
      response->message = "MoveIt not initialized";
      return;
    }

    if (current_joint_states.empty()) {
      response->success = false;
      response->message = "No saved joint state. Call /moveit_lux/save_joints first.";
      RCLCPP_ERROR(logger_, "No saved joint state available.");
      return;
    }

    RCLCPP_INFO(logger_, "start move joints");
    RCLCPP_INFO(logger_, "number of joints to define: %zu", current_joint_states.size());

    for (size_t i = 0; i < current_joint_states.size(); ++i) {
      RCLCPP_INFO(logger_, "Joint %zu target: %f", i + 1, current_joint_states[i]);
    }

    bool within_bounds = move_group_interface_->setJointValueTarget(current_joint_states);
    if (!within_bounds) {
      RCLCPP_WARN(logger_, "Target joint position(s) were outside limits.");
    }

    auto [success, plan] = planToTargetPose();

    if (success) {
      RCLCPP_INFO(logger_, "success in creating the plan, move robot.");
      auto exec_result = move_group_interface_->execute(plan);
      response->success = (exec_result == moveit::core::MoveItErrorCode::SUCCESS);
      response->message = response->success ? "Move executed" : "Execution failed";
    } else {
      RCLCPP_ERROR(logger_, "Planning failed!");
      response->success = false;
      response->message = "Planning failed";
    }
  }

  void MovePresetService(
    const std::shared_ptr<custom_interfaces::srv::MovePreset::Request> request,
    std::shared_ptr<custom_interfaces::srv::MovePreset::Response> response)
  {
    if (!EnsureMoveItReady()) {
      response->success = false;
      return;
    }

    std::vector<double> joints;
    if (!GetCurrentJointPositions(joints)) {
      response->success = false;
      return;
    }

    std::map<std::string, double> target_preset;

    move_group_interface_->setGoalTolerance(0.005);
    move_group_interface_->setGoalOrientationTolerance(0.01);
    move_group_interface_->setPlannerId("RRTstarkConfigDefault");
    move_group_interface_->setNumPlanningAttempts(150);
    move_group_interface_->setPlanningTime(20.0);

    target_preset = move_group_interface_->getNamedTargetValues(request->preset_case);

    if (target_preset.empty()) {
      RCLCPP_ERROR(logger_, "Preset '%s' not found.", request->preset_case.c_str());
      response->success = false;
      return;
    }

    bool within_bounds = move_group_interface_->setJointValueTarget(target_preset);
    if (!within_bounds) {
      RCLCPP_ERROR(logger_, "Target joint position(s) were outside limits.");
      response->success = false;
      return;
    }

    auto [success_plan, plan] = planToTargetPose();

    if (success_plan) {
      auto success_move = move_group_interface_->execute(plan);
      if (success_move == moveit::core::MoveItErrorCode::SUCCESS) {
        RCLCPP_INFO(logger_, "Execution succeeded!");
        response->success = true;
      } else {
        RCLCPP_ERROR(logger_, "Execution failed!");
        response->success = false;
      }
    } else {
      RCLCPP_ERROR(logger_, "Planning failed!");
      response->success = false;
    }
  }

  void SetupMoveit()
  {
    std::vector<double> joints;
    if (!GetCurrentJointPositions(joints)) {
      RCLCPP_WARN(logger_, "Could not get current joint state during setup.");
      return;
    }

    RCLCPP_INFO(logger_, "MoveIt setup completed.");
  }

  void AdjustRobotSpeedService(
    const std::shared_ptr<custom_interfaces::srv::AdjustRobotSpeed::Request> /*request*/,
    std::shared_ptr<custom_interfaces::srv::AdjustRobotSpeed::Response> response)
  {
    if (!EnsureMoveItReady()) {
      response->success = false;
      return;
    }

    RCLCPP_INFO(logger_, "Adjust Speed of Robot");
    move_group_interface_->setMaxVelocityScalingFactor(0.5);
    move_group_interface_->setMaxAccelerationScalingFactor(0.5);
    response->success = true;
  }

  std::pair<bool, moveit::planning_interface::MoveGroupInterface::Plan> planToTargetPose()
  {
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    bool success = static_cast<bool>(move_group_interface_->plan(plan));
    return {success, plan};
  }

  moveit_msgs::msg::CollisionObject GenerateSmallCollisionObject()
  {
    moveit_msgs::msg::CollisionObject toolChangeCollision;
    toolChangeCollision.header.frame_id = move_group_interface_->getPlanningFrame();
    toolChangeCollision.id = "toolChange";

    shape_msgs::msg::SolidPrimitive toolChange;
    toolChange.type = toolChange.BOX;
    toolChange.dimensions.resize(3);
    toolChange.dimensions[toolChange.BOX_X] = 0.30;
    toolChange.dimensions[toolChange.BOX_Y] = 0.15;
    toolChange.dimensions[toolChange.BOX_Z] = 0.15;

    geometry_msgs::msg::Pose toolChangePose;
    toolChangePose.orientation.w = 1.0;
    toolChangePose.position.x = -1.1;
    toolChangePose.position.y = 0.4607;
    toolChangePose.position.z = 0.075;

    toolChangeCollision.primitives.push_back(toolChange);
    toolChangeCollision.primitive_poses.push_back(toolChangePose);
    toolChangeCollision.operation = toolChangeCollision.ADD;

    return toolChangeCollision;
  }

  moveit_msgs::msg::CollisionObject GenerateCollisionObject(
    double width, double height, double depth,
    double baseElevation, double wallWidth, double wallDepth)
  {
    moveit_msgs::msg::CollisionObject tvCollision;
    tvCollision.header.frame_id = move_group_interface_->getPlanningFrame();
    tvCollision.id = "TV_BOX";

    shape_msgs::msg::SolidPrimitive display;
    shape_msgs::msg::SolidPrimitive displayWallLeft;
    shape_msgs::msg::SolidPrimitive displayWallRight;

    display.type = display.BOX;
    display.dimensions.resize(3);
    display.dimensions[display.BOX_X] = width;
    display.dimensions[display.BOX_Y] = depth;
    display.dimensions[display.BOX_Z] = height;

    geometry_msgs::msg::Pose displayPose;
    displayPose.orientation.w = 1.0;
    displayPose.position.x = 0.0;
    displayPose.position.y = 0.0;
    displayPose.position.z = baseElevation + (height / 2.0);

    tvCollision.primitives.push_back(display);
    tvCollision.primitive_poses.push_back(displayPose);

    displayWallLeft.type = displayWallLeft.BOX;
    displayWallLeft.dimensions.resize(3);
    displayWallLeft.dimensions[displayWallLeft.BOX_X] = wallWidth;
    displayWallLeft.dimensions[displayWallLeft.BOX_Y] = wallDepth;
    displayWallLeft.dimensions[displayWallLeft.BOX_Z] = height;

    geometry_msgs::msg::Pose displayWallLeftPose;
    displayWallLeftPose.orientation.w = 1.0;
    displayWallLeftPose.position.x = -(width / 2.0) - (displayWallLeft.dimensions[0] / 2.0);
    displayWallLeftPose.position.y = 0.0;
    displayWallLeftPose.position.z = baseElevation + (height / 2.0);

    tvCollision.primitives.push_back(displayWallLeft);
    tvCollision.primitive_poses.push_back(displayWallLeftPose);

    displayWallRight.type = displayWallRight.BOX;
    displayWallRight.dimensions.resize(3);
    displayWallRight.dimensions[displayWallRight.BOX_X] = wallWidth;
    displayWallRight.dimensions[displayWallRight.BOX_Y] = wallDepth;
    displayWallRight.dimensions[displayWallRight.BOX_Z] = height;

    geometry_msgs::msg::Pose displayWallRightPose;
    displayWallRightPose.orientation.w = 1.0;
    displayWallRightPose.position.x = (width / 2.0) + (displayWallRight.dimensions[0] / 2.0);
    displayWallRightPose.position.y = 0.0;
    displayWallRightPose.position.z = baseElevation + (height / 2.0);

    tvCollision.primitives.push_back(displayWallRight);
    tvCollision.primitive_poses.push_back(displayWallRightPose);
    tvCollision.operation = tvCollision.ADD;

    return tvCollision;
  }

  // Services
  rclcpp::Service<custom_interfaces::srv::MoveLinear>::SharedPtr move_linear_interface_;
  rclcpp::Service<custom_interfaces::srv::MoveJoints>::SharedPtr move_joints_interface_;
  rclcpp::Service<custom_interfaces::srv::MovePreset>::SharedPtr move_preset_interface_;
  rclcpp::Service<custom_interfaces::srv::AdjustRobotSpeed>::SharedPtr adjust_robot_speed_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr save_joints_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr pause_robot_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr move_joints_srv_;

  // Callback groups
  rclcpp::CallbackGroup::SharedPtr pause_cb_group_;
  rclcpp::CallbackGroup::SharedPtr move_cb_group_;
  rclcpp::CallbackGroup::SharedPtr move_cb_group_reentrant_;
  rclcpp::CallbackGroup::SharedPtr save_joints_group_;
  rclcpp::CallbackGroup::SharedPtr move_linear_group_;

  // Delayed init timer
  rclcpp::TimerBase::SharedPtr init_timer_;

  // MoveIt
  std::string base_link = "base_link";
  std::string end_effector = "tool1";
  std::string robot_group = "ur10e";

  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_interface_;
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface_;

  // Logger
  rclcpp::Logger logger_;

  // Current joint states
  std::vector<double> current_joint_states;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<MoveItLux>();

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();

  rclcpp::shutdown();
  return 0;
}