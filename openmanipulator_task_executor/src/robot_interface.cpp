#include "openmanipulator_task_executor/robot_interface.hpp"
#include <algorithm>  // std::find


using namespace std::chrono_literals;

RobotInterface::RobotInterface() : Node("robot_interface")
{
  RCLCPP_INFO(get_logger(), "🚀 RobotInterface (Action-based) initialized");

  // ===== Action Servers =====
  move_to_pose_server_ = rclcpp_action::create_server<MoveToPose>(
    this,
    "move_to_pose",
    std::bind(&RobotInterface::handleMoveToPoseGoal, this, std::placeholders::_1, std::placeholders::_2),
    nullptr,
    std::bind(&RobotInterface::executeMoveToPose, this, std::placeholders::_1));

  move_to_joint_pose_server_ = rclcpp_action::create_server<MoveToJointPose>(
      this,
      "move_to_joint_pose",
      std::bind(&RobotInterface::handleMoveToJointPoseGoal, this, std::placeholders::_1, std::placeholders::_2),
      nullptr,
      std::bind(&RobotInterface::executeMoveToJointPose, this, std::placeholders::_1));


  move_to_named_server_ = rclcpp_action::create_server<MoveToNamed>(
    this,
    "move_to_named",
    std::bind(&RobotInterface::handleMoveToNamedGoal, this, std::placeholders::_1, std::placeholders::_2),
    nullptr,
    std::bind(&RobotInterface::executeMoveToNamed, this, std::placeholders::_1));

  gripper_server_ = rclcpp_action::create_server<GripperControl>(
    this,
    "gripper_control",
    std::bind(&RobotInterface::handleGripperGoal, this, std::placeholders::_1, std::placeholders::_2),
    nullptr,
    std::bind(&RobotInterface::executeGripper, this, std::placeholders::_1));

  estop_server_ = rclcpp_action::create_server<EmergencyStop>(
    this,
    "emergency_stop",
    std::bind(&RobotInterface::handleEStopGoal, this, std::placeholders::_1, std::placeholders::_2),
    nullptr,
    std::bind(&RobotInterface::executeEmergencyStop, this, std::placeholders::_1));

  // ===== Gripper HW Client =====
  gripper_hw_client_ =
    rclcpp_action::create_client<control_msgs::action::GripperCommand>(
      this, "/gripper_controller/gripper_cmd");

  while (!gripper_hw_client_->wait_for_action_server(2s)) {
    RCLCPP_INFO(get_logger(), "⏳ Waiting for gripper controller...");
  }
}

void RobotInterface::initMoveGroups()
{
  arm_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
    shared_from_this(), "arm");
  RCLCPP_INFO(get_logger(), "✅ MoveGroupInterface ready");
}

////////////////////////////////////////////////////////////
// Common Velocity, Acceleration Scaling Setting
////////////////////////////////////////////////////////////
void RobotInterface::applyMotionScaling(
  double velocity,
  double acceleration)
{
  double vel = std::clamp(velocity, 0.01, 1.0);
  double acc = std::clamp(acceleration, 0.01, 1.0);

  arm_group_->setMaxVelocityScalingFactor(vel);
  arm_group_->setMaxAccelerationScalingFactor(acc);
}

////////////////////////////////////////////////////////////
// MoveToPose
////////////////////////////////////////////////////////////
rclcpp_action::GoalResponse
RobotInterface::handleMoveToPoseGoal(
  const rclcpp_action::GoalUUID &,
  std::shared_ptr<const MoveToPose::Goal>)
{
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

void RobotInterface::executeMoveToPose(
  const std::shared_ptr<GoalHandleMoveToPose> goal_handle)
{
  auto goal = goal_handle->get_goal();
  auto feedback = std::make_shared<MoveToPose::Feedback>();
  auto result = std::make_shared<MoveToPose::Result>();
 
  // 1. Set Target Pose
  arm_group_->setPoseTarget(goal->target_pose);
  applyMotionScaling(goal->velocity_scaling, goal->acceleration_scaling);
  arm_group_->setGoalPositionTolerance(0.01);
  arm_group_->setGoalOrientationTolerance(0.2);
  arm_group_->setPlanningTime(5.0);

  // 2. Planning
  feedback->state = "PLANNING";
  goal_handle->publish_feedback(feedback);

  moveit::planning_interface::MoveGroupInterface::Plan plan;
  auto code = arm_group_->plan(plan);

  if (code != moveit::core::MoveItErrorCode::SUCCESS) {
    result->success = false;
    result->error_code = code.val;
    result->message = "Planning failed";
    goal_handle->abort(result);
    arm_group_->clearPoseTargets();
    return;
  }
  
  // 3. Executing
  feedback->state = "EXECUTING";
  goal_handle->publish_feedback(feedback);
  arm_group_->execute(plan);

  // 4. Done
  result->success = true;
  result->error_code = 0;
  result->message = "Motion complete";
  goal_handle->succeed(result);
}

////////////////////////////////////////////////////////////
// MoveToJointPose
////////////////////////////////////////////////////////////
rclcpp_action::GoalResponse
RobotInterface::handleMoveToJointPoseGoal(
  const rclcpp_action::GoalUUID &,
  std::shared_ptr<const MoveToJointPose::Goal> goal)
{
  const auto joint_names = arm_group_->getJointNames();

  if (goal->joints.size() != joint_names.size()) {
    RCLCPP_WARN(
      get_logger(),
      "❌ Joint size mismatch: expected %zu, got %zu",
      joint_names.size(),
      goal->joints.size()
    );
    return rclcpp_action::GoalResponse::REJECT;
  }

  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

void RobotInterface::executeMoveToJointPose(
  const std::shared_ptr<GoalHandleMoveToJointPose> goal_handle)
{
  auto goal = goal_handle->get_goal();
  auto feedback = std::make_shared<MoveToJointPose::Feedback>();
  auto result   = std::make_shared<MoveToJointPose::Result>();

  // 1. Set joint target
  arm_group_->setJointValueTarget(goal->joints);
  applyMotionScaling(goal->velocity_scaling, goal->acceleration_scaling);
  arm_group_->setPlanningTime(5.0);

  // 2. Planning
  feedback->state = "PLANNING";
  goal_handle->publish_feedback(feedback);

  moveit::planning_interface::MoveGroupInterface::Plan plan;
  auto code = arm_group_->plan(plan);

  if (code != moveit::core::MoveItErrorCode::SUCCESS) {
    result->success = false;
    result->message = "Planning failed";
    goal_handle->abort(result);
    return;
  }

  // 3. Executing
  feedback->state = "EXECUTING";
  goal_handle->publish_feedback(feedback);
  arm_group_->execute(plan);

  // 4. Done
  result->message = "Motion complete";
  goal_handle->publish_feedback(feedback);

  result->success = true;
  goal_handle->succeed(result);
}


////////////////////////////////////////////////////////////
// MoveToNamed
////////////////////////////////////////////////////////////
rclcpp_action::GoalResponse
RobotInterface::handleMoveToNamedGoal(
  const rclcpp_action::GoalUUID &,
  std::shared_ptr<const MoveToNamed::Goal> goal)
{
  const auto & names = arm_group_->getNamedTargets();

  auto it = std::find(names.begin(), names.end(), goal->name);
  if (it == names.end()) {
    RCLCPP_WARN(
      get_logger(),
      "❌ Invalid named target: '%s'",
      goal->name.c_str()
    );
    return rclcpp_action::GoalResponse::REJECT;
  }

  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

void RobotInterface::executeMoveToNamed(
  const std::shared_ptr<GoalHandleMoveToNamed> goal_handle)
{
  auto goal = goal_handle->get_goal();
  auto result = std::make_shared<MoveToNamed::Result>();

  arm_group_->setNamedTarget(goal->name);
  applyMotionScaling(goal->velocity_scaling, goal->acceleration_scaling);
  arm_group_->move();

  result->success = true;
  result->message = "Named motion complete";
  goal_handle->succeed(result);
}

////////////////////////////////////////////////////////////
// Gripper
////////////////////////////////////////////////////////////
rclcpp_action::GoalResponse
RobotInterface::handleGripperGoal(
  const rclcpp_action::GoalUUID &,
  std::shared_ptr<const GripperControl::Goal>)
{
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

void RobotInterface::executeGripper(
  const std::shared_ptr<GoalHandleGripper> goal_handle)
{
  auto goal = goal_handle->get_goal();
  auto result = std::make_shared<GripperControl::Result>();

  bool ok = sendGripperCommand(goal->position, goal->max_effort);

  result->success = ok;
  result->message = ok ? "Gripper command sent" : "Gripper failed";
  ok ? goal_handle->succeed(result) : goal_handle->abort(result);
}

bool RobotInterface::sendGripperCommand(double position, double effort)
{
  using Gripper = control_msgs::action::GripperCommand;

  if (!gripper_hw_client_->action_server_is_ready()) {
    return false;
  }

  Gripper::Goal goal;
  goal.command.position = position;
  goal.command.max_effort = effort;

  gripper_hw_client_->async_send_goal(goal);
  return true;
}

////////////////////////////////////////////////////////////
// Emergency Stop
////////////////////////////////////////////////////////////
rclcpp_action::GoalResponse
RobotInterface::handleEStopGoal(
  const rclcpp_action::GoalUUID &,
  std::shared_ptr<const EmergencyStop::Goal>)
{
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

void RobotInterface::executeEmergencyStop(
  const std::shared_ptr<GoalHandleEStop> goal_handle)
{
  auto result = std::make_shared<EmergencyStop::Result>();

  if (arm_group_) {
    arm_group_->stop();
    arm_group_->clearPoseTargets();
  }

  gripper_hw_client_->async_cancel_all_goals();

  result->success = true;
  result->message = "Emergency stop executed";
  goal_handle->succeed(result);
}

////////////////////////////////////////////////////////////
// main
////////////////////////////////////////////////////////////
int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<RobotInterface>();
  node->initMoveGroups();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
