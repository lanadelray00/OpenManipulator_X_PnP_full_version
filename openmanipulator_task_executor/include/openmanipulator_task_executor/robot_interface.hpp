#pragma once

#include <memory>
#include <string>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "geometry_msgs/msg/pose.hpp"
#include "control_msgs/action/gripper_command.hpp"

#include "moveit/move_group_interface/move_group_interface.h"

#include "pnp_interfaces/action/move_to_pose.hpp"
#include "pnp_interfaces/action/move_to_joint_pose.hpp"
#include "pnp_interfaces/action/move_to_named.hpp"
#include "pnp_interfaces/action/gripper_control.hpp"
#include "pnp_interfaces/action/emergency_stop.hpp"

class RobotInterface : public rclcpp::Node
{
public:
  RobotInterface();
  void initMoveGroups();

private:
  // interface using name
  using MoveToPose = pnp_interfaces::action::MoveToPose;
  using MoveToJointPose = pnp_interfaces::action::MoveToJointPose;
  using MoveToNamed = pnp_interfaces::action::MoveToNamed;
  using GripperControl = pnp_interfaces::action::GripperControl;
  using EmergencyStop = pnp_interfaces::action::EmergencyStop;
  // GoalHandle 
  using GoalHandleMoveToPose = rclcpp_action::ServerGoalHandle<MoveToPose>;
  using GoalHandleMoveToJointPose = rclcpp_action::ServerGoalHandle<MoveToJointPose>;
  using GoalHandleMoveToNamed = rclcpp_action::ServerGoalHandle<MoveToNamed>;
  using GoalHandleGripper = rclcpp_action::ServerGoalHandle<GripperControl>;
  using GoalHandleEStop = rclcpp_action::ServerGoalHandle<EmergencyStop>;

  // MoveIt
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> arm_group_;

  // Action Servers
  rclcpp_action::Server<MoveToPose>::SharedPtr move_to_pose_server_;
  rclcpp_action::Server<MoveToJointPose>::SharedPtr move_to_joint_pose_server_;
  rclcpp_action::Server<MoveToNamed>::SharedPtr move_to_named_server_;
  rclcpp_action::Server<GripperControl>::SharedPtr gripper_server_;
  rclcpp_action::Server<EmergencyStop>::SharedPtr estop_server_;

  // Gripper HW Action Client
  rclcpp_action::Client<control_msgs::action::GripperCommand>::SharedPtr gripper_hw_client_;

 // ===== MoveToPose =====
  void applyMotionScaling(double velocity, double acceleration);

  // ===== MoveToPose =====
  rclcpp_action::GoalResponse handleMoveToPoseGoal(
    const rclcpp_action::GoalUUID &,
    std::shared_ptr<const MoveToPose::Goal> goal);

  void executeMoveToPose(const std::shared_ptr<GoalHandleMoveToPose> goal_handle);

  // ===== MoveToNamed =====
  rclcpp_action::GoalResponse handleMoveToJointPoseGoal(
    const rclcpp_action::GoalUUID &,
    std::shared_ptr<const MoveToJointPose::Goal> goal);

  void executeMoveToJointPose(
    const std::shared_ptr<GoalHandleMoveToJointPose> goal_handle);

  // ===== MoveToNamed =====
  rclcpp_action::GoalResponse handleMoveToNamedGoal(
    const rclcpp_action::GoalUUID &,
    std::shared_ptr<const MoveToNamed::Goal> goal);

  void executeMoveToNamed(const std::shared_ptr<GoalHandleMoveToNamed> goal_handle);

  // ===== Gripper =====
  rclcpp_action::GoalResponse handleGripperGoal(
    const rclcpp_action::GoalUUID &,
    std::shared_ptr<const GripperControl::Goal> goal);

  void executeGripper(const std::shared_ptr<GoalHandleGripper> goal_handle);

  bool sendGripperCommand(double position, double effort);

  // ===== Emergency Stop =====
  rclcpp_action::GoalResponse handleEStopGoal(
    const rclcpp_action::GoalUUID &,
    std::shared_ptr<const EmergencyStop::Goal> goal);

  void executeEmergencyStop(const std::shared_ptr<GoalHandleEStop> goal_handle);
};
