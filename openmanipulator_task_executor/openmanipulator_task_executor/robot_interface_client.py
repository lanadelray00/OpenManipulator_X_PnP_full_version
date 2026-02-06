import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from action_msgs.msg import GoalStatus
from moveit_msgs.srv import GetPositionFK
from moveit_msgs.msg import RobotState
from sensor_msgs.msg import JointState

from pnp_interfaces.action import (
    MoveToPose,
    MoveToNamed,
    MoveToJointPose, 
    GripperControl,
    EmergencyStop,
)


class RobotInterfaceClient:
    """
    Action / Service / Subscription helper.
    ❗ NOT a ROS Node
    ❗ No spin / wait responsibility
    """

    def __init__(self, node: Node):
        # ======================================================
        # Node reference (ownership stays in PickAndPlaceNode)
        # ======================================================
        self.node = node                              # [MODIFIED]
        self.logger = node.get_logger()               # [MODIFIED]
        self.logger.info("🧪 RobotInterfaceClient helper initialized")

        # ======================================================
        # Action Clients
        # ======================================================
        self.move_to_pose_client = ActionClient(
            node, MoveToPose, 'move_to_pose'
        )
        self.move_to_named_client = ActionClient(
            node, MoveToNamed, 'move_to_named'
        )
        self.move_to_joint_pose_client = ActionClient(
            node, MoveToJointPose, 'move_to_joint_pose'
        )

        self.gripper_client = ActionClient(
            node, GripperControl, 'gripper_control'
        )
        self.emergency_client = ActionClient(
            node, EmergencyStop, 'emergency_stop'
        )

        # ======================================================
        # FK Service & JointState Subscription
        # ======================================================
        self.fk_client = node.create_client(           # [MODIFIED]
            GetPositionFK, '/compute_fk'
        )
        self.subscription = node.create_subscription(  # [MODIFIED]
            JointState, '/joint_states',
            self.joint_callback, 10
        )

        # ======================================================
        # State cache
        # ======================================================
        self.current_position = None
        self.current_orientation = None

        self.default_velocity_scaling = 0.2
        self.default_acceleration_scaling = 0.2        

        self.wait_for_servers()

    # ======================================================
    # Wait for servers (allowed, blocking only at startup)
    # ======================================================
    def wait_for_servers(self):
        self.logger.info("⏳ Waiting for action servers...")

        self.move_to_pose_client.wait_for_server()
        self.move_to_named_client.wait_for_server()
        self.move_to_joint_pose_client.wait_for_server()
        self.gripper_client.wait_for_server()
        self.emergency_client.wait_for_server()
        self.fk_client.wait_for_service()

        self.logger.info("✅ All action servers available")

    # ======================================================
    # MoveToPose (ASYNC ONLY)
    # ======================================================
    def send_move_to_pose(self, x, y, z, qx, qy, qz, qw,
                          velocity_scaling=None,
                          acceleration_scaling=None):
        goal = MoveToPose.Goal()
        goal.target_pose.position.x = float(x)
        goal.target_pose.position.y = float(y)
        goal.target_pose.position.z = float(z)
        goal.target_pose.orientation.x = float(qx)
        goal.target_pose.orientation.y = float(qy)
        goal.target_pose.orientation.z = float(qz)
        goal.target_pose.orientation.w = float(qw)

        goal.velocity_scaling = (
            velocity_scaling
            if velocity_scaling is not None
            else self.default_velocity_scaling
        )
        goal.acceleration_scaling = (
            acceleration_scaling
            if acceleration_scaling is not None
            else self.default_acceleration_scaling
        )

        return self.move_to_pose_client.send_goal_async(goal)

    # ======================================================
    # MoveToNamed (ASYNC ONLY)
    # ======================================================
    def send_move_to_named(self, name,
                           velocity_scaling=None,
                           acceleration_scaling=None):
        goal = MoveToNamed.Goal()
        goal.name = name

        goal.velocity_scaling = (
            velocity_scaling
            if velocity_scaling is not None
            else self.default_velocity_scaling
        )
        goal.acceleration_scaling = (
            acceleration_scaling
            if acceleration_scaling is not None
            else self.default_acceleration_scaling
        )

        return self.move_to_named_client.send_goal_async(goal)

    # ======================================================
    # MoveToJointPose (ASYNC ONLY)
    # ======================================================
    def send_move_to_joint_pose(self, joints,
                                velocity_scaling=None,
                                acceleration_scaling=None):
        goal = MoveToJointPose.Goal()
        goal.joints = [float(j) for j in joints]

        goal.velocity_scaling = (
            velocity_scaling
            if velocity_scaling is not None
            else self.default_velocity_scaling
        )
        goal.acceleration_scaling = (
            acceleration_scaling
            if acceleration_scaling is not None
            else self.default_acceleration_scaling
        )

        return self.move_to_joint_pose_client.send_goal_async(goal)

    # ======================================================
    # Gripper (ASYNC ONLY)
    # ======================================================
    def send_gripper(self, position, effort=5.0):
        goal = GripperControl.Goal()
        goal.position = float(position)
        goal.max_effort = float(effort)

        return self.gripper_client.send_goal_async(goal)

    # ======================================================
    # Emergency Stop
    # ======================================================
    def send_emergency_stop(self):
        goal = EmergencyStop.Goal()
        goal.stop = True
        return self.emergency_client.send_goal_async(goal)

    # ======================================================
    # FK Calculation
    # ======================================================
    def joint_callback(self, msg):
        if not self.fk_client.service_is_ready():
            return

        request = GetPositionFK.Request()
        request.header.frame_id = 'world'
        request.fk_link_names = ['end_effector_link']

        robot_state = RobotState()
        robot_state.joint_state = msg
        request.robot_state = robot_state

        future = self.fk_client.call_async(request)
        future.add_done_callback(self.fk_response_callback)

    def fk_response_callback(self, future):
        try:
            response = future.result()
            if response.pose_stamped:
                pose = response.pose_stamped[0].pose
                self.current_position = [
                    pose.position.x,
                    pose.position.y,
                    pose.position.z,
                ]
                self.current_orientation = [
                    pose.orientation.x,
                    pose.orientation.y,
                    pose.orientation.z,
                    pose.orientation.w,
                ]
        except Exception as e:
            self.logger.error(f"FK call failed: {e}")
