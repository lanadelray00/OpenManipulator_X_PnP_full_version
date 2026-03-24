#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from std_msgs.msg import Bool
from pnp_interfaces.msg import DetectedObjectArray
from openmanipulator_task_executor.robot_interface_client import RobotInterfaceClient


class PickAndPlaceNode(Node):

    JOINT_POSES = {
        "ground_10": [0.0752, -1.1919, 0.4556, 1.6935],
        "pick_up": [0.0, -0.6, 0.3, 1.4],
        "placing_spot": [1.5171, 0.6351, -0.0782, 0.9603]
    }

    POSITION_OFFSET = {
        "x": -0.01,
        "y": 0.00,
        "z": 0.05,
    }

    def __init__(self):
        super().__init__('Openmanipulator_pick_and_place_node')

        # ======================================================
        # Robot Interface
        # ======================================================
        self.robot = RobotInterfaceClient(self)

        # ======================================================
        # State flags
        # ======================================================
        self.start_requested = False
        self.is_executing = False
        self.auto_advance = False
        self.action_in_progress = False
        self.initializing = True
        self.current_stage = None
        self.wait_log_printed = False

        # ======================================================
        # Auto toggle mode
        # ======================================================
        self.auto_run_enabled = False   # /pick_and_place/start 받을 때마다 토글

        # ======================================================
        # Latest detected objects from coordinate extractor
        # ======================================================
        self.latest_objects_msg = None

        # ======================================================
        # Trigger
        # ======================================================
        self.trigger_sub = self.create_subscription(
            Bool,
            '/pick_and_place/start',
            self.trigger_cb,
            10
        )

        # ======================================================
        # Coordinate input from ArUco node
        # ======================================================
        self.objects_sub = self.create_subscription(
            DetectedObjectArray,
            '/aruco/refined_objects',
            self.objects_cb,
            10
        )

        # ======================================================
        # Done publisher
        # ======================================================
        self.done_pub = self.create_publisher(
            Bool,
            '/pick_and_place/done',
            10
        )

        # ======================================================
        # Timer (main loop)
        # ======================================================
        self.timer = self.create_timer(1.0 / 30.0, self.loop)

        # Notification
        self.get_logger().info("🟢 Pick & Place node started")
        self.get_logger().info("👉 Waiting for /pick_and_place/start trigger")

        # ======================================================
        # Startup motion
        # ======================================================
        self.send_joint_pose(self.JOINT_POSES["ground_10"], next_stage=None)


    # ======================================================
    # Trigger: message 수신할 때마다 자동모드 토글
    # ======================================================
    def trigger_cb(self, msg):
        self.auto_run_enabled = not self.auto_run_enabled

        if self.auto_run_enabled:
            self.get_logger().info("▶ AUTO Pick&Place ENABLED")
            if not self.is_executing and self.current_stage == "idle":
                self.start_requested = True
        else:
            self.get_logger().info("⏸ AUTO Pick&Place DISABLED")

    # ======================================================
    # Detection subscriber
    # ======================================================
    def objects_cb(self, msg: DetectedObjectArray):
        self.latest_objects_msg = msg

    # ======================================================
    # Main loop
    # ======================================================
    def loop(self):
        if self.action_in_progress:
            return

        if self.start_requested and not self.is_executing:
            self.start_requested = False
            self.is_executing = True
            self.execute_next_stage()

        elif self.current_stage == "pick" and self.is_executing:
            self.execute_next_stage()

        elif self.auto_advance:
            self.auto_advance = False
            self.is_executing = True
            self.execute_next_stage()

        # 한 사이클 완료 후 idle이면, auto_run_enabled 상태일 때만 다시 시작
        elif (
            self.auto_run_enabled
            and not self.is_executing
            and self.current_stage == "idle"
        ):
            self.start_requested = True

    # ======================================================
    # State machine
    # ======================================================
    def execute_next_stage(self):

        if self.current_stage == "idle":
            self.current_stage = "pick_open"
            self.execute_next_stage()
            return

        if self.current_stage == "pick_open":
            self.get_logger().info("🟢 gripper open")
            self.send_gripper(0.019, next_stage="pick")

        elif self.current_stage == "pick":
            if not self.wait_log_printed:
                self.get_logger().info("🟢 pick")
                self.wait_log_printed = True
            self.execute_pick(next_stage="grip")

        elif self.current_stage == "grip":
            self.get_logger().info("🟢 grip")
            self.send_gripper(-0.010, next_stage="lift")

        elif self.current_stage == "lift":
            self.get_logger().info("🟢 lift")
            self.send_joint_pose(
                self.JOINT_POSES["pick_up"],
                next_stage="place"
            )

        elif self.current_stage == "place":
            self.get_logger().info("🟢 place")
            self.send_joint_pose(
                self.JOINT_POSES["placing_spot"],
                next_stage="release"
            )

        elif self.current_stage == "release":
            self.get_logger().info("🟢 release")
            self.send_gripper(0.019, next_stage="return")

        elif self.current_stage == "return":
            self.get_logger().info("🟢 return")

            msg = Bool()
            msg.data = True
            self.done_pub.publish(msg)

            self.send_joint_pose(
                self.JOINT_POSES["ground_10"],
                next_stage="done"
            )

        elif self.current_stage == "done":
            self.get_logger().info("✅ Pick & Place completed")
            self.current_stage = "idle"
            self.is_executing = False

    # ======================================================
    # Pick using latest DetectedObjectArray.objects[0]
    # ======================================================
    def execute_pick(self, next_stage):
        if self.action_in_progress:
            return

        if self.latest_objects_msg is None:
            if not self.wait_log_printed:
                self.get_logger().info("⏳ waiting for detected objects topic...")
                self.wait_log_printed = True
            return

        if len(self.latest_objects_msg.objects) == 0:
            if not self.wait_log_printed:
                self.get_logger().info("⏳ waiting for detected object array...")
                self.wait_log_printed = True
            return

        self.wait_log_printed = False
        self.action_in_progress = True

        target = self.latest_objects_msg.objects[0]

        x = target.pose.position.x
        y = target.pose.position.y
        z = target.pose.position.z
        qx = target.pose.orientation.x
        qy = target.pose.orientation.y
        qz = target.pose.orientation.z
        qw = target.pose.orientation.w

        x += self.POSITION_OFFSET["x"]
        y += self.POSITION_OFFSET["y"]
        z += self.POSITION_OFFSET["z"]

        self.get_logger().info(
            f"🎯 Target[0] marker_id={target.marker_id}, "
            f"group_index={target.group_index}, "
            f"pos=({x:.3f}, {y:.3f}, {z:.3f})"
        )

        future = self.robot.send_move_to_pose(x, y, z, qx, qy, qz, qw)
        future.add_done_callback(
            lambda f: self.on_action_done(next_stage)
        )

    # ======================================================
    # Helpers
    # ======================================================
    def send_joint_pose(self, joints, next_stage):
        self.action_in_progress = True
        future = self.robot.send_move_to_joint_pose(joints)
        future.add_done_callback(
            lambda f: self.on_action_done(next_stage)
        )

    def send_gripper(self, position, next_stage=None):
        self.action_in_progress = True
        future = self.robot.send_gripper(position)
        future.add_done_callback(
            lambda f: self.on_action_done(next_stage)
        )

    def on_action_done(self, next_stage):
        self.action_in_progress = False

        if self.initializing:
            self.initializing = False
            self.current_stage = "idle"
            self.get_logger().info("🟢 Initial pose ready, FSM idle")
            return

        self.current_stage = next_stage
        self.is_executing = False
        self.auto_advance = True

    # ======================================================
    # Shutdown
    # ======================================================
    def destroy(self):
        self.destroy_node()


def main():
    rclpy.init()
    node = PickAndPlaceNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("🛑 Shutdown requested")
    finally:
        node.destroy()
        rclpy.shutdown()


if __name__ == '__main__':
    main()