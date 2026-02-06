#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import cv2
from std_msgs.msg import Bool
import threading

# from robot_interface_client import RobotInterfaceClient
# from ArUco_coord_extractor import MarkerPoseProcessor
from openmanipulator_task_executor.robot_interface_client import RobotInterfaceClient
from openmanipulator_task_executor.ArUco_coord_extractor import MarkerPoseProcessor



class PickAndPlaceNode(Node):

    JOINT_POSES = {
        "ground_10": [0.0752, -1.1919, 0.4556, 1.6935],
        "pick_up": [0.0, -0.6, 0.3, 1.4],
        "placing_spot": [1.6199, 0.1427, 0.2102, 1.2471]
    }

    POSITION_OFFSET = {
        "x": 0.01,
        "y": 0.01,
        "z": -0.01,
    }

    def __init__(self):
        super().__init__('Openmanipulator_pick_and_place_node')

        # ======================================================
        # Robot Interface (helper)
        # ======================================================
        self.robot = RobotInterfaceClient(self)

        # [REMOVED]
        # success = self.robot.move_to_joint_pose_and_wait(...)
        # 이유:
        #   helper에는 blocking API가 없고,
        #   노드에서 비동기 시퀀스로 처리해야 함

        # ======================================================
        # State flags
        # ======================================================
        self.start_requested = False
        self.is_executing = False
        self.action_in_progress = False
        self.current_stage = "idle"   # [MODIFIED] 상태 머신용

        # ======================================================
        # Marker Processor
        # ======================================================
        self.processor = MarkerPoseProcessor(self.robot)

        # ======================================================
        # Camera
        # ======================================================
        cv2.utils.logging.setLogLevel(cv2.utils.logging.LOG_LEVEL_ERROR)
        url = "http://192.168.0.33:5000/video_feed"
        self.cap = cv2.VideoCapture(url, cv2.CAP_FFMPEG)

        if not self.cap.isOpened():
            self.get_logger().error("Camera open failed")
            raise RuntimeError("Camera open failed")

        self.camera_running = True
        threading.Thread(target=self.camera_loop, daemon=True).start()

        # ======================================================
        # Timer (main loop)
        # ======================================================
        self.timer = self.create_timer(1.0 / 30.0, self.loop)

        # ======================================================
        # Trigger
        # ======================================================
        self.trigger_sub = self.create_subscription(
            Bool,
            '/pick_and_place/start',
            self.trigger_cb,
            10
        )

        self.get_logger().info("🟢 Pick & Place node started")
        self.get_logger().info("👉 Waiting for /pick_and_place/start trigger")

        # ======================================================
        # Startup motion (ASYNC)
        # ======================================================
        self.send_joint_pose(self.JOINT_POSES["ground_10"], next_stage="idle")

    # ======================================================
    # Trigger
    # ======================================================
    def trigger_cb(self, msg):
        if msg.data and not self.is_executing:
            self.get_logger().info("▶ Start trigger received")
            self.start_requested = True
            self.processor.start_recording()

    # ======================================================
    # Camera thread
    # ======================================================
    def camera_loop(self):
        while self.camera_running:
            ret, frame = self.cap.read()
            if not ret:
                continue
            self.processor.process_frame(frame)
            cv2.imshow("camera", frame)
            cv2.waitKey(1)

    # ======================================================
    # Main loop (state driven)
    # ======================================================
    def loop(self):
        if self.action_in_progress:
            return

        if self.start_requested and self.processor.is_ready():
            self.start_requested = False
            self.is_executing = True

            pose = self.processor.get_refined_pose()
            self.target_pose = pose
            self.current_stage = "pick"
            self.execute_next_stage()

    # ======================================================
    # State machine
    # ======================================================
    def execute_next_stage(self):
        self.action_in_progress = True

        if self.current_stage == "pick":
            self.execute_pick()

        elif self.current_stage == "lift":
            self.send_joint_pose(
                self.JOINT_POSES["pick_up"],
                next_stage="place"
            )

        elif self.current_stage == "place":
            self.send_joint_pose(
                self.JOINT_POSES["placing_spot"],
                next_stage="release"
            )

        elif self.current_stage == "release":
            self.send_gripper(0.019, next_stage="return")

        elif self.current_stage == "return":
            self.send_joint_pose(
                self.JOINT_POSES["ground_10"],
                next_stage="done"
            )

        elif self.current_stage == "done":
            self.get_logger().info("✅ Pick & Place completed")
            self.is_executing = False
            self.current_stage = "idle"
            self.action_in_progress = False

    # ======================================================
    # Pick
    # ======================================================
    def execute_pick(self):
        x, y, z, qx, qy, qz, qw = self.target_pose
        x += self.POSITION_OFFSET["x"]
        y += self.POSITION_OFFSET["y"]
        z += self.POSITION_OFFSET["z"]

        self.get_logger().info(
            f"🎯 Target pose: {x:.3f}, {y:.3f}, {z:.3f}"
        )

        self.send_gripper(0.019)
        future = self.robot.send_move_to_pose(x, y, z, qx, qy, qz, qw)
        future.add_done_callback(
            lambda f: self.on_action_done("lift")
        )

    # ======================================================
    # Helpers (ASYNC)
    # ======================================================
    def send_joint_pose(self, joints, next_stage):
        future = self.robot.send_move_to_joint_pose(joints)
        future.add_done_callback(
            lambda f: self.on_action_done(next_stage)
        )

    def send_gripper(self, position, next_stage=None):
        future = self.robot.send_gripper(position)
        if next_stage:
            future.add_done_callback(
                lambda f: self.on_action_done(next_stage)
            )
        else:
            self.action_in_progress = False

    def on_action_done(self, next_stage):
        self.action_in_progress = False
        self.current_stage = next_stage
        # self.execute_next_stage()

    # ======================================================
    # Shutdown
    # ======================================================
    def destroy(self):
        self.camera_running = False
        self.cap.release()
        cv2.destroyAllWindows()
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
