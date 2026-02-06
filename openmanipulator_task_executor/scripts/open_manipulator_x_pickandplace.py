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
        self.step_mode = True # (True, 단계별 실행) <-> (False, 전체 자동 실행) 결정
        self.start_requested = False
        self.is_executing = False # trigger_cb 용 
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
        # ===============================
        # STEP MODE (단계별 실행)
        # ===============================
        if self.step_mode:
            if self.start_requested:
                self.start_requested = False
                self.is_executing = True
                self.execute_next_stage()   # ✅ FSM 진입은 여기 한 곳뿐

        # ===============================
        # AUTO MODE (전체 자동 실행)
        # ===============================
        else:
            if self.start_requested:
                self.start_requested = False
                self.is_executing = True
                self.execute_next_stage()
            elif self.is_executing:
                self.execute_next_stage()

    # ======================================================
    # State machine
    # ======================================================
    def execute_next_stage(self):

        if self.current_stage == "idle":
            self.current_stage = "pick_open"
            self.execute_next_stage()
            return

        if self.current_stage == "pick_open":
            self.get_logger().info("🟢gripper open")
            self.send_gripper(0.019, next_stage="pick")
        
        elif self.current_stage == "pick":
            self.get_logger().info("🟢pick")
            self.processor.start_recording()
            self.execute_pick(next_stage="grip")

        elif self.current_stage == "grip":
            self.get_logger().info("🟢grip")
            self.send_gripper(-0.010, next_stage="lift")

        elif self.current_stage == "lift":
            self.get_logger().info("🟢lift")
            self.send_joint_pose(
                self.JOINT_POSES["pick_up"],
                next_stage="place"
            )

        elif self.current_stage == "place":
            self.get_logger().info("🟢place")
            self.send_joint_pose(
                self.JOINT_POSES["placing_spot"],
                next_stage="release"
            )

        elif self.current_stage == "release":
            self.get_logger().info("🟢release")
            self.send_gripper(0.019, next_stage="return")

        elif self.current_stage == "return":
            self.get_logger().info("🟢return")
            self.send_joint_pose(
                self.JOINT_POSES["ground_10"],
                next_stage="done"
            )

        elif self.current_stage == "done":
            self.get_logger().info("✅ Pick & Place completed")
            self.current_stage = "idle"
            self.is_executing = False


    # ======================================================
    # Pick
    # ======================================================
    def execute_pick(self, next_stage):
        self.action_in_progress = True

        if self.processor.is_ready():
            self.target_pose = self.processor.get_refined_pose()
            x, y, z, qx, qy, qz, qw = self.target_pose
            x += self.POSITION_OFFSET["x"]
            y += self.POSITION_OFFSET["y"]
            z += self.POSITION_OFFSET["z"]

            self.get_logger().info(f"🎯 Target pose: {x:.3f}, {y:.3f}, {z:.3f}")
            future = self.robot.send_move_to_pose(x, y, z, qx, qy, qz, qw)

            future.add_done_callback(
                lambda f: self.on_action_done(next_stage)
            )
        else:
            self.get_logger().info("⏳ waiting for marker...")
            self.action_in_progress = False
            return


    # ======================================================
    # Helpers (ASYNC)
    # ======================================================
    def send_joint_pose(self, joints, next_stage):
        self.action_in_progress = True
        future = self.robot.send_move_to_joint_pose(joints)
        future.add_done_callback(
            lambda f: self.on_action_done(next_stage))

    def send_gripper(self, position, next_stage=None):
        self.action_in_progress = True
        future = self.robot.send_gripper(position)
        future.add_done_callback(
                lambda f: self.on_action_done(next_stage))

    def on_action_done(self, next_stage):
        self.action_in_progress = False
        self.is_executing = False
        self.current_stage = next_stage

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
