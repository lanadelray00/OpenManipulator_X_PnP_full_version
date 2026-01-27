#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import cv2
from std_msgs.msg import Bool
import threading


from robot_interface_client import RobotInterfaceClient
from ArUco_coord_extractor import MarkerPoseProcessor

class PickAndPlaceNode(Node):

    # Joint pose table (MoveIt joint order, radian)
    JOINT_POSES = {
        "ground_10" : [0.0752, -1.1919, 0.4556, 1.6935],
        "pick_up" : [0.0, -0.6, 0.3, 1.4], # joints 1, 2, 3, 4
        "placing_spot" : [1.6199, 0.1427, 0.2102, 1.2471]
    }
    # pick_and_place offset setting
    POSITION_OFFSET = {
        "x": 0.01,
        "y": 0.01,
        "z": -0.01,
    }
    def __init__(self):
        super().__init__('Openmanipulator_pick_and_place_node')

        # Robot
        self.robot = RobotInterfaceClient()
        self.get_logger().info("🚀 Moving to initial pose: ground_10")
        success = self.robot.move_to_joint_pose_and_wait(self.JOINT_POSES["ground_10"])
        if not success:
            self.get_logger().error("❌ Failed to move to ground_10 at startup")
            raise RuntimeError("Initial pose move failed")
        
        # Marker Processor
        self.processor = MarkerPoseProcessor(self.robot)

        # Camera
        url = "http://192.168.0.101:5000/video_feed"
        self.cap = cv2.VideoCapture(url, cv2.CAP_FFMPEG)
        if not self.cap.isOpened():
            self.get_logger().error("Camera open failed")
            raise RuntimeError("Camera open failed")

        # ★ ROS timer (30 Hz)
        self.timer = self.create_timer(1.0 / 30.0, self.loop)
        
        self.start_requested = False
        self.is_executing = False
        self.camera_running = True
        threading.Thread(target=self.camera_loop, daemon=True).start()

        self.get_logger().info("🟢 Pick & Place node started")
        self.get_logger().info("👉 Waiting for /pick_and_place/start trigger")

        # P&P Start Trigger
        self.trigger_sub = self.create_subscription(
            Bool,
            '/pick_and_place/start',
            self.trigger_cb,
            10
        )
    # Trigger method
    def trigger_cb(self, msg):
        if msg.data:
            self.get_logger().info("▶ Start trigger received")
            self.start_requested = True
            self.processor.start_recording()

    # Cam window thread
    def camera_loop(self):
        while self.camera_running:
            ret, frame = self.cap.read()
            if not ret:
                continue

            self.processor.process_frame(frame)
            cv2.imshow("camera", frame)
            cv2.waitKey(1)

    # Main P&P control loop (30 Hz trigger-based execution)
    def loop(self):
        if not self.start_requested:
            return
        if self.is_executing:
            return
        
        if self.processor.is_ready():
            self.start_requested = False
            self.is_executing = True
            pose = self.processor.get_refined_pose()
            self.execute_pick_and_place(pose)
            self.is_executing = False

    # ======================================================
    # Pick & Place Scenario Setting Zone
    # ======================================================
    def execute_pick_and_place(self, pose):
        x, y, z, qx, qy, qz, qw = pose
        x += self.POSITION_OFFSET["x"]
        y += self.POSITION_OFFSET["y"]
        z += self.POSITION_OFFSET["z"]
        
        self.get_logger().info(
            f"🎯 Target pose: "
            f"{x:.3f}, {y:.3f}, {z:.3f}"
        )
        
        # Pick
        self.robot.gripper_and_wait(0.019)
        success = self.robot.move_to_pose_and_wait(
            x, y, z, qx, qy, qz, qw
        )
        if not success:
            self.get_logger().error("❌ Move to target failed")
            return
        self.robot.gripper_and_wait(-0.004)

        # Place
        self.robot.move_to_joint_pose_and_wait(self.JOINT_POSES["pick_up"]) # up
        self.robot.move_to_joint_pose_and_wait(self.JOINT_POSES["placing_spot"]) # place 
        self.robot.gripper_and_wait(0.019)
        
        #return
        self.robot.move_to_joint_pose_and_wait(self.JOINT_POSES["ground_10"]) # return
        self.get_logger().info("✅ Pick & Place completed")


def main():
    rclpy.init()
    node = PickAndPlaceNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("🛑 Shutdown requested")
        pass
    finally:
        node.get_logger().info("Cleaning up resources")
        node.timer.cancel()
        node.cap.release()
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()