import cv2
import rclpy
from rclpy.node import Node

import threading
import numpy as np
import math
import signal
import os

from robot_interface_client import RobotInterfaceClient
from openmanipulator_task_executor.ArUco_coord_extractor import MarkerPoseProcessor


class DummyNode(Node):
    def __init__(self):
        super().__init__('markerpose_test_node')

def run_detector():

    rclpy.init()

    node = DummyNode()

    # ✅ node 전달
    robot = RobotInterfaceClient(node)

    processor = MarkerPoseProcessor(robot)

    cv2.utils.logging.setLogLevel(cv2.utils.logging.LOG_LEVEL_ERROR)
    url = "http://192.168.0.15:5000/video_feed" # 학원

    cap = cv2.VideoCapture(url, cv2.CAP_FFMPEG)

    node.get_logger().info(f"Camera opened: {cap.isOpened()}")

    node.get_logger().info("📸 Press SPACE to start recording")
    node.get_logger().info("❌ Press ESC to exit")

    while rclpy.ok():

        rclpy.spin_once(node, timeout_sec=0.01)

        ret, frame = cap.read()
        if not ret:
            continue

        processor.process_frame(frame)

        cv2.imshow("Aruco Detection", frame)
        key = cv2.waitKey(1) & 0xFF

        if key == 27:
            break

        if key == 32 and not processor.is_recording():
            node.get_logger().info("🟢 Recording started...")
            processor.start_recording()

        if processor.is_ready():
            results = processor.get_refined_poses()

            node.get_logger().info("====== REFINED RESULTS ======")
            for r in results:
                marker_id, group_index, x, y, z, qx, qy, qz, qw = r
                node.get_logger().info(
                    f"ID:{marker_id}-{group_index} | "
                    f"X:{x:.3f} Y:{y:.3f} Z:{z:.3f}"
                )
            node.get_logger().info("=============================")

    cap.release()
    cv2.destroyAllWindows()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    run_detector()