"""
ArUco Marker Pose Extraction Tool (OpenManipulator-X)
alternative for move_to_named

Purpose:
- Acquire ArUco marker pose in the robot base coordinate frame
- This pose is used as a direct replacement for predefined named poses
- Intended to provide target coordinates for Pick & Place execution

Usage (Required startup order):
1) Start hardware bringup:
   ros2 launch open_manipulator_x_bringup hardware.launch.py

2) Start MoveIt:
   ros2 launch open_manipulator_x_moveit_config moveit_core.launch.py

3) Start robot interface:
   ros2 run openmanipulator_task_executor robot_interface

4) Run this script:
   python3 aruco_place_pose_extractor.py

Controls:
- SPACE : start marker pose recording
- ESC   : quit

Behavior:
- ArUco markers are detected in real time from the camera feed
- When recording starts, multiple samples are collected
- Once enough data is gathered, a refined pose is computed
- Final pose is printed as:
  x, y, z, qx, qy, qz, qw

Notes:
- This script only estimates and prints the marker pose
- Motion execution should be handled by a separate P&P controller
- Camera is used only for vision input and visualization
"""

import cv2
import cv2.aruco as aruco
import os
import sys
import rclpy
import threading

# Path setup (ipynb 대응)
file_dir = os.path.dirname(os.path.abspath(__file__))
pkg_root = os.path.dirname(file_dir)
scripts_dir = os.path.join(pkg_root, 'scripts')
sys.path.insert(0, scripts_dir)
from ArUco_coord_extractor import MarkerPoseProcessor
from robot_interface_client import RobotInterfaceClient
#########################################################

# ROS2 init
if not rclpy.ok():
    rclpy.init()

robot = RobotInterfaceClient()
marker_processor = MarkerPoseProcessor(robot)
start_requested = False
is_executing = False
# Camera setup
cap = cv2.VideoCapture('/dev/camera_c270')
if not cap.isOpened():
    raise RuntimeError("Camera open failed")

while True:
    rclpy.spin_once(robot, timeout_sec=0.0)

    ret, frame = cap.read()
    if not ret:
        continue

    marker_processor.process_frame(frame)
    cv2.imshow("camera", frame)

    if start_requested and not is_executing:
        if marker_processor.is_ready():
            start_requested = False
            is_executing = True

            x, y, z, qx, qy, qz, qw = marker_processor.get_refined_pose()

            print("\n[REFINED PLACE POSE]")
            print(f"x y z qx qy qz qw | "
                f"{x:.4f}, {y:.4f}, {z:.4f}, "
                f"{qx:.3f}, {qy:.3f}, {qz:.3f}, {qw:.3f}")

            
            is_executing = False

    key = cv2.waitKey(1) & 0xFF
 
    if key == 27: # ESC
        break

    elif key == 32:  # SPACE
        if not start_requested and not marker_processor.is_recording():
            print("\n[MARKER RECORDING START]")
            start_requested = True
            marker_processor.start_recording()

cap.release()
cv2.destroyAllWindows()

if rclpy.ok():
    rclpy.shutdown()

print("Camera closed")