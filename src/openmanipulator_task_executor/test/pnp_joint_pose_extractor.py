"""
Joint Pose Teaching Tool (OpenManipulator-X)
alternative for move_to_named

Purpose:
- Read current joint angles from /joint_states
- Print joint values to create Pick & Place joint pose presets
- Output values can be directly used in move_to_joint_pose_and_wait()

Usage (Required startup order):
1) Start hardware bringup:
   ros2 launch open_manipulator_x_bringup hardware.launch.py

2) Start MoveIt:
   ros2 launch open_manipulator_x_moveit_config moveit_core.launch.py

3) Start robot interface:
   ros2 run openmanipulator_task_executor robot_interface

3) Disable motor torque (allow manual joint movement):
   ros2 service call /dynamixel_hardware_interface/set_dxl_torque std_srvs/srv/SetBool "{data: false}"
5) Run this script:
   python3 joint_pose_teaching.py

Controls:
- SPACE : print current joint values (teaching)
- ESC   : quit

Notes:
- Joint values are printed in MoveIt-compatible order (joint1~joint4)
- Printed lists can be copied directly into JOINT_POSES for P&P control
"""


import cv2
import os
import sys
import rclpy
from sensor_msgs.msg import JointState

# =========================================================
# Path setup
# =========================================================
file_dir = os.path.dirname(os.path.abspath(__file__))
pkg_root = os.path.dirname(file_dir)
scripts_dir = os.path.join(pkg_root, 'scripts')
sys.path.insert(0, scripts_dir)

from robot_interface_client import RobotInterfaceClient

# =========================================================
# ROS2 init
# =========================================================
if not rclpy.ok():
    rclpy.init()

robot = RobotInterfaceClient()

# =========================================================
# JointState buffer
# =========================================================
latest_joint_state = None

def joint_state_cb(msg: JointState):
    global latest_joint_state
    latest_joint_state = msg

# joint_states subscriber (ROS2 기본)
joint_sub = robot.create_subscription(
    JointState,
    '/joint_states',
    joint_state_cb,
    10
)

MOVEIT_JOINT_ORDER = [
    "joint2",
    "joint3",
    "joint1",
    "joint4",
]

HUMAN_JOINT_ORDER = [
    "joint1",
    "joint2",
    "joint3",
    "joint4",
]

# =========================================================
# Camera setup (view 확인용)
# =========================================================
cap = cv2.VideoCapture('/dev/camera_c270')
if not cap.isOpened():
    raise RuntimeError("Camera open failed")

print("\n=== Joint Pose Teaching ===")
print("[SPACE] : print current joint values")
print("[ESC]   : quit")
print("--------------------------------")

# =========================================================
# Main loop
# =========================================================
while True:
    # 🔑 ROS executor
    rclpy.spin_once(robot, timeout_sec=0.0)

    ret, frame = cap.read()
    if not ret:
        continue

    cv2.imshow("camera", frame)

    key = cv2.waitKey(1) & 0xFF

    if key == 27:  # ESC
        break

    elif key == 32:  # SPACE
        if latest_joint_state is None:
            print("[WARN] joint_states not received yet")
            continue

        joint_map = dict(
            zip(latest_joint_state.name, latest_joint_state.position)
        )

        # 1️⃣ MoveIt 기준 (실제 setJointValueTarget에 들어갈 순서)
        moveit_joints = [joint_map[name] for name in HUMAN_JOINT_ORDER]

        print("\n[MoveIt Joint Order → setJointValueTarget]")
        print(
            "joints 2, 3, 1, 4 | "
            + ", ".join(f"{v:.4f}" for v in moveit_joints)
        )

        # 2️⃣ 사람이 이해하기 쉬운 순서
        print("[Human-friendly Joint Order]")
        for name in HUMAN_JOINT_ORDER:
            print(f"{name}: {joint_map[name]:.4f}")

cap.release()
cv2.destroyAllWindows()

if rclpy.ok():
    rclpy.shutdown()

print("Camera closed")
