#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

import cv2
import numpy as np
import cv2.aruco as aruco
from scipy.spatial.transform import Rotation
import math
import tf_transformations
import socket
import os
from collections import deque

from pnp_interfaces.msg import DetectedObject, DetectedObjectArray
from openmanipulator_task_executor.robot_interface_client import RobotInterfaceClient
from ament_index_python.packages import get_package_share_directory

class ArucoVisionNode(Node):

    def __init__(self):

        super().__init__('aruco_vision_node')

        # robot client
        self.robot = RobotInterfaceClient(self)

        # publisher
        self.pub = self.create_publisher(
            DetectedObjectArray,
            '/aruco/refined_objects',
            10
        )

        # camera
        self.ip = self.get_local_ip()
        cv2.utils.logging.setLogLevel(cv2.utils.logging.LOG_LEVEL_ERROR)
        url = f"http://{self.ip}:5000/video_feed" # check camera : http://192.168.0.106:5000/video_feed
        self.cap = cv2.VideoCapture(url, cv2.CAP_FFMPEG)

        # timer
        self.timer = self.create_timer(0.03, self.process_frame) # call process_frame every 0.03 seconds(-33Hz)

        # calibration
        package_share = get_package_share_directory('openmanipulator_task_executor')
        calib_path = os.path.join(package_share, 'config', 'calib_data.npz')
        data = np.load(calib_path)
        self.camera_matrix = data['mtx']
        self.dist_coeffs = data['dist']
        self.marker_length = 0.02

        # aruco
        self.aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
        self.detector = cv2.aruco.ArucoDetector(self.aruco_dict, cv2.aruco.DetectorParameters())

        # eye-in-hand transform
        R_cam2gripper = np.array([
            [-0.09837893,  0.16064060,  0.98209785],
            [-0.99123926, -0.10321322, -0.08241218],
            [ 0.08812674, -0.98160156,  0.16938727]
        ])

        t_cam2gripper = np.array([
            -0.05113446,
            -0.00675610,
            0.04876112
        ])

        self.T_cam2gripper = np.eye(4)
        self.T_cam2gripper[:3, :3] = R_cam2gripper
        self.T_cam2gripper[:3, 3] = t_cam2gripper

        # buffer
        self.buffer = {}
        self.buffer_size = 60

        self.no_detection_count = 0

        self.missed_counts = {}               # key별 미검출 카운트
        self.object_miss_thresh = 20          # 객체 프레임 연속 미검출 시 전체 초기화
        # Gap between objects
        self.TRACK_DIST_THRESH = 0.03  # 3cm

        # log check
        self.get_logger().info("📷 ArUco Vision Node started")

    # =========================================================

    def get_T_gripper2base(self):

        pose = self.robot.current_position
        orient = self.robot.current_orientation

        if pose is None or orient is None:
            return None

        R_gripper2base = Rotation.from_quat(orient).as_matrix()

        T = np.eye(4)
        T[:3, :3] = R_gripper2base
        T[:3, 3] = pose

        return T

    # =========================================================
    def process_frame(self):

        ret, frame = self.cap.read()

        if not ret:
            self.get_logger().warning("⚠️ Camera frame not received")
            return

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        corners, ids, _ = self.detector.detectMarkers(gray)

        if ids is None:
            self.no_detection_count += 1

            if self.no_detection_count >= self.object_miss_thresh:
                self.reset_all_tracking()

            self.publish_empty_objects()
            return
        else:
            self.no_detection_count = 0
            self.get_logger().info(f"🎯 Marker detected: {ids.flatten().tolist()}")
            
        objp = np.array([
            [-self.marker_length/2,  self.marker_length/2, 0],
            [ self.marker_length/2,  self.marker_length/2, 0],
            [ self.marker_length/2, -self.marker_length/2, 0],
            [-self.marker_length/2, -self.marker_length/2, 0],
        ], dtype=np.float32)

        aruco.drawDetectedMarkers(frame, corners, ids)

        # T_gripper2base
        T_gripper2base = self.get_T_gripper2base()
        if T_gripper2base is None:
            return
        
        T_cam2base = T_gripper2base @ self.T_cam2gripper

        seen_keys_in_frame = set()

        for i, corner in enumerate(corners):

            marker_id = int(ids[i][0])
            matched_key = None
            min_dist = float("inf")

            img_points = corner.reshape(4, 2).astype(np.float32)

            success, rvec, tvec = cv2.solvePnP(
                objp,
                img_points,
                self.camera_matrix,
                self.dist_coeffs,
                flags=cv2.SOLVEPNP_IPPE_SQUARE
            )

            if not success:
                continue

            # T_target2cam
            t_target2cam = tvec.reshape(3, 1)
            R_target2cam, _ = cv2.Rodrigues(rvec)

            T_target2cam = np.eye(4)
            T_target2cam[:3, :3] = R_target2cam
            T_target2cam[:3, 3] = t_target2cam.reshape(3)

            # T_target2base
            T_target2base = T_cam2base @ T_target2cam
            
            # coordinate result
            pos = T_target2base[:3, 3]
            quat = Rotation.from_matrix(T_target2base[:3, :3]).as_quat()
            
            x, y, z, qx, qy, qz, qw = (*pos, *quat)

            cv2.drawFrameAxes(frame, self.camera_matrix, self.dist_coeffs, rvec, tvec, self.marker_length)

            # 기존 그룹 중 같은 ID 탐색
            for key in self.buffer.keys():
                key_id, _ = key
                if key_id != marker_id:
                    continue

                prev_pos = np.array(self.buffer[key][-1][:3])
                dist = np.linalg.norm(pos - prev_pos)

                if dist < min_dist:
                    min_dist = dist
                    matched_key = key

            # threshold 이내면 같은 그룹
            if matched_key is not None and min_dist < self.TRACK_DIST_THRESH:
                self.buffer[matched_key].append((*pos, *quat))
                seen_keys_in_frame.add(matched_key)
                self.missed_counts[matched_key] = 0
            else:
                # 새 그룹 생성
                group_index = sum(1 for k in self.buffer if k[0] == marker_id)
                new_key = (marker_id, group_index)
                self.buffer[new_key] = deque(maxlen=self.buffer_size)
                self.buffer[new_key].append((*pos, *quat))
                seen_keys_in_frame.add(new_key)
                self.missed_counts[new_key] = 0
            
            # 화면 표시용 텍스트
            cX, cY = int(corners[i][0][0][0]), int(corners[i][0][0][1])
            cv2.putText(frame, f"ID:{ids[i][0]} X={x:.3f}m Y={y:.3f}m Z={z:.3f}m", (cX, cY - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)
        
        for key in list(self.buffer.keys()):
            if key not in seen_keys_in_frame:
                self.missed_counts[key] = self.missed_counts.get(key, 0) + 1

                if self.missed_counts[key] >= self.object_miss_thresh:
                    self.buffer.pop(key, None)
                    self.missed_counts.pop(key, None)
        
        # Pusblish
        refined_objects = []

        for key, poses in list(self.buffer.items()):
            if len(poses) == self.buffer_size:
                refined = self.refine_pose(poses)
                if refined is not None:
                    marker_id, group_index = key
                    refined_objects.append((marker_id, group_index, refined))

        if refined_objects:
            self.publish_objects(refined_objects)
        else:
            self.publish_empty_objects()

        cv2.imshow("aruco_vision_debug", frame) # camera debuging test용
        cv2.waitKey(1) # camera debuging test용

    # =========================================================

    def refine_pose(self, poses):
        data = np.array(poses)
        xyz = data[:, :3]

        mean = np.mean(xyz, axis=0)
        std = np.std(xyz, axis=0) + 1e-6

        z_scores = np.abs((xyz - mean) / std)
        mask = np.all(z_scores < 2.5, axis=1)

        filtered = xyz[mask]

        if len(filtered) == 0:
            return None

        filtered_std = np.std(filtered, axis=0)

        if (
            filtered_std[0] > 0.005 or
            filtered_std[1] > 0.005 or
            filtered_std[2] > 0.005
        ):
            return None

        mean_xyz = np.mean(filtered, axis=0)

        pitch = math.pi / 2
        yaw = math.atan2(mean_xyz[1], mean_xyz[0])

        q = tf_transformations.quaternion_from_euler(
            0.0,
            pitch,
            yaw
        )

        return (*mean_xyz, *q)

    # =========================================================

    def publish_objects(self, refined_objects):
        msg = DetectedObjectArray()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "world"

        for marker_id, group_index, pose in refined_objects:
            obj = DetectedObject()
            obj.marker_id = int(marker_id)
            obj.group_index = int(group_index)

            obj.pose.position.x = float(pose[0])
            obj.pose.position.y = float(pose[1])
            obj.pose.position.z = float(pose[2])

            obj.pose.orientation.x = float(pose[3])
            obj.pose.orientation.y = float(pose[4])
            obj.pose.orientation.z = float(pose[5])
            obj.pose.orientation.w = float(pose[6])

            msg.objects.append(obj)

        self.pub.publish(msg)

    def publish_empty_objects(self):
        msg = DetectedObjectArray()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "world"
        self.pub.publish(msg)

    # ======================================================
    # ip finder
    # ======================================================
    def get_local_ip(self):
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            s.connect(("8.8.8.8", 80))  # 실제로 연결되지는 않음
            self.ip = s.getsockname()[0]
        finally:
            s.close()
        return self.ip

    # ======================================================
    # reset
    # ======================================================
    def reset_all_tracking(self):
        self.get_logger().info("🧹 Reset all tracking buffers")
        self.buffer.clear()
        self.missed_counts.clear()
        self.no_detection_count = 0

# =========================================================


def main(args=None):

    rclpy.init(args=args)

    node = ArucoVisionNode()

    # rclpy.spin(node)
    # node.destroy_node()
    # rclpy.shutdown()
    
    #########################
    # # camera debuging test용
    try:
        rclpy.spin(node)
    finally:
        node.cap.release()
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()
    #########################
if __name__ == '__main__':
    main()