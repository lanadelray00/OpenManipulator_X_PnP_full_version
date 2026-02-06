import cv2
import cv2.aruco as aruco
import numpy as np

cap = cv2.VideoCapture("http://192.168.0.33:5000/video_feed")

aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
params = aruco.DetectorParameters()

while True:
    ret, frame = cap.read()
    if not ret or frame is None:
        continue

    frame = frame.copy()  # ⭐ 중요
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    corners, ids, _ = aruco.detectMarkers(gray, aruco_dict, parameters=params)
    print("ids:", ids)
