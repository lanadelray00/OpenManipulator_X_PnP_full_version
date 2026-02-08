#!/usr/bin/env python3

import cv2
import cv2.aruco as aruco
import numpy as np

def main():
    # 🔒 OpenCV 안정화 옵션
    cv2.setNumThreads(1)
    cv2.ocl.setUseOpenCL(False)

    # === 카메라 선택 (하나만 사용) ===
    cap = cv2.VideoCapture('/dev/camera_c270')
    # cap = cv2.VideoCapture("http://192.168.0.105:5000/video_feed", cv2.CAP_FFMPEG)
    # cap = cv2.VideoCapture("http://192.168.0.33:5000/video_feed", cv2.CAP_FFMPEG)

    print("isOpened:", cap.isOpened())
    if not cap.isOpened():
        return

    aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
    params = aruco.DetectorParameters()

    while True:
        ret, frame = cap.read()
        if not ret or frame is None:
            continue

        # ⭐ C++ segfault 방지 핵심
        frame = np.ascontiguousarray(frame)
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        print("before detectMarkers")
        # corners, ids, _ = aruco.detectMarkers(gray, aruco_dict, parameters=params)

        detector = cv2.aruco.ArucoDetector(aruco_dict, params)
        corners, ids, _ = detector.detectMarkers(gray)


        print("after detectMarkers", ids)

        if ids is not None:
            aruco.drawDetectedMarkers(frame, corners, ids)

        cv2.imshow("aruco test", frame)
        if cv2.waitKey(1) & 0xFF == 27:
            break

    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
