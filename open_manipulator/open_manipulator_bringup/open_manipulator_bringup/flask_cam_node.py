#!/usr/bin/env python3

from flask import Flask, Response
import cv2
import signal
import sys

app = Flask(__name__)
cap = None


def generate():
    global cap
    while True:
        ret, frame = cap.read()
        if not ret:
            break

        _, jpeg = cv2.imencode('.jpg', frame)
        yield (
            b'--frame\r\n'
            b'Content-Type: image/jpeg\r\n\r\n' +
            jpeg.tobytes() + b'\r\n'
        )


@app.route('/video_feed')
def video_feed():
    return Response(
        generate(),
        mimetype='multipart/x-mixed-replace; boundary=frame'
    )


def shutdown_handler(sig, frame):
    global cap
    if cap:
        cap.release()
    sys.exit(0)


def main():
    global cap

    cap = cv2.VideoCapture('/dev/camera_c270')
    if not cap.isOpened():
        print("❌ Camera open failed")
        sys.exit(1)

    signal.signal(signal.SIGINT, shutdown_handler)
    signal.signal(signal.SIGTERM, shutdown_handler)

    app.run(host='0.0.0.0', port=5000, threaded=True)


if __name__ == '__main__':
    main()
