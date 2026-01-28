#!/usr/bin/env python3

from flask import Flask, Response
import cv2
import signal
import sys

app = Flask(__name__)
cap = cv2.VideoCapture('/dev/camera_c270')  # USB camera


def generate():
    while True:
        ret, frame = cap.read()
        if not ret:
            break

        _, jpeg = cv2.imencode('.jpg', frame)
        frame_bytes = jpeg.tobytes()

        yield (
            b'--frame\r\n'
            b'Content-Type: image/jpeg\r\n\r\n' +
            frame_bytes + b'\r\n'
        )


@app.route('/video_feed')
def video_feed():
    return Response(
        generate(),
        mimetype='multipart/x-mixed-replace; boundary=frame'
    )


def shutdown_handler(sig, frame):
    cap.release()
    sys.exit(0)


if __name__ == "__main__":
    signal.signal(signal.SIGINT, shutdown_handler)
    signal.signal(signal.SIGTERM, shutdown_handler)

    app.run(host='0.0.0.0', port=5000, threaded=True)
