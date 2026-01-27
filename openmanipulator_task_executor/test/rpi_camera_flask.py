# rpi_camera_flask.py
from flask import Flask, Response
import cv2

app = Flask(__name__)
cap = cv2.VideoCapture('/dev/camera_c270')  # USB camera

def generate():
    while True:
        ret, frame = cap.read()
        if not ret:
            break

        _, jpeg = cv2.imencode('.jpg', frame)
        frame_bytes = jpeg.tobytes()

        yield (b'--frame\r\n'
               b'Content-Type: image/jpeg\r\n\r\n' +
               frame_bytes + b'\r\n')

@app.route('/video_feed')
def video_feed():
    return Response(
        generate(),
        mimetype='multipart/x-mixed-replace; boundary=frame'
    )

if __name__ == "__main__":
    app.run(host='0.0.0.0', port=5000, threaded=True)
