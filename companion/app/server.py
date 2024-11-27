import cv2
from time import sleep
import numpy as np
from flask import Flask, Response, render_template_string

app = Flask(__name__)

# Path to the video file
video_path = 'output.mp4'

def generate_frames():
    # Open the video file
    cap = cv2.VideoCapture(video_path)

    while True:
        ret, frame = cap.read()
        if not ret:
            break

        ret, jpeg = cv2.imencode('.jpg', frame)
        if not ret:
            continue

        frame_bytes = jpeg.tobytes()
        sleep(1/30)
        yield (b'--frame\r\n'
               b'Content-Type: image/jpeg\r\n\r\n' + frame_bytes + b'\r\n\r\n')

    cap.release()

@app.route('/')
def index():
    return render_template_string(open('client.html').read())

@app.route('/video')
def video():
    return Response(generate_frames(),
                    mimetype='multipart/x-mixed-replace; boundary=frame')

if __name__ == '__main__':
    app.run(debug=True)
