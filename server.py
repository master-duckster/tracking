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
        # Read frame by frame
        ret, frame = cap.read()
        print(f"{ret=}")
        if not ret:
            break

        # Convert the frame to JPEG format
        ret, jpeg = cv2.imencode('.jpg', frame)
        if not ret:
            continue

        # Convert the image to bytes and send as response
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
