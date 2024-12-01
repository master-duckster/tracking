import time
from flask_classful import FlaskView
from flask import Flask, abort, jsonify
import threading
import cv2
from time import sleep
from flask import Flask, Response, render_template_string, render_template
import os
from flask_classful import FlaskView


class WebGui():
    def __init__(self, video_stream_cb=None, set_tracking_target_cb=None) -> None:
        self.set_tracking_target_cb = set_tracking_target_cb
        self.video_stream_cb = video_stream_cb
        self.app = Flask(__name__)
        self.app_route = self.app.route
        self.app.add_url_rule('/', view_func=self.index)
        self.app.add_url_rule(
            '/config', view_func=self.config, methods=['GET'])
        self.app.add_url_rule('/video', view_func=self.video)
        self.app.add_url_rule('/set_tracking_target',
                              view_func=self.set_tracking_target, methods=['POST'])

        self.gui_thread = threading.Thread(
            target=self.app.run, daemon=True).start()

    def index(self):
        return render_template('base.html')

    def config(self):
        print(f"{locals()=}")
        config_options = {"asdfasdfas": 2}
        # if request.method == 'POST':
        #     new_config = request.form.to_dict()
        #     print(f"{new_config=}")
        # return jsonify(status='success', message='Configuration updated.')
        # else:
        return render_template('config.html', config_options=config_options, parameters={'aasdf': 1, 'aasf': 1})

    def set_tracking_target(self):
        if self.set_tracking_target_cb is not None:
            self.set_tracking_target_cb()
        return jsonify(status='success', message='Configuration updated.')

    def video(self):
        return Response(self.generate_frames(),
                        mimetype='multipart/x-mixed-replace; boundary=frame')

    def generate_frames(self):
        # Open the video file
        # cap = cv2.VideoCapture('output.mp4')
        if self.video_stream_cb is None:
            yield (b'--frame\r\n'
                   b'Content-Type: image/jpeg\r\n\r\n' + b'\r\n\r\n')

        while True:
            frame = self.video_stream_cb()
            if len(frame) < 1:
                break

            ret, jpeg = cv2.imencode('.jpg', frame)
            if not ret:
                continue

            frame_bytes = jpeg.tobytes()
            yield (b'--frame\r\n'
                   b'Content-Type: image/jpeg\r\n\r\n' + frame_bytes + b'\r\n\r\n')

        # cap.release()


if __name__ == '__main__':
    gui = WebGui()
    while 1:
        time.sleep(1)
