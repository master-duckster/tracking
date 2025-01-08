import threading
import time
import cv2
import logging
import sys
from pathlib import Path
import os
from datetime import datetime
from .utils.tracker import Tracker


class DroneVision:
    def __init__(self, video_source, logger=None, frame_rate=30, tracker_config={}, log_video=False, video_log_path='tracker_logs/tracker_capture'):
        self.logger = logger or logging.getLogger(__name__)
        self.stop_event = threading.Event()
        self._frame_lock = threading.Lock()

        self.video_source = video_source
        self.frame_rate = frame_rate
        self.frame_duration = 1.0 / frame_rate
        self.frame_count = 0
        self._next_frame_time = 0
        self.tracker_config = {}
        self.tracker_config.update(tracker_config)

        self._initialize_video(video_source)

        # Initialize tracker with default parameters
        self.tracker = Tracker(
            logger=self.logger,
            frame_dimensions=(self.frame_width, self.frame_height),
            tracker_config=self.tracker_config,
        )

        self._log_video = log_video
        if self._log_video:
            self.output_file = self._create_output_file(video_log_path)
            fourcc = cv2.VideoWriter_fourcc(*'MJPG')
            self.out = cv2.VideoWriter(str(
                self.output_file), fourcc, self.frame_rate, (self.frame_width, self.frame_height))

        self.thread = threading.Thread(target=self.run, daemon=True)

    def run(self):
        # while not self.stop_event.is_set():
        with self._frame_lock:
            self._next_frame_time = time.time() + self.frame_duration
            self._process_frame()
        if self._log_video:
            self._write_to_video(self._frame)
        self.frame_count += 1
        time.sleep(max(0, self._next_frame_time - time.time() - 0.01))

    def set_tracking_target(self, x=0, y=0):
        self.tracker.set_tracking_target(x=x, y=y, size_percents=10)

    def get_frame(self):
        with self._frame_lock:
            return self._frame

    def _write_to_video(self, frame):
        self.out.write(frame)

    def _draw_crosser(self, frame, x, y, size=200):
        cv2.line(frame, (x, y), (x + size, y), (0, 0, 255), 3)
        cv2.line(frame, (x, y), (x - size, y), (0, 0, 255), 3)
        cv2.line(frame, (x, y), (x, y + size), (0, 0, 255), 3)
        cv2.line(frame, (x, y), (x, y - size), (0, 0, 255), 3)
        # ... (unchanged)

    def _initialize_video(self, video_source):
        """Initialize video capture."""
        self.video = cv2.VideoCapture(video_source)
        if not self.video.isOpened():
            raise ValueError("Could not open video source")

        self.ok, self._frame = self.video.read()
        if not self.ok:
            raise ValueError("Could not open video source")

        self.frame_height, self.frame_width = self._frame.shape[:2]

    def _create_output_file(self, basename):
        """Create or update the output video file name."""
        # Define the directory path
        dir_path = Path(basename).parent
        os.makedirs(str(dir_path), exist_ok=True)

        formatted_time = datetime.now().strftime('%d-%b-%Y_%H-%M-%S')

        return Path(f"{basename}_{formatted_time}.mp4")

    def _process_frame(self):
        self.ok, self._frame = self.video.read()
        self._draw_crosser(frame=self._frame,
                           x=self.frame_width//2, y=self.frame_height//2)
        if self.ok:
            bbox = self.tracker._process_frame(self._frame)
            if bbox is not None:
                p1 = (int(bbox[0]), int(bbox[1]))
                p2 = (int(bbox[0] + bbox[2]),
                      int(bbox[1] + bbox[3]))
                cv2.rectangle(self._frame, p1, p2, (0, 255, 0), 2)
        else:
            raise ValueError("could not read camera data")

    def kill(self):
        self.stop_event.set()
        if self.video.isOpened():
            self.video.release()
        if self._log_video and self.out.isOpened():
            self.out.release()
        # cv2.destroyAllWindows()


if __name__ == '__main__':
    # Initialize the tracker with the path to the video file and the output files
    drone_vision = DroneVision("output.mp4", log_video=True)
    drone_vision.start()
