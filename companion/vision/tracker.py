from datetime import datetime
import os
import cv2
import threading
import time
import sys
from pathlib import Path
import logging


class Tracker:
    """
    A class for tracking objects in video streams using OpenCV's CSRT tracker.

    Attributes:
        video_source (str): Source of the video stream.
        frame_rate (int): Desired frame rate.
        frame_time (float): Time per frame.
        frame_count (int): Number of frames processed.
        timer (float): Timer for FPS calculation.
        tracker_initialized (bool): Flag indicating tracker initialization.
        bbox (tuple): Current bounding box coordinates.
        video (cv2.VideoCapture): Video capture object.
        out (cv2.VideoWriter): Video output writer.
        stop_event (threading.Event): Event for stopping the main thread.
    """

    def __init__(self, video_source, logger=None, frame_rate=30, show_video_window=False, tracker_config={}, video_log_path='tracker_logs/tracker_capture'):
        self.logger = logger or logging.getLogger(__name__)
        self.stop_event = threading.Event()
        self._frame_lock = threading.Lock()

        self.open_video_window = show_video_window
        self.video_source = video_source
        self.frame_rate = frame_rate
        self.frame_time = 1.0 / frame_rate
        self.frame_count = 0
        self.timer = time.time()
        self.tracker_initialized = False
        self.bbox = None
        self.drawing = False
        self.tracker_config = {}
        self.tracker_config.update(tracker_config)

        self.center_x = None
        self.center_y = None

        # Initialize tracker with default parameters
        self.tracker = cv2.TrackerCSRT_create()
        self._initialize_video(video_source)
        self.output_file = self._create_output_file(video_log_path)
        fourcc = cv2.VideoWriter_fourcc(*'MJPG')
        self.out = cv2.VideoWriter(str(
            self.output_file), fourcc, self.frame_rate, (self.frame_width, self.frame_height))

        if self.open_video_window:
            cv2.setMouseCallback("Tracking", self.click_and_draw)


    def _draw_crosser(self, frame, x, y, size=200):
        cv2.line(frame, (x, y), (x + size, y), (0, 0, 255), 3)
        cv2.line(frame, (x, y), (x - size, y), (0, 0, 255), 3)
        cv2.line(frame, (x, y), (x, y + size), (0, 0, 255), 3)
        cv2.line(frame, (x, y), (x, y - size), (0, 0, 255), 3)

    def _initialize_video(self, video_source):
        """Initialize video capture."""
        self.video = cv2.VideoCapture(video_source)
        if not self.video.isOpened():
            self.logger.error("Could not open video source")
            sys.exit()

        self.ok, self._frame = self.video.read()
        if not self.ok:
            self.logger.error('Cannot read video file')
            sys.exit()

        self.frame_height, self.frame_width = self._frame.shape[:2]

    def _create_output_file(self, basename):
        """Create or update the output video file name."""
        # Define the directory path
        dir_path = Path(basename).parent
        os.makedirs(str(dir_path), exist_ok=True)

        formatted_time = datetime.now().strftime('%d-%b-%Y_%H-%M-%S')

        return Path(f"{basename}_{formatted_time}.mp4")

    def set_tracking_target(self, x=0, y=0, size_percents=10):
        """
        Set the tracking target position and size.

        Args:
            x (float): Normalized x-coordinate (-1 to 1). Default is 0.
            y (float): Normalized y-coordinate (-1 to 1). Default is 0.
            size_percents (int): Percentage of frame width for target size. Default is 10%.

        Returns:
            tuple: A bounding box (left, top, width, height) representing the tracking target area.
        """
        # Normalize x and y coordinates if they are outside -1 to 1 range
        x = max(-1, min(x, 1))
        y = max(-1, min(y, 1))

        # Calculate target size based on percentage of frame width
        size = size_percents * self.frame_width // 100

        # Calculate target position
        pixels_x = ((x + 1) * self.frame_width // 2) - size // 2
        pixels_y = ((y + 1) * self.frame_height // 2) - size // 2

        # Ensure target area is within the frame boundaries
        left = max(0, min(pixels_x, self.frame_width - size))
        top = max(0, min(pixels_y, self.frame_height - size))
        width = min(size, self.frame_width - left)
        height = min(size, self.frame_height - top)

        self.logger.info(f"Tracking target: {left}, {top}, {width}, {height}")
        self.tracker_initialized = False
        self.bbox = (int(left), int(top), int(width), int(height))
        # return (left, top, width, height)

    def click_and_draw(self, event, x, y, flags, param):
        """
        Mouse callback function to capture click location and draw the bounding box.

        Args:
            event (int): Type of event.
            x (int): X-coordinate of click.
            y (int): Y-coordinate of click.
            flags (int): Event flags.
            param (list): List containing current bbox.
        """
        if event == cv2.EVENT_LBUTTONDOWN:
            self.x_, self.y_ = x, y
            self.drawing = True
        elif event == cv2.EVENT_LBUTTONUP and self.drawing:
            size = 50
            self.bbox = (self.x_ - size // 2, self.y_ - size // 2, size, size)
            self.drawing = False


    def _process_frame(self):
        """Process a single frame."""
        self.ok, self._frame = self.video.read()
        if not self.ok:
            self.out.release()
            self.logger.warning("No frame read from video source")
            return

        self._update_tracker()

        self._draw_crosser(self._frame, self.frame_width //
                           2, self.frame_height//2, int(self.frame_width*0.05))
        # Draw the bounding box on the frame.
        if self.bbox:
            p1 = (int(self.bbox[0]), int(self.bbox[1]))
            p2 = (int(self.bbox[0] + self.bbox[2]),
                  int(self.bbox[1] + self.bbox[3]))
            cv2.rectangle(self._frame, p1, p2, (0, 255, 0), 2)

        # Write the frame to the output video file.
        self.out.write(self._frame)
        if self.open_video_window:
            cv2.imshow("Tracking", self._frame)
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                self.stop_event.set()

        self.frame = self._frame

    def _update_tracker(self):
        """Update the tracker with the current frame and bounding box."""
        if self.bbox is not None and not self.tracker_initialized:
            self.tracker_initialized = True
            self.tracker.init(self._frame, self.bbox)
            self.logger.info(
                "Tracker initialized with bounding box: %s", self.bbox)

        if self.tracker_initialized:
            ok, self.bbox = self.tracker.update(self._frame)

            if ok:
                p1 = (int(self.bbox[0]), int(self.bbox[1]))
                p2 = (int(self.bbox[0] + self.bbox[2]),
                      int(self.bbox[1] + self.bbox[3]))
                cv2.rectangle(self._frame, p1, p2, (255, 0, 0), 2, 1)
            else:
                cv2.putText(self._frame, "Tracking failure detected", (100, 80),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.75, (0, 0, 255), 2)
                self.logger.warning("Tracking failure detected")

            if self.bbox and self.tracker_initialized:
                self.center_x = int(self.bbox[0] + self.bbox[2] / 2)
                self.center_y = int(self.bbox[1] + self.bbox[3] / 2)

    def kill(self):
        self.stop_event.set()
        if self.video.isOpened():
            self.video.release()
        if self.out.isOpened():
            print("closed video peacefully")
            self.out.release()
        cv2.destroyAllWindows()


if __name__ == '__main__':
    # Initialize the tracker with the path to the video file and the output files
    tracker = Tracker(
        "output.mp4", "output_with_tracking.avi", "bbox_centers.txt")
    tracker.process_frames()
