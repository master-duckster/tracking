import logging
import cv2
from pathlib import Path


class Tracker:
    def __init__(self, logger=None, frame_dimensions=(0, 0), tracker_config={}):
        self.logger = logger or logging.getLogger(__name__)

        self.tracker_initialized = False
        self.bbox = None
        self.tracker_config = {}

        self.frame_width, self.frame_height = frame_dimensions
        self.center_x = self.frame_width//2
        self.center_y = self.frame_height//2

        # Initialize tracker with default parameters
        self.tracker = cv2.TrackerCSRT_create()

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

    def _process_frame(self, frame):
        """Process a single frame."""
        bbox = self._update_tracker(frame)
        if bbox is not None:
            return bbox
        else:
            return None

        # Draw the bounding box on the frame.

    def _update_tracker(self, frame):
        """Update the tracker with the current frame and bounding box."""
        if self.bbox is not None and not self.tracker_initialized:
            self.tracker_initialized = True
            self.tracker.init(frame, self.bbox)
            self.logger.info(
                "Tracker initialized with bounding box: %s", self.bbox)

        if self.tracker_initialized:
            ok, bbox = self.tracker.update(frame)
            if ok:
                return bbox
            else:
                print('object lost')
                self.bbox = None
                self.tracker_initialized = False
                return None
        return None
