import numpy as np
import cv2


class OpticalFlow:
    def __init__(self, frame_width, frame_height) -> None:
        # params for ShiTomasi corner detection
        self.feature_params = dict(maxCorners=100,
                                   qualityLevel=0.3,
                                   minDistance=40,
                                   blockSize=40)
        # Parameters for lucas kanade optical flow
        self.lk_params = dict(winSize=(15, 15),
                         maxLevel=4,
                              criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 10, 0.03))

    def __call__(self, frame):
        frame_gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # Calculate optical flow
        if p0 is None:
            p0 = cv2.goodFeaturesToTrack(
                frame_gray, mask=None, **self.feature_params)

        iteration_count += 1

        if iteration_count % 20 == 0:  # Update feature points every 20 iterations
            p0 = cv2.goodFeaturesToTrack(
                frame_gray, mask=None, **self.feature_params)

        p1, st, err = cv2.calcOpticalFlowPyrLK(
            frame_gray, frame_gray, p0, None, **self.lk_params)

        # Select good points
        good_new = p1[st == 1]
        good_old = p0[st == 1]

        # Draw lines showing motion
        for i, ((new, old)) in enumerate(zip(good_new, good_old)):
            a, b = new.ravel()
            c, d = old.ravel()
            frame = cv2.line(frame, (a, b), (c, d), (0, 255, 0), 2)
            frame = cv2.circle(frame, (a, b), 5, (0, 0, 255), -1)

        # Update the previous frame and feature points
        frame_gray = frame_gray.copy()
        p0 = good_new.reshape(-1, 1, 2)

