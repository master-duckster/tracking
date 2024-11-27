import cv2
import sys
import time
from time import sleep
import threading

class Tracker:
    def __init__(self, video_path, output_path="output_with_tracking.mp4", center_output="bbox_centers.txt", frame_rate=30):
        self.video_path = video_path
        self.output_path = output_path
        self.center_output = center_output  # File to store bounding box centers
        self.frame_rate = frame_rate
        self.frame_time = 1.0 / frame_rate
        self.bbox = None
        self.tracker_initialized = False
        self.fps = 0
        self.frame_count = 0
        self.timer = time.time()

        self.tracker = cv2.TrackerCSRT_create()
        # Initialize video capture
        self.video = cv2.VideoCapture(self.video_path)
        if not self.video.isOpened():
            print("Could not open video")
            sys.exit()

        # Read the first frame
        self.ok, self.frame = self.video.read()
        if not self.ok:
            print('Cannot read video file')
            sys.exit()

        # Get frame size (width and height) from the first frame
        self.frame_height, self.frame_width = self.frame.shape[:2]

        # Initialize the VideoWriter to write output video
        fourcc = cv2.VideoWriter_fourcc(*'mp4v')  # 'mp4v' is a common codec for mp4
        self.out = cv2.VideoWriter(self.output_path, fourcc, self.frame_rate, (self.frame_width, self.frame_height))

        # Open the file for writing the centers of the bounding box
        self.center_file = open(self.center_output, 'w')

        # Create OpenCV window
        cv2.imshow("Tracking", self.frame)
        # Set the mouse callback function to track the click events
        cv2.setMouseCallback("Tracking", self.click_and_draw, [self.bbox])

    def click_and_draw(self, event, x, y, flags, param):
        """Mouse callback function to capture click location and draw the bounding box."""
        if event == cv2.EVENT_LBUTTONDOWN:
            # Set the starting point when the left mouse button is clicked
            self.x_, self.y_ = x, y
            self.drawing = True
        elif event == cv2.EVENT_LBUTTONUP:
            size = 50  # Size of the rectangle
            self.bbox = (self.x_ - size // 2, self.y_ - size // 2, size, size)
            param[0] = self.bbox  

    def process_frames(self):
        """Main loop to process video frames and track the object."""
        while True:
            # Read a new frame
            self.ok, self.frame = self.video.read()
            self.frame_count += 1
            sleep(self.frame_time)  # Sleep to maintain the frame rate

            if time.time() - self.timer >= 1:  # Check if 1 second has passed
                self.fps = self.frame_count  # FPS is equal to the number of frames in 1 second
                self.frame_count = 0
                self.timer = time.time()
                
            if not self.ok:
                continue

            # Check if the user clicked and defined a bounding box
            if self.bbox is not None:
                # Initialize tracker with the first frame and bounding box
                if not self.tracker_initialized:
                    self.tracker_initialized = True
                    self.tracker.init(self.frame, self.bbox)

                # Update tracker
                ok, self.bbox = self.tracker.update(self.frame)

                # Draw bounding box if tracking is successful
                if ok:
                    p1 = self.bbox[0], self.bbox[1]
                    p2 = self.bbox[0] + self.bbox[2],  self.bbox[1] + self.bbox[3]
                    cv2.rectangle(self.frame, p1, p2, (255, 0, 0), 2, 1)
                    
                    # Calculate the center of the bounding box
                    center_x = int(self.bbox[0] + self.bbox[2] / 2)
                    center_y = int(self.bbox[1] + self.bbox[3] / 2)
                    
                    # Write the center coordinates to the output file
                    self.center_file = open(self.center_output, 'w')
                    self.center_file.write(f"{center_x},{center_y}\n")
                    self.center_file.close()  # Close the file when done
                else:
                    # Tracking failure message
                    cv2.putText(self.frame, "Tracking failure detected", (100, 80), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (0, 0, 255), 2)

            # Display FPS on frame
            cv2.putText(self.frame, "FPS : " + str(int(self.fps)), (100, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (50, 170, 50), 2)

            # Write the frame to the output video file
            self.out.write(self.frame)

            # Display result
            cv2.imshow("Tracking", self.frame)

            # Exit if ESC pressed
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
            if cv2.waitKey(1) & 0xFF == ord('r'):
                self.tracker_initialized = False
                self.bbox = None

        # Release video capture and output writer
        self.video.release()
        self.out.release()
        self.center_file.close()  # Close the file when done
        cv2.destroyAllWindows()


if __name__ == '__main__':
    # Initialize the tracker with the path to the video file and the output files
    tracker = Tracker("output.mp4", "output_with_tracking.mp4", "bbox_centers.txt")
    tracker.process_frames()
