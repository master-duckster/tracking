import cv2
import numpy as np


def get_frame(cap):
    ret, frame = cap.read()
    if not ret:
        return None, None
    gray_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    return frame, gray_frame


class Tracker:
    def __init__(self, video_source=0):
        self.cap = cv2.VideoCapture(video_source)
        # Initialize frames
        self.prev_frame_color, self.prev_frame_gray = get_frame(self.cap)
        self.cur_frame_color, self.cur_frame_gray = get_frame(self.cap)
        self.next_frame_color, self.next_frame_gray = get_frame(self.cap)
        if self.prev_frame_gray is None or self.cur_frame_gray is None or self.next_frame_gray is None:
            print("Error: Unable to read frames from the video source.")
            exit()
        self.tracker = None
        self.bbox = None
        self.init_tracking = False

    def set_tracking_target(self, x, y):
        # Copy the current frames to avoid modifying the originals
        frame_color = self.cur_frame_color.copy()
        frame_gray = self.cur_frame_gray.copy()

        # Calculate local intensity statistics around the clicked point
        window_size = 11  # Define the window size (must be odd)
        half_window = window_size // 2
        x_start = max(0, x - half_window)
        y_start = max(0, y - half_window)
        x_end = min(frame_gray.shape[1], x + half_window + 1)
        y_end = min(frame_gray.shape[0], y + half_window + 1)

        # Extract the local window
        local_window = frame_gray[y_start:y_end, x_start:x_end]

        # Calculate mean and standard deviation
        mean_intensity = np.mean(local_window)
        std_intensity = np.std(local_window)

        # Set adaptive loDiff and upDiff based on standard deviation
        # Ensure that loDiff and upDiff are at least 1 to allow for some variation
        loDiff = max(1, int(std_intensity * 0.5))
        upDiff = max(1, int(std_intensity * 0.5))

        print(f"Adaptive loDiff: {loDiff}, upDiff: {upDiff}")

        # Create a mask for flood filling (mask dimensions are image dimensions + 2)
        h, w = frame_gray.shape[:2]
        mask = np.zeros((h + 2, w + 2), np.uint8)

        # Seed point for flood fill
        seed_point = (x, y)

        # Flags for flood fill
        connectivity = 4  # 4-connected connectivity
        flags = connectivity
        flags |= cv2.FLOODFILL_MASK_ONLY
        flags |= cv2.FLOODFILL_FIXED_RANGE
        flags |= (255 << 8)  # New color value shifted by 8 bits

        # Perform flood fill
        num, im, mask, rect = cv2.floodFill(frame_gray, mask, seedPoint=seed_point, newVal=255,
                                            loDiff=loDiff, upDiff=upDiff, flags=flags)
        # cv2.imshow("mask",mask)
        # Extract the bounding rectangle of the filled region
        x, y, w, h = rect

        # Calculate the area of the filled region using the mask
        filled_area = cv2.countNonZero(mask)

        # Set a maximum area limit (e.g., 20% of the frame area)
        frame_area = frame_gray.shape[0] * frame_gray.shape[1]
        max_area = frame_area * 0.2  # 20% of the frame area
        min_area = 100  # Minimum area to consider

        if w > 0 and h > 0 and min_area <= filled_area <= max_area:
            self.bbox = (x, y, w, h)
            # Initialize the tracker on the color frame
            self.tracker = cv2.TrackerCSRT_create()
            self.tracker.init(self.cur_frame_color, self.bbox)
            self.init_tracking = True
            print(
                "Tracking initialized with adaptive flood-filled bounding box:", self.bbox)
        else:
            print(
                "No suitable region found at the clicked point or region size out of bounds")

    def update_tracking(self):
        # Update frames
        self.prev_frame_color = self.cur_frame_color
        self.prev_frame_gray = self.cur_frame_gray
        self.cur_frame_color, self.cur_frame_gray = self.next_frame_color, self.next_frame_gray
        self.next_frame_color, self.next_frame_gray = get_frame(self.cap)

        # Check if we have a valid next frame
        if self.next_frame_gray is None:
            print("Failed to capture frame")
            return

        # Update tracking
        if self.init_tracking:
            success, box = self.tracker.update(self.cur_frame_color)
            if success:
                x, y, w, h = [int(v) for v in box]
                # Draw the bounding box on the color frame
                cv2.rectangle(self.cur_frame_color, (x, y),
                              (x + w, y + h), (0, 255, 0), 2)
                cv2.putText(self.cur_frame_color, "Tracking", (x, y - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
            else:
                print("Tracking failure")
                self.init_tracking = False  # Stop tracking if it fails

        # Display the tracking result on the color image
        cv2.imshow("Tracking", self.cur_frame_color)

    def release(self):
        self.cap.release()
        cv2.destroyAllWindows()

# Main code


def click_event(event, x, y, flags, param):
    if event == cv2.EVENT_LBUTTONDOWN:
        param.set_tracking_target(x, y)



if __name__ == "__main__":
    tracker = Tracker("output.mp4")
    
    cv2.namedWindow("Tracking")
    cv2.setMouseCallback("Tracking", click_event, tracker)

    while True:
        tracker.update_tracking()

        if cv2.waitKey(5) & 0xFF == 27:  # ESC key
            break

    tracker.release()