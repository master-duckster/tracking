import time
import cv2
import numpy as np
from sklearn.cluster import DBSCAN

# Initialize video capture from the default camera
cap = cv2.VideoCapture('output.mp4')

# Parameters for Shi-Tomasi corner detection (good features to track)
feature_params = dict(
    maxCorners=500,        # Increased number of corners to track
    qualityLevel=0.01,     # Lowered quality level to detect more features
    minDistance=10,
    blockSize=7
)

# Parameters for Lucas-Kanade optical flow
lk_params = dict(
    winSize=(50, 50),      # Increased window size for larger movements
    maxLevel=8,            # Increased pyramid levels
    criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 30, 0.01)
)

# Read the first frame and find corners in it
ret, old_frame = cap.read()
if not ret:
    print("Failed to read from camera. Exiting.")
    cap.release()
    exit()

old_gray = cv2.cvtColor(old_frame, cv2.COLOR_BGR2GRAY)

# Get frame dimensions
h, w = old_frame.shape[:2]
cross_x = w // 2
cross_y = h // 2
cross_size = int(min(h, w) * 0.1)
max_disp_x = w * 0.20
max_disp_y = h * 0.20
crosshair_velocity_scale = 1.0

# Initialize features to track
p0 = cv2.goodFeaturesToTrack(old_gray, mask=None, **feature_params)

# Check if features are detected
if p0 is None:
    print("No features found. Exiting.")
    cap.release()
    exit()

while True:
    ret, frame = cap.read()
    if not ret:
        break

    frame_gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # Calculate optical flow
    p1, st, err = cv2.calcOpticalFlowPyrLK(
        old_gray, frame_gray, p0, None, **lk_params)

    # Select good points
    if p1 is not None and st is not None:
        st = st.reshape(-1)  # Flatten the status array
        p0 = p0.reshape(-1, 1, 2)
        p1 = p1.reshape(-1, 1, 2)

        good_new = p1[st == 1]
        good_old = p0[st == 1]

        # Reshape good_new and good_old to (N, 2)
        good_new = good_new.reshape(-1, 2)
        good_old = good_old.reshape(-1, 2)

        # If too few points are being tracked, re-detect features
        if len(good_new) < 10:
            # Re-detect features
            p0 = cv2.goodFeaturesToTrack(
                frame_gray, mask=None, **feature_params)
            # Update the old frame and old points
            old_gray = frame_gray.copy()
            if p0 is None:
                continue
            else:
                continue  # Skip the rest of the loop and start with new features

        # Estimate motion
        dx = np.mean(good_new[:, 0] - good_old[:, 0])
        dy = np.mean(good_new[:, 1] - good_old[:, 1])

        # Update crosshair position
        cross_x += int(dx) * crosshair_velocity_scale
        cross_y += int(dy) * crosshair_velocity_scale

        # Limit crosshair movement
        disp_x = cross_x - (w // 2)
        disp_y = cross_y - (h // 2)
        disp_x = np.clip(disp_x, -int(max_disp_x), int(max_disp_x))
        disp_y = np.clip(disp_y, -int(max_disp_y), int(max_disp_y))
        cross_x = (w // 2) + disp_x
        cross_y = (h // 2) + disp_y

        # Clustering the points using BDSCAN
        clustering = DBSCAN(eps=int(w*0.03), min_samples=5).fit(good_new)
        labels = clustering.labels_
        unique_labels = set(labels)
        n_clusters = len(unique_labels) - (1 if -1 in labels else 0)

        for k in unique_labels:
            class_member_mask = labels == k

            xy = good_new[class_member_mask]

            if k == -1:
                color = (0, 0, 0)
            else:
                color = (255, 0, 0)

                mean_pos = np.mean(xy, axis=0).astype(int)
                # cv2.circle(frame, tuple(mean_pos), 30, color, 5)
                mean_pos = mean_pos[0] - 30, mean_pos[1] + 60
                cv2.putText(frame, str(k), tuple(mean_pos),
                            cv2.FONT_HERSHEY_SIMPLEX, 3, (0, 0, 255), 8)

            # for point in xy:
            #     a,b = point
            #     cv2.circle

        # Draw the tracked features
        # for i, (new, old) in enumerate(zip(good_new, good_old)):
        #     a, b = new
        #     cv2.circle(frame, (int(a), int(b)), 3, (0, 0, 255), -1)

        # Update the previous frame and previous points
        time.sleep(0.02)
        old_gray = frame_gray.copy()
        p0 = good_new.reshape(-1, 1, 2)

    else:
        # Re-detect features if tracking failed
        p0 = cv2.goodFeaturesToTrack(frame_gray, mask=None, **feature_params)
        old_gray = frame_gray.copy()
        if p0 is None:
            continue
        else:
            continue  # Skip the rest of the loop and start with new features

    # Ensure crosshair stays within frame boundaries
    cross_x = int(np.clip(cross_x, cross_size, w - cross_size))
    cross_y = int(np.clip(cross_y, cross_size, h - cross_size))

    print((dx + dy)**2)
    if (dx + dy)**2 < 3:
        # If no good points, reset crosshair towards center
        cross_x = int(0.98 * cross_x + 0.02 * (w // 2))
        cross_y = int(0.98 * cross_y + 0.02 * (h // 2))

    # Draw the crosshair
    cv2.line(frame, (cross_x - cross_size, cross_y),
             (cross_x + cross_size, cross_y), (0, 255, 0), 2)
    cv2.line(frame, (cross_x, cross_y - cross_size),
             (cross_x, cross_y + cross_size), (0, 255, 0), 2)

    # Display the frame with the crosshair and tracked features
    cv2.imshow('Optical Flow Crosshair with Tracked Features', frame)

    # Exit when 'q' or 'ESC' is pressed
    key = cv2.waitKey(1)
    if key == ord('q') or key == 27:
        break

# Release the capture and close windows
cap.release()
cv2.destroyAllWindows()
