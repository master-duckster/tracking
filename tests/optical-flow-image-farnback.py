import cv2
import numpy as np

# Initialize video capture from the default camera
cap = cv2.VideoCapture('output.mp4')

# Set desired frame width and height (reduce resolution)
# frame_width = 320
# frame_height = 240
# frame_width = cap.get(cv2.CAP_PROP_FRAME_WIDTH)
# frame_height = cap.get(cv2.CAP_PROP_FRAME_HEIGHT)

# Read the first frame and convert it to grayscale
ret, prev_frame = cap.read()
if not ret:
    print("Failed to read from camera. Exiting.")
    cap.release()
    exit()

prev_frame = cv2.resize(prev_frame, (320, 240))
# Resize the frame
# prev_frame = cv2.resize(prev_frame, (frame_width, frame_height))
prev_gray = cv2.cvtColor(prev_frame, cv2.COLOR_BGR2GRAY)

# Get frame dimensions
h, w = prev_frame.shape[:2]
cross_x = w // 2
cross_y = h // 2
cross_size = int(min(h, w) * 0.1)
max_disp_x = w * 0.25
max_disp_y = h * 0.25

# Threshold for minimal camera movement
threshold = 0.5

# Read the second frame to initialize
ret, frame = cap.read()
if not ret:
    print("Failed to read from camera. Exiting.")
    cap.release()
    exit()

# Resize and convert to grayscale
frame = cv2.resize(frame, (w, h))
# curr_frame = cv2.resize(curr_frame, (w, h))

# curr_gray = cv2.cvtColor(curr_frame, cv2.COLOR_BGR2GRAY)
prev_flow_x, prev_flow_y = 0, 0
while True:
    # Compute the difference images
    # diff_prev = cv2.absdiff(prev_gray, curr_gray)

    # Read the next frame
    ret, frame = cap.read()
    if not ret:
        break

    # Resize and convert to grayscale
    frame = cv2.resize(frame, (w, h))
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    # diff_curr = cv2.absdiff(curr_gray, next_gray)

    # Optional: Apply thresholding to remove minor differences
    # _, diff_prev_thresh = cv2.threshold(diff_prev, 25, 255, cv2.THRESH_BINARY)
    # _, diff_curr_thresh = cv2.threshold(diff_curr, 25, 255, cv2.THRESH_BINARY)

    # Compute optical flow between difference images
    flow = cv2.calcOpticalFlowFarneback(
        prev=prev_gray,
        next=gray,
        flow=None,
        pyr_scale=0.5,
        levels=3,            # Reduced from 3 to 1
        winsize=15,
        iterations=1,        # Reduced from 3 to 1
        poly_n=5,
        poly_sigma=1.2,
        flags=0,
    )

    # Estimate average flow in x and y directions
    flow_x = np.mean(flow[..., 0])
    flow_y = np.mean(flow[..., 1])

    # Update crosshair position based on the flow
    if abs(flow_x) > threshold or abs(flow_y) > threshold:
        # Invert flow direction and apply scaling
        cross_x += int(flow_x)
        cross_y += int(flow_y)
    else:
        # Move crosshair slowly towards the center
        cross_x = int(0.95 * cross_x + 0.05 * (w // 2))
        cross_y = int(0.95 * cross_y + 0.05 * (h // 2))

    # accel_x, accel_y = flow_x - prev_flow_x, flow_y - prev_flow_y

    # scale_acc = 2
    # accel_x, accel_y = accel_x * scale_acc, accel_y * scale_acc
    # print(f"{accel_x=}, {accel_y=}")

    # if abs(accel_x) > threshold or abs(accel_y) > threshold:
    #     # Invert flow direction and apply scaling
    #     cross_x += int(accel_x)
    #     cross_y += int(flow_y)
    # else:
    #     # Move crosshair slowly towards the center
    #     cross_x = int(0.95 * cross_x + 0.05 * (w // 2))
    #     cross_y = int(0.95 * cross_y + 0.05 * (h // 2))

    # Limit crosshair movement to within 50% from center
    disp_x = cross_x - (w // 2)
    disp_y = cross_y - (h // 2)
    disp_x = np.clip(disp_x, -int(max_disp_x), int(max_disp_x))
    disp_y = np.clip(disp_y, -int(max_disp_y), int(max_disp_y))
    cross_x = int((w // 2) + disp_x)
    cross_y = int((h // 2) + disp_y)

    # Ensure crosshair stays within frame boundaries
    cross_x = np.clip(cross_x, cross_size, w - cross_size)
    cross_y = np.clip(cross_y, cross_size, h - cross_size)

    # Draw the crosshair on the current frame
    cv2.line(
        frame,
        (cross_x - cross_size, cross_y),
        (cross_x + cross_size, cross_y),
        (0, 255, 0),
        2,
    )
    cv2.line(
        frame,
        (cross_x, cross_y - cross_size),
        (cross_x, cross_y + cross_size),
        (0, 255, 0),
        2,
    )

    # Option to disable visualizations for better performance
    show_flow_vectors = False
    show_features = False

    if show_flow_vectors:
        # Draw the optical flow vectors
        step = 30  # Increased step size to reduce number of vectors
        scale = 5  # Scaling factor for flow vectors
        y, x = np.mgrid[step / 2: h: step, step / 2: w: step].astype(int)
        fx, fy = flow[y, x].T

        # Draw the flow vectors
        for (xi, yi, fxi, fyi) in zip(
            x.flatten(), y.flatten(), fx.flatten(), fy.flatten()
        ):
            cv2.arrowedLine(
                frame,
                (xi, yi),
                (int(xi + fxi * scale), int(yi + fyi * scale)),
                (0, 0, 255),
                1,
                tipLength=0.3,
            )

    # if show_features:
    #     # Detect features in the difference image and draw them
    #     corners = cv2.goodFeaturesToTrack(
    #         diff_curr_thresh, maxCorners=50, qualityLevel=0.02, minDistance=10
    #     )
    #     if corners is not None:
    #         for corner in corners:
    #             x_corner, y_corner = corner.ravel()
    #             cv2.circle(
    #                 curr_frame, (int(x_corner), int(
    #                     y_corner)), 3, (255, 0, 0), -1
    #             )

    # Display the frame with the crosshair
    cv2.imshow("Optical Flow on Difference Image", frame)

    # Prepare for the next iteration
    prev_gray = gray
    prev_flow_x, prev_flow_y = flow_x, flow_y
    # curr_gray = next_gray
    # curr_frame = next_frame

    # Exit when 'q' or 'ESC' is pressed
    key = cv2.waitKey(1) & 0xFF
    if key == ord("q") or key == 27:
        break

# Release the capture and close windows
cap.release()
cv2.destroyAllWindows()
