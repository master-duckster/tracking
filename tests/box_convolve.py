import cv2
import numpy as np
from vidgear.gears import VideoGear

# Open video stream
stream = VideoGear(source="output.mp4").start()

# Box blur kernel
kernel_size = 25
def adaptive_kernel(size=10):
    # Create a circular kernel
    kernel = np.zeros((size, size))
    center = size // 2
    
    # Fill the kernel with decreasing values from the center
    for i in range(size):
        for j in range(size):
            dist = np.sqrt((i-center)**2 + (j-center)**2)
            val = 1 - (dist / (size // 2 - 1))**2
            kernel[i, j] = val
    
    return kernel / np.sum(kernel)

# Create the adaptive kernel
kernel = adaptive_kernel()

while True:
    # Read frame from stream
    frame = stream.read()

    if frame is None:
        break

    # Convert frame to uint8 format
    frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)  # Convert to grayscale if needed
    frame = frame.astype(np.uint8)

    # Apply box blur using OpenCV's filter2D function
    blurred_frame = cv2.filter2D(frame, -1, kernel)

    # Display the original and blurred frames
    cv2.imshow("Original Frame", frame)
    cv2.imshow("Blurred Frame", blurred_frame)

    # Check for 'q' key press
    key = cv2.waitKey(1) & 0xFF
    if key == ord("q"):
        break

# Close windows and stop video stream
cv2.destroyAllWindows()
stream.stop()