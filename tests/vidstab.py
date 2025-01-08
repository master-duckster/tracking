import cv2
import numpy as np

# Create a TwoPassStabilizer object
stabilizer = cv2.videostab.TwoPassStabilizer()

# Set up video capture
input_path = "output.mp4"

# Read input video
vidcap = cv2.VideoCapture(input_path)
fps = vidcap.get(cv2.CAP_PROP_FPS)
width = int(vidcap.get(cv2.CAP_PROP_FRAME_WIDTH))
height = int(vidcap.get(cv2.CAP_PROP_FRAME_HEIGHT))

# Open output video writer

# Process frames
frame_count = 0
while True:
    ret, frame = vidcap.read()
    
    if not ret:
        break
    
    # Convert frame to grayscale
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    
    # Stabilize the frame
    stabilized_frame = stabilizer.stabilize(gray)
    
    # Convert back to color
    stabilized_color = cv2.cvtColor(stabilized_frame, cv2.COLOR_GRAY2BGR)
    
    # Write the stabilized frame to the output video
    
    # Display the original and stabilized frames side by side
    cv2.imshow('Original vs Stabilized', np.hstack([frame, stabilized_color]))
    
    frame_count += 1
    print(f"Processed frame {frame_count}")
    
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Release resources
vidcap.release()
cv2.destroyAllWindows()
