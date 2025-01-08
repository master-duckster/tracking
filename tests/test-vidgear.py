# import required libraries
from vidgear.gears import VideoGear
import cv2


# open any valid video stream(for e.g `myvideo.avi` file)
stream = VideoGear(source="output.mp4",stabilize=True).start()
stream_not = VideoGear(source="output.mp4",stabilize=False).start()

# loop over
while True:

    # read frames from stream
    frame = stream.read()
    frame_not = stream_not.read()

    # check for frame if Nonetype
    if frame is None or frame_not is None:
        break

    # {do something with the frame here}

    # Show output window
    cv2.imshow("Output Frame", frame)
    cv2.imshow("Output not", frame_not)

    # check for 'q' key if pressed
    key = cv2.waitKey(1) & 0xFF
    if key == ord("q"):
        break

# close output window
cv2.destroyAllWindows()

# safely close video stream
stream.stop()
stream_not.stop()