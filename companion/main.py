import time
from ardupilot_interface import ArduPilotInterface
from companion.vision.tracker import Tracker

video_source = 

def main():
    pixhawk = ArduPilotInterface(params.connection_string)
    tracker_control = TrackerControl()
    tracker =  Tracker(video_source)
    pixhawk.set_mode("STABILIZE")
    
    with pixhawk.rc_override() as rc_override:
        while True:
            x,y = tracker.get_xy()
            tracker_control.update_target(x, y) #x,y range - (-1 - 1)
            roll, pitch = tracker_control.get_xy_feedback()
            rc_override(roll=roll, pitch=pitch)
            tracker.process_frames()