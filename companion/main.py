import json
import time
from companion.control.ardupilot_interface import ArduPilotInterface
from companion.vision.tracker import Tracker
from control.control import TrackerPid

config_file = "config.json"


def load_config(filename):
    with open(filename, "r") as f:
        return json.load(f)


def main():
    config = load_config("config.json")

    tracker = Tracker(
        video_source=config["tracking"]["video_source"],
        frame_rate=config["tracking"]["frame_rate"],
    )

    ardupilot_interface = ArduPilotInterface(
        config["ardupilot_interface"]["com_port"],
        baudrate=config["ardupilot_interface"]["baud_rate"],
        system_id=config["ardupilot_interface"]["system_id"],
        reconnect_on_the_fly=config["ardupilot_interface"]["reconnect_on_the_fly"],
    )

    tracker_pid = TrackerPid(
        config["control_params"]["kp"],
        config["control_params"]["ki"],
        config["control_params"]["kd"],
        config["control_params"]["integral_limit"],
    )

    ardupilot_interface.set_mode("STABILIZE")

    with ardupilot_interface.rc_override() as rc_override:
        while True:
            x, y = tracker.get_xy()
            tracker_pid.update_target(x, y)  # x,y range - (-1 - 1)
            roll, pitch = tracker_pid.get_xy_feedback()
            rc_override(roll=roll, pitch=pitch)
            tracker.process_frames()
