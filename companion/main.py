import json
import time
from control.ardupilot_interface import ArduPilotInterface
from vision.tracker import Tracker
from control.control import TrackerPid
from app.server import WebGui
import logging

config_file = "config.json"


def load_config(filename):
    with open(filename, "r") as f:
        return json.load(f)


logFormatter = logging.Formatter(
    "%(asctime)s [%(threadName)-12.12s] [%(levelname)-5.5s]  %(message)s")
logger = logging.getLogger()

# fileHandler = logging.FileHandler("{0}/{1}.log".format(logPath, fileName))
# fileHandler.setFormatter(logFormatter)
# logger.addHandler(fileHandler)

consoleHandler = logging.StreamHandler()
consoleHandler.setFormatter(logFormatter)
logger.addHandler(consoleHandler)

def main():
    config = load_config("config.json")

    tracker = Tracker(
        logger=logger,
        video_source=config["vision"]["video_source"],
        frame_rate=config["vision"]["frame_rate"],
        tracker_config=config["tracker_config"]
    )

    # ardupilot_interface = ArduPilotInterface(
    #     config["ardupilot_interface"]["com_port"],
    #     baudrate=config["ardupilot_interface"]["baud_rate"],
    #     system_id=config["ardupilot_interface"]["system_id"],
    #     reconnect_on_the_fly=config["ardupilot_interface"]["reconnect_on_the_fly"],
    # )

    tracker_pid = TrackerPid(
        config["control_params"],
        logger=logger
    )

    def get_stream_frame():
        return tracker.frame

    user_interface = WebGui(video_stream_cb=get_stream_frame,
                            set_tracking_target_cb=tracker.set_tracking_target)
    while 1:
        tracker._process_frame()
        # time.sleep(1)

    # ardupilot_interface.set_mode("STABILIZE")

    # with ardupilot_interface.rc_override() as rc_override:
    # while True:
    #     x, y = tracker.get_xy()
    #     tracker_pid.update_target(x, y)  # x,y range - (-1 - 1)
    #     roll, pitch = tracker_pid.get_xy_feedback()
    #     rc_override(roll=roll, pitch=pitch)
    #     tracker.process_frames()


if __name__ == "__main__":
    main()
