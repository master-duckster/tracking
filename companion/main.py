import json
import time
from vision.optical_flow import OpticalFlow
from control.ardupilot_interface import ArduPilotInterface
from vision.tracker import Tracker
from control.control import TrackerPid
from app.server import WebGui
import logging
import atexit

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


class TrackingComputer():
    def __init__(self):
        self.config = load_config("config.json")

        self.tracker = Tracker(
            logger=logger,
            video_source=self.config["vision"]["video_source"],
            frame_rate=self.config["vision"]["frame_rate"],
            tracker_config=self.config["tracker_config"])

        # ardupilot_interface = ArduPilotInterface(
        #     self.config["ardupilot_interface"]["com_port"],
        #     baudrate=self.config["ardupilot_interface"]["baud_rate"],
        #     system_id=self.config["ardupilot_interface"]["system_id"],
        #     reconnect_on_the_fly=self.config["ardupilot_interface"]["reconnect_on_the_fly"],
        # )

        self.tracker_pid = TrackerPid(
            self.config["control_params"],
            logger=logger
        )


        def get_stream_frame():
            return self.tracker.frame

        self.user_interface = WebGui(video_stream_cb=get_stream_frame,
                                     set_tracking_target_cb=self.tracker.set_tracking_target)

        self.running = 1

    def run(self):
        while self.running:
            self.tracker._process_frame()
            # time.sleep(0.05)

    def __del__(self):
        self.running = False
        self.tracker.kill()

    # except Exception as e:
    #     raise(e)
    # finally()

    # ardupilot_interface.set_mode("STABILIZE")

    # with ardupilot_interface.rc_override() as rc_override:
    # while True:
    #     x, y = tracker.get_xy()
    #     tracker_pid.update_target(x, y)  # x,y range - (-1 - 1)
    #     roll, pitch = tracker_pid.get_xy_feedback()
    #     rc_override(roll=roll, pitch=pitch)
    #     tracker.process_frames()

if __name__ == "__main__":
    companion = TrackingComputer()
    atexit.register(companion.__del__)
    companion.run()
