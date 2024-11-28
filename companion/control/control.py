import logging
import math
import os
import time
from simple_pid import PID
import json

class TrackerPid:
    pid_params = {
            'Kp': 1.0,
            'Ki': 0.0,
            'Kd': 0.0,
            'setpoint': 0,
            'sample_time': 0.01,
            'output_limits': (-1, 1),
            'auto_mode': True,
            'proportional_on_measurement': True,
            'differential_on_measurement': True,
            'error_map': None,
            'time_fn': None,
            'starting_output': 0.0,
    }

    def __init__(self, control_params, logger: logging.Logger):
        self.logger = logger
        logger.info("creating pid controllers...")
        self.pid_params.update(control_params)
        self.roll_pid = PID(**self.pid_params)
        self.pitch_pid = PID(**self.pid_params)
        logger.info(f"roll pid -> {repr(self.roll_pid)}")
        logger.info(f"roll pid -> {repr(self.pitch_pid)}")

        self._x_value = None
        self._y_value = None
        self.roll_output = None
        self.pitch_output = None
    
    @property
    def x_value(self):
        return self._x_value
   
    @x_value.setter
    def x_value(self, value):
        self._x_value = min(1, max(-1, value))
    
    @property
    def y_value(self):
        return self._y_value

    @y_value.setter
    def y_value(self, value):
        self._y_value = min(1, max(-1, value))
    
    def __call__(self, x, y):
        self.x_value, self.y_value = x, y
        self.roll_output = self.roll_pid(self.x_value)
        self.pitch_output = self.pitch_pid(self.y_value)
        
        self.logger.debug(f"roll_pid({self.x_value}) = {self.roll_output}")
        self.logger.debug(f"roll_pid({self.y_value}) = {self.pitch_output}")

        return self.roll_output, self.pitch_output


if  __name__ == "__main__":
    logger = logging.getLogger('/dev/null')
    logging.basicConfig(level=logging.NOTSET,format="")
    controller = TrackerPid({}, logger=logger)
    while 1:
        logger.info(f"Roll: {controller.roll_output}, Pitch: {controller.pitch_output}")
        controller(math.cos(time.monotonic()), math.sin(time.monotonic()))
        time.sleep(0.1)
        os.system('clear')