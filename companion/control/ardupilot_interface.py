from dataclasses import dataclass
from math import degrees
import os
import threading
from pymavlink import mavutil
import time
from collections import deque
from contextlib import contextmanager
from typing import List
import logging


class RcOverride:
    def __init__(self, rc_override_cb, frequency=200) -> None:
        self.rc_override_cb = rc_override_cb
        self.update_frequency = frequency
        self.last_update_time = 0
        self.rc_override_values = [0] * 4
        self.stop_event = threading.Event()
        self.override_lock = threading.Lock()
        self.rc_override_thread = threading.Thread(
            target=self.rc_override_update, daemon=True
        )
        self.rc_override_thread.start()

    def rc_override_update(self):
        while not self.stop_event.is_set():
            # current_time = time.time()
            # next_update_time = self.last_update_time + (1/self.update_frequency)

            # # Sleep until the next update time
            # sleep_time = max(0, next_update_time - current_time)
            # time.sleep(sleep_time)

            # Update at the desired frequency
            with self.override_lock:
                self.rc_override_cb(*self.rc_override_values)

            # Update last_update_time after each iteration
            # self.last_update_time = time.time()

    def kill(self):
        self.stop_event.set()
        self.rc_override_thread.join()

    def _validate_rc_input(self, value):
        """
        Validate RC input value to ensure it's within the range 1000-2000.

        :param value: RC input value
        :return: Clipped RC input value
        """
        return max(1000, min(2000, int(value)))

    def __call__(
        self, roll: int = None, pitch: int = None, throttle: int = None, yaw: int = None
    ):
        """
        Send control inputs to the vehicle.

        This method sends roll, pitch, throttle, and yaw values to control the vehicle.

        Args:
            roll (int): Roll angle between -1000 and 2000 degrees.
            pitch (int): Pitch angle between -1000 and 2000 degrees.
            throttle (int): Throttle value between -1000 and 2000.
            yaw (int): Yaw angle between -1000 and 2000 degrees.

        Returns:
            None

        Note:
            This method does not return any value. It only sends the input values to the vehicle.
        """
        with self.override_lock:
            self.rc_override_values[0] = (
                self._validate_rc_input(roll) if roll is not None else 0
            )
            self.rc_override_values[1] = (
                self._validate_rc_input(pitch) if pitch is not None else 0
            )
            self.rc_override_values[2] = (
                self._validate_rc_input(throttle) if throttle is not None else 0
            )
            self.rc_override_values[3] = (
                self._validate_rc_input(yaw) if yaw is not None else 0
            )


@dataclass
class Attitude:
    roll: float
    pitch: float
    yaw: float
    roll_rate: float
    pitch_rate: float
    yaw_rate: float


@dataclass
class VfrHud:
    air_speed: float  # m/s
    ground_speed: float  # m/s
    heading: int  # deg
    throttle: int  # percents
    alt: float  # m - MSL
    climb: float  # m/s


@dataclass
class VfrHud:
    air_speed: float  # m/s
    ground_speed: float  # m/s
    heading: int  # deg
    throttle: int  # percents
    alt: float  # m - MSL
    climb: float  # m/s


class ArduPilotInterface:
    NO_HEARTBEAT_TIMEOUT = 3

    def __init__(
        self,
        connection_string,
        logger: logging.Logger,
        baudrate=115200,
        system_id=255,
        reconnect_on_the_fly=True,
    ):
        """
        Initialize the ArduPilot interface.

        :param connection_string: Connection string for MAVLink (e.g., 'COM3' or '/dev/ttyAMA0')
        :param baudrate: Baudrate for the connection (default is 115200)
        :param source_system: MAVLink system ID (default is 255)
        """
        self.logger = logger
        self.threads: List[threading.Thread] = []

        # --- connection sutff --- #
        self.connection_string = connection_string
        self.baudrate = baudrate
        self.system_id = system_id
        self.stop_threads = threading.Event()

        self.ap_version = None
        self._rc_channles = None
        self._rc_rssi = None
        self._attitude: Attitude = None
        self._vfr_hud: VfrHud = None

        self._message_frequencies = {
            mavutil.mavlink.MAVLINK_MSG_ID_ATTITUDE: 200,
            mavutil.mavlink.MAVLINK_MSG_ID_VFR_HUD: 100,
            mavutil.mavlink.MAVLINK_MSG_ID_RC_CHANNELS_SCALED: 200,
        }
        self._messages_to_accept = [
            "ATTITUDE",
            "AUTOPILOT_VERSION",
            "HEARTBEAT",
            "RC_CHANNELS_SCALED",
            "VFR_HUD",
        ]

        self.time_to_trigger_connection_failsafe = 0 
        self.reconnect_on_the_fly = reconnect_on_the_fly
        self.master = None
        self.connected = False
        self.connect()
        self.start()

    def connect(self):
        """
        Establish a connection to the vehicle.
        """
        attempt = 0
        while True:
            try:
                self.logger.info(
                    f"connecting to pixhawk: {self.connection_string}, baud_rate={self.baudrate}, system_id={self.system_id}"
                )
                self.master = mavutil.mavlink_connection(
                    self.connection_string,
                    baud=self.baudrate,
                    source_system=self.system_id,
                )
                self.logger.info("Waiting For HeartBeat")
                self.master.wait_heartbeat()
                self.logger.info("Checking version...")
                self.master.mav.autopilot_version_request_send(
                    self.master.target_system, self.master.target_component
                )
                self.logger.info("Done")

                self.logger.info("Requesting extra data from pixhawk...")
                for msg, frequency in self._message_frequencies.items():
                    self.logger.debug(f"Requesting msg_id={msg}, frequency={frequency}")
                    self._send_command_long(
                        mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
                        [msg, int(1000000 / frequency)],
                    )
                self.logger.info("Done")

                self.connected = True
                self.time_to_trigger_connection_failsafe = time.time() + self.NO_HEARTBEAT_TIMEOUT
                break
            except Exception as e:
                logging.error(f"Mavlink Connection Error: {repr(e)}")
                attempt += 1
                print("could not connect on the fly. retrying...")
                time.sleep(0.5)

    def start(self):
        self.main_thread = threading.Thread(target=self.main, daemon=True)
        self.main_thread.start()
        self.threads.append(self.main_thread)

    def main(self):
        while not self.stop_threads.is_set():
            if time.time() > self.time_to_trigger_connection_failsafe:
                self.connected = False
                self.stop_threads.set()
                return

            try:
                msg = self.get_messege()
                if not msg:
                    continue
                logging.debug(msg)
                msg_type = msg.msgname
                if msg_type == "AUTOPILOT_VERSION":
                    self.version = msg.flight_custom_version
                elif msg_type == "HEARTBEAT":
                    self.time_to_trigger_connection_failsafe = time.time() + self.NO_HEARTBEAT_TIMEOUT 
                elif msg_type == "ATTITUDE":
                    self._attitude = Attitude(
                        degrees(msg.roll),
                        degrees(msg.pitch),
                        degrees(msg.yaw) % 360,
                        degrees(msg.rollspeed),
                        degrees(msg.pitchspeed),
                        degrees(msg.yawspeed),
                    )
                elif msg_type == "VFR_HUD":
                    self._vfr_hud = VfrHud(
                        msg.airspeed,
                        msg.groundspeed,
                        msg.heading,
                        msg.throttle,
                        msg.alt,
                        msg.climb,
                    )
                elif msg_type == "RC_CHANNELS_SCALED":
                    # scale to -1 - 1
                    if msg.port == 0:
                        self._rc_channles = [
                            float(msg.chan1_scaled / 10000.0),
                            float(msg.chan2_scaled / 10000.0),
                            float(msg.chan3_scaled / 10000.0),
                            float(msg.chan4_scaled / 10000.0),
                            float(msg.chan5_scaled / 10000.0),
                            float(msg.chan6_scaled / 10000.0),
                            float(msg.chan7_scaled / 10000.0),
                            float(msg.chan8_scaled / 10000.0),
                        ]
                        self._rc_rssi = float(msg.rssi / 254.0)
            except Exception as e:
                print(e)

    def get_messege(self):
        return self.master.recv_match(type=self._messages_to_accept, blocking=True)

    def set_mode(self, mode):
        """
        Set the flight mode of the vehicle without blocking for acknowledgment.

        :param mode: Mode string (e.g., 'STABILIZE', 'LOITER', 'AUTO')
        """
        with self.pixhawk_lock:
            mode_id = self.master.mode_mapping().get(mode)
            if mode_id is None:
                print(f"Unknown mode: {mode}")
                return

        print(f"Setting mode to {mode}...")
        self._send_command_long(
            mavutil.mavlink.MAV_CMD_DO_SET_MODE,
            [mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, mode_id],
        )

    @property
    def attitude(self) -> Attitude:
        return self._attitude

    @property
    def rc_channels(self):
        return self._rc_channles

    @property
    def vfr_hud(self) -> VfrHud:
        return self._vfr_hud

    @property
    def rc_rssi(self):
        return self._rc_rssi

    @contextmanager
    def rc_override(self):
        try:
            rc_override = None
            rc_override = RcOverride(rc_override_cb=self._send_rc_override)
            yield rc_override
        finally:
            if rc_override:
                rc_override.kill()

    def _send_rc_override(self, roll, pitch, throttle, yaw):
        """
        Thread function to send RC_CHANNELS_OVERRIDE messages at regular intervals.
        """
        rc_override18 = [roll, pitch, throttle, yaw] + [0] * 14
        with self.pixhawk_lock:  # TODO make sure lock is nedded
            print("sent rc override")
            self.master.mav.rc_channels_override_send(
                self.master.target_system, self.master.target_component, *rc_override18
            )

    def set_parameter(self, param_name, param_value, blocking=False,timeout=10):
        """
        Set a parameter on the vehicle and wait for acknowledgment.

        :param param_name: Name of the parameter
        :param param_value: Value to set
        :param blocking: wait for param change ack
        :param timeout: Timeout in seconds
        :return: True if parameter was set successfully, False otherwise
        """
        if param_type is None:
            # Infer param_type from param_value
            if isinstance(param_value, int):
                param_type = mavutil.mavlink.MAV_PARAM_TYPE_INT32
            elif isinstance(param_value, float):
                param_type = mavutil.mavlink.MAV_PARAM_TYPE_REAL32
            else:
                raise ValueError("Unsupported parameter value type")

        # Send PARAM_SET message
        self.master.mav.param_set_send(
            self.master.target_system,
            self.master.target_component,
            param_name.encode("utf-8"),
            float(param_value),
            param_type,
        )

        # Wait for PARAM_VALUE message with matching param_name
        start_time = time.time()
        while time.time() - start_time < timeout:
            with self.pixhawk_lock:
                msg = self.master.recv_match(
                    type="PARAM_VALUE", blocking=True, timeout=1
                )
            if msg:
                if msg.param_id.decode("utf-8").strip("\x00") == param_name:
                    # Check if the parameter value matches
                    if abs(msg.param_value - float(param_value)) < 1e-6:
                        print(f"Parameter '{param_name}' set to {param_value}")
                        return True
                    else:
                        print(
                            f"Parameter '{param_name}' value mismatch: expected {param_value}, got {msg.param_value}"
                        )
                        return False
        print(f"Timeout while setting parameter '{param_name}'")
        return False

    def _send_command_long(self, command, params):
        """
        Send a command_long message to the vehicle without blocking for acknowledgment.

        :param command: MAVLink command ID
        :param params: List of parameters for the command (up to 7 parameters)
        :param target_system: Target system ID (default is the connected vehicle)
        :param target_component: Target component ID (default is the connected component)
        """
        target_system = self.master.target_system
        target_component = self.master.target_component

        # Ensure params list has exactly 7 elements
        params += [0.0] * (7 - len(params))
        params = [0] + params

        # Send the command_long message
        self.master.mav.command_long_send(
            target_system, target_component, command, *params  # Confirmation
        )

    def __del__(self):
        """
        Stop the acknowledgment, RC override, and HUD threads.
        """
        self.stop_threads.set()
        for thread in self.threads:
            if thread and thread.is_alive():
                thread.join()


if __name__ == "__main__":
    logger = logging.getLogger("/dev/null")
    pixhawk = ArduPilotInterface("/dev/ttyACM0", logger)
    while pixhawk.connected:
        # os.system("clear")
        output = ""

        if pixhawk.vfr_hud is not None:
            output += repr(pixhawk.vfr_hud)
            output += "\n\n"

        if pixhawk.attitude is not None:
            output += repr(pixhawk.attitude)
            output += "\n\n"

        if pixhawk.rc_channels is not None:
            output += f"rc_channels({', '.join(pixhawk.rc_channels)})"
            output += "\n\n"

        print(output)
        time.sleep(0.1)
