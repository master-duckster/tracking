import threading
from pymavlink import mavutil
import time
from collections import deque
from contextlib import contextmanager
from typing import List

class RcOverride():
    def __init__(self, rc_override_cb, frequency=200) -> None:
        self.rc_override_cb = rc_override_cb
        self.update_frequency = frequency
        self.last_update_time = 0
        self.rc_override_values = [0] * 4
        self.stop_event = threading.Event()
        self.override_lock = threading.Lock()
        self.rc_override_thread = threading.Thread(target=self.rc_override_update,daemon=True)
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

    def __call__(self, roll: int=None, pitch: int=None, throttle: int=None, yaw: int=None):
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
            self.rc_override_values[0] = self._validate_rc_input(roll) if roll is not None else 0
            self.rc_override_values[1] = self._validate_rc_input(pitch) if pitch is not None else 0
            self.rc_override_values[2] = self._validate_rc_input(throttle) if throttle is not None else 0
            self.rc_override_values[3] = self._validate_rc_input(yaw) if yaw is not None else 0

    


class ArduPilotInterface:
    NO_HEARTBEAT_TIMEOUT = 1.5

    def __init__(self, connection_string, baudrate=115200, source_system=255, reconnect_on_the_fly=True):
        """
        Initialize the ArduPilot interface.

        :param connection_string: Connection string for MAVLink (e.g., 'COM3' or '/dev/ttyAMA0')
        :param baudrate: Baudrate for the connection (default is 115200)
        :param source_system: MAVLink system ID (default is 255)
        """
        self.threads: List[threading.Thread] = []
        self.stop_event = threading.Event()

        # --- connection sutff --- #
        self.pixhawk_lock = threading.Lock()  # Lock for self.master access
        self.connection_string = connection_string
        self.baudrate = baudrate
        self.source_system = source_system
        self.master = None
        self.reconnect_on_the_fly = reconnect_on_the_fly
        self.connected = False
        self.connect()

        self.time_for_connection_failed = 0
        self.connected_lock = threading.Lock()  # Lock for self.master access

        self.start_threads()
        # Start the acknowledgment handling thread



        # self.pending_commands = deque()
        # self.acknowledged_commands = {}
        # self.lock = threading.Lock()

        # # For RC override
        # self.rc_override_values = [0] * 8  # Initialize all 8 channels to 0
        # self.rc_override_lock = threading.Lock()
        # self.rc_override_thread = None

        # # For HUD data
        # self.hud_data = {}
        # self.hud_lock = threading.Lock()
        # self.hud_thread = None

    def connect(self, blocking=True):
        """
        Establish a connection to the vehicle.
        """
        with self.pixhawk_lock:
            while blocking and not self.connected:
                try:
                    self.master = mavutil.mavlink_connection(
                        self.connection_string,
                        baud=self.baudrate,
                        source_system=self.source_system
                    )

                    self.master.wait_heartbeat()
                    self.connected = True
                    print("connected to pixhawk")
                except Exception as e:
                    print("could not connect on the fly. retrying...")
                    time.sleep(1)

    def start_threads(self):
        self.ack_thread = threading.Thread(target=self.recv_match_handler, daemon=True)
        self.ack_thread.start()
        self.threads.append(self.ack_thread)

        self.check_connection_thread = threading.Thread(target=self.connection_handler, daemon=True)
        self.check_connection_thread.start()
        self.threads.append(self.check_connection_thread)

        # Start the RC override sending thread
        # self.rc_override_thread = threading.Thread(target=self.send_rc_override, daemon=True)
        # self.rc_override_thread.start()

        # # Start the HUD data retrieval thread
        # self.hud_thread = threading.Thread(target=self.update_hud_data, daemon=True)
        # self.hud_thread.start()

    def recv_match_handler(self):
        """
        Thread function to handle incoming acknowledgments.
        """
        while not self.stop_event.is_set():
            try:
                with self.pixhawk_lock:
                    msg = self.master.recv_match(blocking=True, timeout=1)
                    # print(f"recived msg: {msg}")
                    assert msg is not None
            except Exception as e:
                continue

            msg_type = msg.get_type()
            if msg_type == 'HEARTBEAT':
                with self.connected_lock:
                    self.time_for_connection_failed = time.time() + self.NO_HEARTBEAT_TIMEOUT

            if msg_type == 'COMMAND_ACK':
                with self.lock:
                    if msg.command in self.pending_commands:
                        self.acknowledged_commands[msg.command] = msg.result
                        self.pending_commands.remove(msg.command)
                        print(f"Command {msg.command} acknowledged with result {msg.result}.")

            elif msg_type == 'PARAM_VALUE':
                # Handle parameter acknowledgments if necessary
                pass  # We'll handle parameter acknowledgments in set_parameter method
            # print(msg)

    def connection_handler(self):
        while not self.stop_event.is_set():
            time.sleep(self.NO_HEARTBEAT_TIMEOUT / 3)
            with self.connected_lock:
                if time.time() < self.time_for_connection_failed:
                    self.connected = True
                else:
                    self.connected = False
                    if self.reconnect_on_the_fly:
                        self.connect(blocking=True)

    def send_command(self, command, params):
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

        # Send the command_long message
        with self.pixhawk_lock:
            self.master.mav.command_long_send(
                target_system,
                target_component,
                command,
                0,  # Confirmation
                *params
            )

        # Add the command to the pending_commands queue
        with self.lock:
            self.pending_commands.append(command)
            self.acknowledged_commands[command] = None  # None indicates pending


    def get_command_status(self, command):
        """
        Check the acknowledgment status of a command.

        :param command: MAVLink command ID
        :return: Acknowledgment result or None if still pending
        """
        with self.lock:
            return self.acknowledged_commands.get(command)


    def arm(self):
        """
        Arm the vehicle without blocking for acknowledgment.
        """
        with self.pixhawk_lock:
            if self.master.motors_armed():
                print("Vehicle is already armed.")
                return

        print("Arming vehicle...")
        self.send_command(
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            [1]
        )

    def disarm(self):
        """
        Disarm the vehicle without blocking for acknowledgment.
        """
        with self.pixhawk_lock:
            if not self.master.motors_armed():
                print("Vehicle is already disarmed.")
                return

        print("Disarming vehicle...")
        self.send_command(
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            [0]
        )

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
        self.send_command(
            mavutil.mavlink.MAV_CMD_DO_SET_MODE,
            [mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, mode_id]
        )

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
        with self.pixhawk_lock: #TODO make sure lock is nedded
            print('sent rc override')
            self.master.mav.rc_channels_override_send(
                self.master.target_system,
                self.master.target_component,
                *rc_override18
                )


    def set_parameter(self, param_name, param_value, param_type=None, timeout=10):
        """
        Set a parameter on the vehicle and wait for acknowledgment.

        :param param_name: Name of the parameter
        :param param_value: Value to set
        :param param_type: MAV_PARAM_TYPE (optional, inferred if None)
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
        with self.pixhawk_lock:
            self.master.mav.param_set_send(
                self.master.target_system,
                self.master.target_component,
                param_name.encode('utf-8'),
                float(param_value),
                param_type
            )

        # Wait for PARAM_VALUE message with matching param_name
        start_time = time.time()
        while time.time() - start_time < timeout:
            with self.pixhawk_lock:
                msg = self.master.recv_match(type='PARAM_VALUE', blocking=True, timeout=1)
            if msg:
                if msg.param_id.decode('utf-8').strip('\x00') == param_name:
                    # Check if the parameter value matches
                    if abs(msg.param_value - float(param_value)) < 1e-6:
                        print(f"Parameter '{param_name}' set to {param_value}")
                        return True
                    else:
                        print(f"Parameter '{param_name}' value mismatch: expected {param_value}, got {msg.param_value}")
                        return False
        print(f"Timeout while setting parameter '{param_name}'")
        return False

    def get_hud_data(self):
        """
        Get the latest HUD data.

        :return: Dictionary containing HUD data
        """
        with self.hud_lock:
            return self.hud_data.copy()

    def update_hud_data(self):
        """
        Thread function to continuously update HUD data.
        """
        while not self.stop_event.is_set():
            try:
                with self.pixhawk_lock:
                    msg = self.master.recv_match(type='VFR_HUD', blocking=True, timeout=1)
                if msg:
                    with self.hud_lock:
                        self.hud_data = {
                            'airspeed': msg.airspeed,
                            'groundspeed': msg.groundspeed,
                            'heading': msg.heading,
                            'throttle': msg.throttle,
                            'altitude': msg.alt,
                            'climb': msg.climb
                        }
                time.sleep(0.1)
            except Exception as e:
                print(f"Error in HUD thread: {e}")
                continue
    

    def stop(self):
        """
        Stop the acknowledgment, RC override, and HUD threads.
        """
        self.stop_event.set()
        for thread in self.threads:
            if thread and thread.is_alive():
                thread.join()