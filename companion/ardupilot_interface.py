import threading
from pymavlink import mavutil
import time
from collections import deque
from contextlib import contextmanager

class ArduPilotInterface:
    def __init__(self, connection_string, baudrate=115200, source_system=255):
        """
        Initialize the ArduPilot interface.

        :param connection_string: Connection string for MAVLink (e.g., 'COM3' or '/dev/ttyAMA0')
        :param baudrate: Baudrate for the connection (default is 115200)
        :param source_system: MAVLink system ID (default is 255)
        """
        self.master_lock = threading.Lock()  # Lock for self.master access
        self.connection_string = connection_string
        self.baudrate = baudrate
        self.source_system = source_system
        self.master = None
        self.connect()


        self.stop_event = threading.Event()
        self.ack_thread = None

        self.no_heartbeat_timeout = 1
        self.check_connection_thread = None
        self.connected_lock = threading.Lock()  # Lock for self.master access
        self.connected = False

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


    def connect(self):
        """
        Establish a connection to the vehicle.
        """
        with self.master_lock:
            self.master = mavutil.mavlink_connection(
                self.connection_string,
                baud=self.baudrate,
                source_system=self.source_system
            )

            self.master.wait_heartbeat()
            print("connected to pixhawk")

    def start_threads(self):
        self.ack_thread = threading.Thread(target=self.recv_match_handler, daemon=True)
        self.ack_thread.start()

        self.check_connection_thread = threading.Thread(target=self.update_connection_state, daemon=True)
        self.check_connection_thread.start()
        # Start the RC override sending thread
        # self.rc_override_thread = threading.Thread(target=self.send_rc_override, daemon=True)
        # self.rc_override_thread.start()

        # # Start the HUD data retrieval thread
        # self.hud_thread = threading.Thread(target=self.update_hud_data, daemon=True)
        # self.hud_thread.start()

    def update_connection_state(self):
        with self.connected_lock:
            if time.time() - self.last_heartbeat_time > self.no_heartbeat_timeout:
                self.connected = False
            else:
                self.connected = True

    def send_command(self, command, params, target_system=None, target_component=None):
        """
        Send a command_long message to the vehicle without blocking for acknowledgment.

        :param command: MAVLink command ID
        :param params: List of parameters for the command (up to 7 parameters)
        :param target_system: Target system ID (default is the connected vehicle)
        :param target_component: Target component ID (default is the connected component)
        """
        if target_system is None:
            target_system = self.master.target_system
        if target_component is None:
            target_component = self.master.target_component

        # Ensure params list has exactly 7 elements
        params += [0.0] * (7 - len(params))

        # Send the command_long message
        with self.master_lock:
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

    def recv_match_handler(self):
        """
        Thread function to handle incoming acknowledgments.
        """
        while not self.stop_event.is_set():
            try:
                with self.master_lock:
                    msg = self.master.recv_match(blocking=True, timeout=1)
                    print(f"recived msg: {msg}")
                if msg:
                    msg_type = msg.get_type()
                    if msg_type == 'HEARTBEAT':
                        self.last_heartbeat_time = time.time()
                    if msg_type == 'COMMAND_ACK':
                        with self.lock:
                            if msg.command in self.pending_commands:
                                self.acknowledged_commands[msg.command] = msg.result
                                self.pending_commands.remove(msg.command)
                                print(f"Command {msg.command} acknowledged with result {msg.result}.")
                    elif msg_type == 'PARAM_VALUE':
                        # Handle parameter acknowledgments if necessary
                        pass  # We'll handle parameter acknowledgments in set_parameter method
            except Exception as e:
                print(f"Error in acknowledgment thread: {e}")
                continue

    def get_command_status(self, command):
        """
        Check the acknowledgment status of a command.

        :param command: MAVLink command ID
        :return: Acknowledgment result or None if still pending
        """
        with self.lock:
            return self.acknowledged_commands.get(command)

    def stop(self):
        """
        Stop the acknowledgment, RC override, and HUD threads.
        """
        self.stop_event.set()
        if self.ack_thread and self.ack_thread.is_alive():
            self.ack_thread.join()
        if self.rc_override_thread and self.rc_override_thread.is_alive():
            self.rc_override_thread.join()
        if self.hud_thread and self.hud_thread.is_alive():
            self.hud_thread.join()

    def arm(self):
        """
        Arm the vehicle without blocking for acknowledgment.
        """
        with self.master_lock:
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
        with self.master_lock:
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
        with self.master_lock:
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
    def move_vehicle(self, roll=None, pitch=None, throttle=None, yaw=None):
        """
        Context manager to send RC_CHANNELS_OVERRIDE messages to control the vehicle's movement.
        When the context is exited, it stops commanding the vehicle.

        Channel Mapping:
        - Channel 1: Roll
        - Channel 2: Pitch
        - Channel 3: Throttle
        - Channel 4: Yaw

        :param roll: Roll input (channel 1), range 1000-2000
        :param pitch: Pitch input (channel 2), range 1000-2000
        :param throttle: Throttle input (channel 3), range 1000-2000
        :param yaw: Yaw input (channel 4), range 1000-2000
        """
        try:
            self._start_rc_override(roll, pitch, throttle, yaw)
            yield
        finally:
            self._stop_rc_override()

    def _start_rc_override(self, roll=None, pitch=None, throttle=None, yaw=None):
        """
        Start sending RC override commands.

        :param roll: Roll input (channel 1), range 1000-2000
        :param pitch: Pitch input (channel 2), range 1000-2000
        :param throttle: Throttle input (channel 3), range 1000-2000
        :param yaw: Yaw input (channel 4), range 1000-2000
        """
        with self.rc_override_lock:
            # Update the RC override values for the specified channels
            self.rc_override_values[0] = self._validate_rc_input(roll) if roll is not None else 0
            self.rc_override_values[1] = self._validate_rc_input(pitch) if pitch is not None else 0
            self.rc_override_values[2] = self._validate_rc_input(throttle) if throttle is not None else 0
            self.rc_override_values[3] = self._validate_rc_input(yaw) if yaw is not None else 0

    def _stop_rc_override(self):
        """
        Stop sending RC override commands by resetting the override values.
        """
        with self.rc_override_lock:
            self.rc_override_values = [0] * 8  # Reset all channels to 0 (no override)

    def send_rc_override(self):
        """
        Thread function to send RC_CHANNELS_OVERRIDE messages at regular intervals.
        """
        while not self.stop_event.is_set():
            with self.rc_override_lock:
                # Send the RC_CHANNELS_OVERRIDE message
                with self.master_lock:
                    self.master.mav.rc_channels_override_send(
                        self.master.target_system,
                        self.master.target_component,
                        *self.rc_override_values
                    )
            time.sleep(0.1)  # Send at 10 Hz

    def _validate_rc_input(self, value):
        """
        Validate RC input value to ensure it's within the range 1000-2000.

        :param value: RC input value
        :return: Clipped RC input value
        """
        return max(1000, min(2000, int(value)))

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
        with self.master_lock:
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
            with self.master_lock:
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
                with self.master_lock:
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