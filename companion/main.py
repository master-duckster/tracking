import time
from ardupilot_interface import ArduPilotInterface

def main():
    pixhawk = ArduPilotInterface('/dev/ttyACM0')
    while True:
        print(f"ardupilot is {'connected' if pixhawk.connected else 'disconnected'}")
        with pixhawk.rc_override() as rc_override:
            roll = 1400
            pitch = 1600
            while True:
                rc_override(roll=roll, pitch=pitch)
        time.sleep(1)


if __name__ == "__main__":
    main() 