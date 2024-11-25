import time
from ardupilot_interface import ArduPilotInterface

def main():
    pixhawk = ArduPilotInterface('/dev/ttyACM0')
    while True:
        time.sleep(1)


if __name__ == "__main__":
    main() 