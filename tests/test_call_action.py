import time
from walkie_sdk.robot import WalkieRobot

def main():
    # 1. Initialize the robot connection
    # Replace '127.0.0.1' with your robot's IP address if not running locally
    robot = WalkieRobot(
        ros_protocol="rosbridge", 
        ip="127.0.0.1", 
        ros_port=9090
    )


    try:
        print("Sending 'Go To Home' command to left arm...")
        # Assuming the namespace in walkie-sdk/robot.py is configured for 'left_arm'
        robot.arm.go_to_home()

        print("Waiting for action to complete (Press Ctrl+C to stop)...")
        # Keep the script alive while the action executes in the background
        while True:
            time.sleep(1)

    except KeyboardInterrupt:
        print("\nTest stopped by user.")
    finally:
        robot.disconnect()
        print("Disconnected.")

if __name__ == "__main__":
    main()