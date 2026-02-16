import time
from walkie_sdk import WalkieRobot


def on_arm_feedback(feedback: dict):
    """Callback to handle real-time updates from the robot."""
    print(f"\n[>> FEEDBACK] Raw Data: {feedback}")


def main():
    robot = WalkieRobot(
        ros_protocol="rosbridge",
        ip="127.0.0.1",
        ros_port=9090,
    )

    try:
        print("Sending 'Go To Pose' command to left arm...")
        robot.arm.go_to_pose(
            x=0.38,
            y=0.19,
            z=0.58,
            group_name="left_arm",
            roll=-1.57,
            pitch=0.0,
            yaw=1.57,
            cartesian_path=False,
        )

        robot.arm.control_gripper(group_name="left_gripper", position=0.7)

        print("Waiting for action to complete (Press Ctrl+C to stop)...")
        while True:
            time.sleep(1)

    except KeyboardInterrupt:
        print("\nTest stopped by user.")
    finally:
        robot.disconnect()
        print("Disconnected.")


if __name__ == "__main__":
    main()
