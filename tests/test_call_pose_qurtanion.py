import time
from walkie_sdk.robot import WalkieRobot

# 1. Define the Feedback Callback
# This function will run every time the robot sends an update
def on_arm_feedback(feedback: dict):
    """
    Callback to handle real-time updates from the robot.
    """
    # Print the raw dictionary to see exactly what data is coming back
    print(f"\n[>> FEEDBACK] Raw Data: {feedback}")
    
    # Example: If your action definition has specific fields like 'distance_to_goal'
    # you can access them here:
    # if 'distance_to_goal' in feedback:
    #     print(f"Distance remaining: {feedback['distance_to_goal']:.3f}m")

def main():
    # 1. Initialize the robot connection
    # Replace '127.0.0.1' with your robot's IP address if not running locally
    robot = WalkieRobot(
        ros_protocol="rosbridge", 
        ip="127.0.0.1", 
        ros_port=9090
    )

    # Wait up to 5 seconds for the connection to be established


    try:
        # Coordinates from your successful test
        x_pos = 0.38
        y_pos = 0.19
        z_pos = 0.58

        # Quaternion values for Roll: -1.57, Pitch: 0.0, Yaw: 1.57
        qx_val = -0.5
        qy_val = -0.5
        qz_val = 0.5
        qw_val = 0.5

        # Group and Link names
        group = "left_arm"
        link = "left_link7"

        print(f"Sending arm to Pose: x={x_pos}, y={y_pos}, z={z_pos} with Quaternions...")

        # Call the function you just added to arm.py
        status = robot.arm.go_to_pose_quaternion(
            x=x_pos,
            y=y_pos,
            z=z_pos,
            qx=qx_val,
            qy=qy_val,
            qz=qz_val,
            qw=qw_val,
            group_name=group,
            link_name=link,
            allowed_planning_time=10.0,
            blocking=True
        )

        print(f"Motion result: {status}")
        while True:
            time.sleep(1)

    except KeyboardInterrupt:
        print("\nTest stopped by user.")
    finally:
        robot.disconnect()
        print("Disconnected.")

if __name__ == "__main__":
    main()