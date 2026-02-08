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
        print("Sending 'Go To Home' command to left arm...")
        # Assuming the namespace in walkie-sdk/robot.py is configured for 'left_arm'
        #robot.arm.go_to_home(group_name="left_arm")

        time.sleep(1)
        #dayum
        x_pos, y_pos, z_pos = 0.38, 0.19, 0.58
        qx_val, qy_val, qz_val, qw_val = -0.5, -0.5, 0.5, 0.5
        #print("Sending 'Go To Pose Relative' command to left arm...")
        #Race coonditions lul
        for i in range(20): 
            #print(robot.arm.go_to_pose_quaternion(group_name="left_arm", x=x_pos, y=y_pos, z=z_pos, qx=qx_val, qy=qy_val, qz=qz_val, qw=qw_val,cartesian_path=False,blocking=False,feedback_callback=on_arm_feedback))
            #robot.arm.go_to_pose(group_name="left_arm", x=0.38, y=0.19+(0.01*i), z=0.58, roll=-1.57, pitch=0.0, yaw=1.57,cartesian_path=False,blocking=False,feedback_callback=on_arm_feedback)
            print(f"Sent relative pose {i}")
            #time.sleep(0.2)
        
        print("Sending 'Go To Pose' command to left arm...")
        robot.arm.go_to_pose(
            x=0.38, 
            y=0.19, 
            z=0.58, 
            group_name="left_arm",
            roll=-1.57, 
            pitch=0.0, 
            yaw=1.57, 
            cartesian_path=False
        )
        
        """
            ros2 action send_goal /go_to_pose my_robot_interfaces/action/GoToPose "
            {
            group_name: 'left_arm',
            x: 0.38, 
            y: 0.19, 
            z: 0.58, 
            roll: -1.57, 
            pitch: 0.0, 
            yaw: 1.57, 
            cartesian_path: false
            }"
        )
        """
        robot.arm.control_gripper(group_name="left_gripper",position=0.7) # Close gripper

        #target_positions = [0.1, -0.2, 0.0, 0.5, 0.0, -0.1, 0.0]
    
        #print(f"Sending joint positions to left_arm: {target_positions}")
        #robot.arm.set_joint_positions(target_positions)

        #time.sleep(5)

        #current_states = robot.arm.get_joint_states()
        #print("all joints state: ",current_states)



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