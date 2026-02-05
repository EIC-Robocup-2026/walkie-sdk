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

    # Wait up to 5 seconds for the connection to be established


    try:
        print("Sending 'Go To Home' command to left arm...")
        # Assuming the namespace in walkie-sdk/robot.py is configured for 'left_arm'
        robot.arm.go_to_home(group_name="left_arm")

        time.sleep(1)
        #dayum
        #print("Sending 'Go To Pose Relative' command to left arm...")
        #Race coonditions lul
        for i in range(20): 
            pass
            #print(robot.arm.go_to_pose_relative(group_name="left_arm", x=0.01, y=0.0, z=-0.01, roll=0.0, pitch=0.0, yaw=0.0,cartesian_path=False,blocking=False))
            #time.sleep(2)
        print("Completed relative movements.")
        """
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