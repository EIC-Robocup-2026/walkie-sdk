#!/usr/bin/env python3
from walkie_sdk import WalkieRobot

def main():
    print("--- 3. Arm ---")
    bot = WalkieRobot(ip="127.0.0.1", camera_protocol="none")
    group = "left_arm"

    # 1. Properties
    print(f"Commands Topic: {bot.arm.arm_commands_topic}")
    print(f"States Topic: {bot.arm.arm_states_topic}")
    
    # Change control mode on the fly (ArmControlMode.MOVEIT or ArmControlMode.CUSTOM_IK)
    bot.arm.default_mode = "moveit"
    bot.arm.target_pose_topic = "/my_custom_ik_topic"

    # 2. Reading State
    states = bot.arm.get_joint_states()
    if states:
        print(f"Left Arm Joints: {states['left_arm']['positions']}")

    # 3. Direct Joint Control
    bot.arm.set_joint_positions(left_arm=[0.0]*7, right_arm=[0.0]*7, blocking=True)
    bot.arm.set_joint_velocities(left_arm=[0.1]*7) # Backend stub
    bot.arm.set_joint_torques(left_arm=[0.0]*7)    # Backend stub

    # 4. Gripper & Home Control
    bot.arm.control_gripper("left_gripper", position=0.04)
    bot.arm.go_to_home(group)

    # 5. Cartesian Pose Control (Inverse Kinematics)
    # Standard Euler
    bot.arm.go_to_pose(x=0.3, y=0.1, z=0.5, roll=0.0, pitch=1.57, yaw=0.0, group_name=group)
    
    # Relative Movement (Move from current spot)
    bot.arm.go_to_pose_relative(dx=0.05, dy=0.0, dz=-0.05, roll=0.0, pitch=0.0, yaw=0.0, group_name=group)
    
    # Quaternion (Standard)
    bot.arm.go_to_pose_quaternion(x=0.3, y=0.1, z=0.5, qx=0.0, qy=0.707, qz=0.0, qw=0.707, group_name=group)
    
    # Quaternion (via MoveGroup Action Constraints)
    bot.arm.go_to_pose_quaternion_move_action(x=0.3, y=0.1, z=0.5, qx=0, qy=0.707, qz=0, qw=0.707, group_name=group)

    bot.disconnect()

if __name__ == "__main__":
    main()