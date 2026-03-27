#!/usr/bin/env python3
from walkie_sdk import WalkieRobot

def main():
    print("--- 0. Core Robot & Properties ---")
    
    # 1. Initialization
    bot = WalkieRobot(
        ip="127.0.0.1", 
        ros_protocol="rosbridge", 
        camera_protocol="zenoh",
        namespace="robot1",
        arm_mode="moveit"
    )

    # 2. Read-only Properties
    print(f"IP: {bot.ip}")
    print(f"Is Connected: {bot.is_connected}")
    print(f"ROS Protocol: {bot.ros_protocol}")
    print(f"Camera Protocol: {bot.camera_protocol}")
    
    # 3. Writable Properties
    print(f"Current Namespace: {bot.namespace}")
    bot.namespace = "robot2"  # Updates namespace across all modules dynamically
    print(f"New Namespace: {bot.namespace}")

    # 4. Top-level Convenience Functions (shortcuts for bot.viz)
    m_id = bot.draw_marker(position=[1.0, 0.0, 0.0])
    bot.update_marker(m_id, position=[2.0, 0.0, 0.0])
    
    p_topic = bot.draw_pose(position=[0.5, 0.5, 0.0])
    bot.update_pose(position=[0.5, 1.0, 0.0], topic=p_topic)

    # 5. Teardown
    bot.disconnect()

if __name__ == "__main__":
    main()