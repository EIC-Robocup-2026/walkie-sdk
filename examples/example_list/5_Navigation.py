#!/usr/bin/env python3
import time
from walkie_sdk import WalkieRobot

def main():
    print("--- 4. Navigation ---")
    bot = WalkieRobot(ip="127.0.0.1", camera_protocol="none")

    # 1. Properties
    print(f"Action Name: {bot.nav.nav2_action_name}")
    print(f"Cmd Vel Topic: {bot.nav.cmd_vel_topic}")

    # 2. Status Tracking
    print(f"Is Navigating: {bot.nav.is_navigating}")
    print(f"Current Status: {bot.nav.status}")

    # 3. Actions
    # Go to coordinates (x, y, heading in radians)
    # Set blocking=False to continue running code while the robot moves
    print(f"Navigating to (2.0, 1.0, 1.57) with blocking=True...")
    bot.nav.go_to(x=2.0, y=1.0, heading=1.57, blocking=True)

    print(f"Navigating to (3.0, 1.0, 1.57) with blocking=False...")
    bot.nav.go_to(x=3.0, y=1.0, heading=1.57, blocking=False)

    time.sleep(5)  # Do other things while navigating

    print(f"Canceling navigation...")
    # Cancel the current goal
    bot.nav.cancel()
    
    # Emergency Stop (immediately forces zero velocity)
    bot.nav.stop()

    bot.disconnect()

if __name__ == "__main__":
    main()