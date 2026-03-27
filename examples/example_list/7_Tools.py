#!/usr/bin/env python3
from walkie_sdk import WalkieRobot

def main():
    print("--- 7. Tools & Vision Processing ---")
    bot = WalkieRobot(ip="127.0.0.1")

    # 1. Lifecycle Control (handled automatically by WalkieRobot usually)
    bot.tools.stop()
    bot.tools.start()

    # 2. 2D to 3D Projection
    # Format: [center_x, center_y, width, height]
    mock_bboxes = [[320, 240, 50, 50], [100, 100, 25, 25]]
    
    print("Requesting 3D positions for bounding boxes...")
    
    # Set a timeout in seconds. Returns None if the ROS node doesn't reply.
    positions = bot.tools.bboxes_to_positions(mock_bboxes, timeout=3.0)
    
    if positions:
        print(f"Success! 3D Detections received.")
    else:
        print("Timeout reached (expected if Object Pose Node isn't running).")

    bot.disconnect()

if __name__ == "__main__":
    main()