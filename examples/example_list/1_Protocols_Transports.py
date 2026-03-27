#!/usr/bin/env python3
import sys
from walkie_sdk import WalkieRobot

ROBOT_IP = "127.0.0.1"

def main():
    print("--- 1: Protocols & Mixed Camera Transports ---")
    try:
        # Connecting using rosbridge (WebSocket) and mixed camera transports
        bot = WalkieRobot(
            ip=ROBOT_IP,
            ros_protocol="rosbridge",  # Or "zenoh" / "auto"
            ros_port=9090,
            # cameras={
            #     "head": {"protocol": "zenoh"},             # Zenoh stream
            #     "wrist": {
            #         "protocol": "usb",                     # Local USB
            #         "device": "/dev/video0",
            #         "width": 640,
            #         "height": 480,
            #         "fps": 15,
            #     },
            #     "depth": {"protocol": "shm"}               # Shared Memory
            # },
            timeout=10.0,
        )
        print(f"✓ Connected to ROS via: {bot.ros_protocol}")
        print(f"✓ Connected to Cameras via: {bot.camera_protocol}")
        
        bot.disconnect()
        print("✓ Disconnected.")

    except ConnectionError as e:
        print(f"❌ Connection failed: {e}")
        sys.exit(1)

if __name__ == "__main__":
    main()