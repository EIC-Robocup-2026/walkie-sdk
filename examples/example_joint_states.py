#!/usr/bin/env python3
"""
Walkie SDK - Print Joint States Loop

Continuously prints position, velocity, and effort for every joint reported
by the robot's joint_states topic (via the shared JointStateHub, bot.joints).

Usage:
    uv run python examples/example_joint_states.py
"""

import sys
import time

# Configuration - Change this to your robot's IP
ROBOT_IP = "127.0.0.1"
NAMESPACE = ""  # Optional: "robot1" for namespaced topics
ROS_PROTOCOL = "hybrid"
RATE_HZ = 10.0  # how often to print


def main():
    from walkie_sdk import WalkieRobot

    print(f"Connecting to {ROBOT_IP} ({ROS_PROTOCOL})...")
    try:
        bot = WalkieRobot(
            ip=ROBOT_IP,
            ros_protocol=ROS_PROTOCOL,
            ros_port=9090,
            camera_protocol="none",  # camera not needed
            timeout=10.0,
            namespace=NAMESPACE,
        )
    except ConnectionError as e:
        print(f"❌ Connection failed: {e}")
        sys.exit(1)

    print("Connected. Printing joint states (Ctrl+C to stop)...\n")
    period = 1.0 / RATE_HZ

    try:
        while True:
            joints = bot.joints.get_all()  # {name: {position, velocity, effort}}
            if not joints:
                print("⚠ No joint data yet...")
            else:
                print(f"{'joint':<24} {'pos':>10} {'vel':>10} {'eff':>10}")
                print("-" * 56)
                for name in sorted(joints):
                    s = joints[name]
                    print(
                        f"{name:<24} "
                        f"{s['position']:>10.4f} "
                        f"{s['velocity']:>10.4f} "
                        f"{s['effort']:>10.4f}"
                    )
                print()
            time.sleep(period)
    except KeyboardInterrupt:
        print("\nStopping...")
    finally:
        bot.disconnect()


if __name__ == "__main__":
    main()
