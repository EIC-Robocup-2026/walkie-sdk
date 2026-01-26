#!/usr/bin/env python3
"""
Walkie SDK - Example WITHOUT Camera

Demonstrates navigation and telemetry features with the new protocol selection API.
Useful when WebRTC camera is not available.

Usage:
    uv run python examples/example_no_camera.py
"""

import sys
import time

# Configuration - Change this to your robot's IP
ROBOT_IP = "127.0.0.1"
NAMESPACE = ""  # Optional: "robot1" for namespaced topics

# Protocol selection:
# - "rosbridge": WebSocket via roslibpy (default, no ROS2 required on client)
# - "zenoh": Zenoh DDS bridge (not yet implemented)
# - "auto": Auto-detect best available protocol
ROS_PROTOCOL = "rosbridge"


def main():
    print("=" * 60)
    print("Walkie SDK - No Camera Example")
    print("=" * 60)

    from walkie_sdk import WalkieRobot

    # 1. Connect (camera disabled)
    print(f"\n[1] Connecting to {ROBOT_IP}...")
    print(f"    ROS Protocol: {ROS_PROTOCOL}")
    print(f"    Camera: disabled")

    try:
        bot = WalkieRobot(
            ip=ROBOT_IP,
            ros_protocol=ROS_PROTOCOL,  # New protocol selection API
            ros_port=9090,
            camera_protocol="none",  # Disable camera (new API)
            timeout=10.0,
            namespace=NAMESPACE,
        )
    except ConnectionError as e:
        print(f"❌ Connection failed: {e}")
        sys.exit(1)

    print(f"    Using: {bot.ros_protocol} protocol")

    # 2. Read Telemetry
    print("\n[2] Reading telemetry...")
    time.sleep(0.3)

    pose = bot.status.get_pose()
    if pose:
        print(f"  📍 Position: x={pose['x']:.3f}, y={pose['y']:.3f}")
        print(f"  🧭 Heading:  {pose['heading']:.3f} rad")
    else:
        print("  ⚠ No pose data yet")

    vel = bot.status.get_velocity()
    if vel:
        print(
            f"  🚗 Velocity: linear={vel['linear']:.3f}, angular={vel['angular']:.3f}"
        )

    # 3. Navigation Demo
    print("\n[3] Navigation commands available:")
    print("  bot.nav.go_to(x=1.0, y=0.0, heading=0.0)")
    print("  bot.nav.cancel()")
    print("  bot.nav.stop()")

    print("\n  Navigating to (1.0, 0.0)...")
    result = bot.nav.go_to(x=1.0, y=0.0, heading=0.0, blocking=False)
    print(f"  Result: {result}")

    # 4. Monitor Loop
    print("\n[4] Monitoring until navigation completes...")
    i = 0
    while bot.nav.status != "SUCCEEDED":
        pose = bot.status.get_pose()
        if pose:
            print(
                f" [{i + 1}s] Status={bot.nav.status} x={pose['x']:+6.2f}  y={pose['y']:+6.2f}  θ={pose['heading']:+5.2f}"
            )
        i += 1
        time.sleep(1.0)

        # Safety timeout
        if i > 60:
            print("  ⚠ Navigation timeout, stopping...")
            bot.nav.stop()
            break

    # 5. Disconnect
    print("\n[5] Disconnecting...")
    bot.disconnect()

    print("\n✓ Example completed!")


if __name__ == "__main__":
    main()
