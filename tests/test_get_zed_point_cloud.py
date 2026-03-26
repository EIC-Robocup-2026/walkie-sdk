#!/usr/bin/env python3
"""
Walkie SDK - Example WITHOUT Camera

Demonstrates navigation and telemetry features with the new protocol selection API.
Useful when camera is not needed or not available.

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

    t_end = time.time() + 10  # Read telemetry for 10 seconds
    while time.time() < t_end:
        pc_info = bot.status.get_point_cloud_info()
        
        if pc_info:
            print("  📸 ZED Point Cloud Metadata Available:")
            print(f"    Resolution : {pc_info.get('width')}x{pc_info.get('height')}")
            
            # --- NEW: Extract the entire point cloud ---
            t0 = time.time()
            full_cloud = bot.status.get_full_point_cloud()
            t1 = time.time()
            
            if full_cloud:
                print(f"    Extracted  : {len(full_cloud)} points (took {(t1-t0)*1000:.1f} ms)")
                print(f"    First 3 Pts: {full_cloud[:3]}")
                print(f"    Last 3 Pts : {full_cloud[-3:]}")
            print("-" * 40)
        else:
            print("  ⚠ No ZED point cloud data available yet...")
        
        time.sleep(1.0)

    # 5. Disconnect
    print("\n[5] Disconnecting...")
    bot.disconnect()

    print("\n✓ Example completed!")


if __name__ == "__main__":
    main()