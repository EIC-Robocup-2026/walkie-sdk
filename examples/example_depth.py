#!/usr/bin/env python3
"""
Walkie SDK - Depth Camera Example

Demonstrates how to read the depth stream from the ZED head camera.

Depth is streamed automatically by the zenoh camera transport, alongside the
color frames -- no extra flag is needed.

    bot = WalkieRobot(ip="...", camera_protocol="zenoh")
    depth = bot.camera.get_depth()   # HxW float32, metres (NaN = invalid)

Usage:
    uv run python examples/example_depth.py

Requirements:
    - Robot publishing /zed_head/zed_node/depth/depth_registered (32FC1)
    - OpenCV installed (opencv-python), only for the optional visualisation

Controls (when a display is available):
    - Press 'q' to quit
"""

import sys
import time

import numpy as np

# Configuration - change to your robot's IP
ROBOT_IP = "127.0.0.1"
NAMESPACE = ""  # Optional: "robot1" for namespaced topics


def main():
    print("=" * 60)
    print("Walkie SDK - Depth Camera Example")
    print("=" * 60)

    from walkie_sdk import WalkieRobot

    print(f"\n[1] Connecting to {ROBOT_IP}...")
    try:
        bot = WalkieRobot(
            ip=ROBOT_IP,
            ros_protocol="zenoh",
            camera_protocol="zenoh",  # depth requires the zenoh camera
            camera_port=7447,
            namespace=NAMESPACE,
        )
    except ConnectionError as e:
        print(f"❌ Connection failed: {e}")
        sys.exit(1)

    if bot.camera is None:
        print("❌ Camera is not available (camera_protocol='none' or failed to connect).")
        bot.disconnect()
        sys.exit(1)

    # Depth frames arrive asynchronously; wait briefly for the first one.
    print("[2] Waiting for the first depth frame...")
    depth = None
    for _ in range(50):  # ~5s
        depth = bot.camera.get_depth()
        if depth is not None:
            break
        time.sleep(0.1)

    if depth is None:
        print("❌ No depth frame received. Check that the depth topic is publishing:")
        print("     /zed_head/zed_node/depth/depth_registered")
        bot.disconnect()
        sys.exit(1)

    h, w = depth.shape
    print(f"[3] Got depth frame: {w}x{h}, dtype={depth.dtype}")

    # depth[y, x] is the distance in metres (32FC1). Invalid pixels are NaN,
    # so use NaN-aware reductions.
    centre = depth[h // 2, w // 2]
    valid = depth[np.isfinite(depth)]
    print(f"    Centre pixel distance : {centre:.3f} m")
    if valid.size:
        print(f"    Nearest valid point   : {valid.min():.3f} m")
        print(f"    Farthest valid point  : {valid.max():.3f} m")

    # Optional: live colourised visualisation.
    try:
        import cv2
    except ImportError:
        print("\n(opencv-python not installed -- skipping live visualisation)")
        bot.disconnect()
        return

    print("\n[4] Showing colourised depth. Press 'q' to quit.")
    try:
        while True:
            depth = bot.camera.get_depth()
            if depth is not None:
                # Replace NaN/inf with 0 before scaling to an 8-bit colour map.
                d = np.nan_to_num(depth, nan=0.0, posinf=0.0, neginf=0.0)
                far = float(d.max()) or 1.0
                vis = cv2.applyColorMap(
                    (d / far * 255).astype(np.uint8), cv2.COLORMAP_JET
                )
                cv2.imshow("Depth (JET, near=blue)", vis)
            if cv2.waitKey(1) & 0xFF == ord("q"):
                break
    finally:
        cv2.destroyAllWindows()
        bot.disconnect()


if __name__ == "__main__":
    main()
