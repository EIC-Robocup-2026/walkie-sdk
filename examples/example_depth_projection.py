#!/usr/bin/env python3
"""
Walkie SDK - Depth Projection Example

Projects the ZED head depth image into the map frame as a 3D point cloud,
using the camera intrinsics (CameraInfo) and the camera pose from TF.

The pipeline:

    1. bot.camera.get_depth()        -> HxW float32 depth, metres (NaN = invalid)
    2. bot.camera.get_intrinsics()   -> fx, fy, cx, cy (cached CameraInfo)
    3. bot.transform.lookup()        -> camera pose in the map frame
    4. Back-project pixels to the camera *optical* frame, then apply the pose.

Usage:
    uv run python examples/example_depth_projection.py

Requirements:
    - Robot publishing the ZED depth, camera_info, and TF topics
    - The walkie_tf transform service running on the robot
"""

import sys
import time

import numpy as np

from walkie_sdk.utils.converters import quaternion_to_matrix

# Configuration - change to your robot's IP
ROBOT_IP = "127.0.0.1"
NAMESPACE = ""  # Optional: "robot1" for namespaced topics

# The back-projected points live in the OPTICAL frame (Z forward, X right,
# Y down) -- not the body-convention camera frame (X forward, Z up).
CAMERA_FRAME = "zed_head_left_camera_frame_optical"
WORLD_FRAME = "map"


def depth_to_points(depth: np.ndarray, intrinsics: dict) -> np.ndarray:
    """Back-project a depth image to an Nx3 cloud in the camera optical frame."""
    fx, fy = intrinsics["fx"], intrinsics["fy"]
    cx, cy = intrinsics["cx"], intrinsics["cy"]

    # CameraInfo describes the calibrated resolution; if the depth stream is
    # scaled (e.g. half-size), scale the intrinsics to match.
    h, w = depth.shape
    if (intrinsics["width"], intrinsics["height"]) != (w, h):
        sx, sy = w / intrinsics["width"], h / intrinsics["height"]
        fx, cx = fx * sx, cx * sx
        fy, cy = fy * sy, cy * sy

    us, vs = np.meshgrid(np.arange(w), np.arange(h))
    valid = np.isfinite(depth) & (depth > 0)

    z = depth[valid]
    x = (us[valid] - cx) * z / fx
    y = (vs[valid] - cy) * z / fy
    return np.column_stack([x, y, z])


def main():
    print("=" * 60)
    print("Walkie SDK - Depth Projection Example")
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
        print("❌ Camera is not available.")
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
        print("❌ No depth frame received.")
        bot.disconnect()
        sys.exit(1)

    print("[3] Fetching camera intrinsics (CameraInfo)...")
    intr = bot.camera.get_intrinsics()
    if intr is None:
        print("❌ No CameraInfo received. Check the camera_info topic is bridged:")
        print("     /zed_head/zed_node/rgb/color/rect/camera_info")
        bot.disconnect()
        sys.exit(1)
    print(f"    fx={intr['fx']:.1f} fy={intr['fy']:.1f} "
          f"cx={intr['cx']:.1f} cy={intr['cy']:.1f} "
          f"({intr['width']}x{intr['height']})")

    print(f"[4] Looking up camera pose: {WORLD_FRAME} <- {CAMERA_FRAME}...")
    pose = bot.transform.lookup(WORLD_FRAME, CAMERA_FRAME)
    if pose is None:
        print("❌ Transform lookup failed. Is the walkie_tf service running?")
        bot.disconnect()
        sys.exit(1)

    q, p = pose["quaternion"], pose["position"]
    R = quaternion_to_matrix(q["x"], q["y"], q["z"], q["w"])
    t = np.array([p["x"], p["y"], p["z"]])
    print(f"    camera at ({t[0]:.2f}, {t[1]:.2f}, {t[2]:.2f}) in '{WORLD_FRAME}'")

    print("[5] Projecting depth into the map frame...")
    points_optical = depth_to_points(depth, intr)   # Nx3, optical frame
    points_map = points_optical @ R.T + t           # Nx3, map frame

    lo, hi = points_map.min(axis=0), points_map.max(axis=0)
    print(f"    {len(points_map)} points")
    print(f"    bounds x [{lo[0]:.2f}, {hi[0]:.2f}]  "
          f"y [{lo[1]:.2f}, {hi[1]:.2f}]  z [{lo[2]:.2f}, {hi[2]:.2f}] m")

    # Single-pixel version of the same math, for the image centre:
    h, w = depth.shape
    v, u = h // 2, w // 2
    z = depth[v, u]
    if np.isfinite(z):
        centre_opt = np.array([
            (u - intr["cx"]) * z / intr["fx"],
            (v - intr["cy"]) * z / intr["fy"],
            z,
        ])
        centre_map = R @ centre_opt + t
        print(f"    centre pixel -> ({centre_map[0]:.2f}, "
              f"{centre_map[1]:.2f}, {centre_map[2]:.2f}) in '{WORLD_FRAME}'")

        # Drop a marker there so the projection can be verified in RViz2.
        bot.draw_marker(centre_map.tolist(), frame_id=WORLD_FRAME)
        print("    (published a marker at the centre point -- check RViz2)")

    bot.disconnect()


if __name__ == "__main__":
    main()
