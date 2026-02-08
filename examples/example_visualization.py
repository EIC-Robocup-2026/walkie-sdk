"""
Example: Visualization Markers, Poses, and Axis Triads in RViz2

Demonstrates how to publish visualization markers, PoseStamped messages,
and axis triads using the Walkie SDK. These are visible in RViz2.

RViz2 Setup:
  1. Open RViz2
  2. Add a "Marker" display, set topic to /walkie/viz_markers
  3. Add a "MarkerArray" display, set topic to /walkie/viz_axis
  4. Add a "Pose" display, set topic to /walkie/target_pose
  5. Markers, poses, and axis triads will appear in the 3D view

Usage:
    python example_visualization.py
"""

import math
import time

from walkie_sdk import WalkieRobot, ARROW, CUBE, SPHERE, CYLINDER, TEXT_VIEW_FACING

# Connect to the robot
robot = WalkieRobot(
    ip="127.0.0.1",
    camera_protocol="none",  # No camera needed for this example
)

# --- Single Marker Examples ---

# 1. Draw a red arrow at (1, 0, 0) in base_link frame (default)
marker_id = robot.draw_marker(
    position=[1.0, 0.0, 0.0],
    quaternion=[0.0, 0.0, 0.0, 1.0],
)
print(f"Drew arrow marker with id={marker_id}")

# 2. Draw a green sphere at (2, 1, 0.5)
sphere_id = robot.draw_marker(
    position=[2.0, 1.0, 0.5],
    quaternion=[0.0, 0.0, 0.0, 1.0],
    marker_type=SPHERE,
    color=[0.0, 1.0, 0.0, 0.8],  # green, slightly transparent
    scale=[0.2, 0.2, 0.2],
)
print(f"Drew sphere marker with id={sphere_id}")

# 3. Draw a blue cube at (0, 2, 0) in map frame
marker_id = robot.draw_marker(
    position=[0.0, 2.0, 0.0],
    quaternion=[0.0, 0.0, 0.0, 1.0],
    frame_id="map",
    marker_type=CUBE,
    color=[0.0, 0.0, 1.0, 1.0],  # blue
    scale=[0.3, 0.3, 0.3],
)
print(f"Drew cube marker with id={marker_id}")

# 4. Draw text label
marker_id = robot.viz.draw_marker(
    position=[1.0, 0.0, 1.0],
    quaternion=[0.0, 0.0, 0.0, 1.0],
    marker_type=TEXT_VIEW_FACING,
    text="Hello from Walkie!",
    color=[1.0, 1.0, 1.0, 1.0],  # white
    scale=[0.0, 0.0, 0.15],  # text height = 0.15m
)
print(f"Drew text marker with id={marker_id}")

# --- Continuous Marker Update Example ---

# 5. Create a marker and continuously update its position (circular motion)
print("\nStarting continuous marker update (circle path, 5 seconds)...")
moving_id = robot.draw_marker(
    position=[1.0, 0.0, 0.0],
    quaternion=[0.0, 0.0, 0.0, 1.0],
    marker_type=SPHERE,
    color=[1.0, 1.0, 0.0, 1.0],  # yellow
    scale=[0.15, 0.15, 0.15],
)

start_time = time.time()
radius = 1.5
while time.time() - start_time < 5.0:
    t = time.time() - start_time
    angle = t * 2.0  # radians per second
    x = radius * math.cos(angle)
    y = radius * math.sin(angle)

    # Only update position -- color, scale, frame_id all stay the same
    robot.update_marker(moving_id, position=[x, y, 0.0])
    time.sleep(0.05)  # 20 Hz

print("Continuous update done.")

# --- MarkerArray Example ---

# 6. Draw multiple markers at once
print("\nDrawing marker array (3 waypoints)...")
waypoints = [
    {
        "position": [1.0, 0.0, 0.0],
        "quaternion": [0.0, 0.0, 0.0, 1.0],
        "marker_type": SPHERE,
        "color": [1.0, 0.0, 0.0, 1.0],
        "scale": [0.15, 0.15, 0.15],
        "ns": "waypoints",
    },
    {
        "position": [2.0, 1.0, 0.0],
        "quaternion": [0.0, 0.0, 0.0, 1.0],
        "marker_type": SPHERE,
        "color": [1.0, 0.5, 0.0, 1.0],
        "scale": [0.15, 0.15, 0.15],
        "ns": "waypoints",
    },
    {
        "position": [3.0, 2.0, 0.0],
        "quaternion": [0.0, 0.0, 0.0, 1.0],
        "marker_type": SPHERE,
        "color": [0.0, 1.0, 0.0, 1.0],
        "scale": [0.15, 0.15, 0.15],
        "ns": "waypoints",
    },
]

ids = robot.viz.draw_markers(waypoints)
print(f"Drew {len(ids)} waypoint markers with ids={ids}")

# --- PoseStamped Examples ---

# 7. Publish a PoseStamped (visible in RViz2 with "Pose" display type)
print("\nPublishing PoseStamped at (1, 0, 0.5) on 'walkie/target_pose'...")
topic = robot.draw_pose(
    position=[1.0, 0.0, 0.5],
    quaternion=[0.0, 0.0, 0.0, 1.0],
)
print(f"Published pose on topic '{topic}'")

# 8. Multiple poses on separate topics (e.g. one per arm)
print("\nPublishing left and right arm target poses...")
robot.draw_pose(
    position=[0.3, 0.2, 0.8],
    quaternion=[0.0, 0.0, 0.0, 1.0],
    topic="walkie/target_pose/left_arm",
)
robot.draw_pose(
    position=[0.3, -0.2, 0.8],
    quaternion=[0.0, 0.0, 0.0, 1.0],
    topic="walkie/target_pose/right_arm",
)
print(
    "Published poses on 'walkie/target_pose/left_arm' and 'walkie/target_pose/right_arm'"
)

# 9. Continuously update a pose (circular motion, 5 seconds)
print("\nStarting continuous pose update (circle path, 5 seconds)...")
pose_topic = "walkie/target_pose/left_arm"
robot.draw_pose(
    position=[1.0, 0.0, 0.5],
    quaternion=[0.0, 0.0, 0.0, 1.0],
    topic=pose_topic,
)

start_time = time.time()
radius = 1.0
while time.time() - start_time < 5.0:
    t = time.time() - start_time
    angle = t * 2.0
    x = radius * math.cos(angle)
    y = radius * math.sin(angle)

    # Compute a quaternion facing the direction of travel
    qz = math.sin(angle / 2.0)
    qw = math.cos(angle / 2.0)

    robot.update_pose(
        position=[x, y, 0.5],
        quaternion=[0.0, 0.0, qz, qw],
        topic=pose_topic,
    )
    time.sleep(0.05)  # 20 Hz

print("Continuous pose update done.")

# --- Axis Triad Examples ---

# 10. Draw an axis triad (RGB arrows: Red=X, Green=Y, Blue=Z)
print("\nDrawing axis triad at (1, 0, 0) in base_link frame...")
axis_name = robot.draw_axis(
    position=[1.0, 0.0, 0.0],
    quaternion=[0.0, 0.0, 0.0, 1.0],
    axis_name="demo_axis",
)
print(f"Drew axis triad with name='{axis_name}'")

# 11. Draw a second axis triad rotated 45 degrees around Z
print("Drawing rotated axis triad at (2, 0, 0)...")
yaw_45 = math.pi / 4
robot.draw_axis(
    position=[2.0, 0.0, 0.0],
    quaternion=[0.0, 0.0, math.sin(yaw_45 / 2), math.cos(yaw_45 / 2)],
    axis_name="rotated_axis",
    scale=0.3,
)
print("Drew rotated axis triad")

# 12. Continuously update an axis triad (circular motion with rotation, 5s)
print("\nStarting continuous axis update (circle path, 5 seconds)...")
robot.draw_axis(
    position=[1.0, 0.0, 0.5],
    quaternion=[0.0, 0.0, 0.0, 1.0],
    axis_name="moving_axis",
    scale=0.2,
)

start_time = time.time()
radius = 1.0
while time.time() - start_time < 5.0:
    t = time.time() - start_time
    angle = t * 2.0
    x = radius * math.cos(angle)
    y = radius * math.sin(angle)

    # Orient the axis to face the direction of travel (yaw = angle)
    qz = math.sin(angle / 2.0)
    qw = math.cos(angle / 2.0)

    robot.update_axis(
        axis_name="moving_axis",
        position=[x, y, 0.5],
        quaternion=[0.0, 0.0, qz, qw],
    )
    time.sleep(0.05)  # 20 Hz

print("Continuous axis update done.")

# --- Deletion Examples ---

time.sleep(3)

# 13. Delete a specific marker
print(f"\nDeleting sphere marker (id={sphere_id})...")
robot.viz.delete_marker(marker_id=sphere_id)

time.sleep(2)

# 14. Clear all markers
print("Clearing all markers...")
robot.viz.clear_markers()

print("\nDone! Disconnecting...")
robot.disconnect()
