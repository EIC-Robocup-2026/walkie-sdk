"""
Example: Visualization Markers in RViz2

Demonstrates how to publish visualization markers using the Walkie SDK.
These markers are visible in RViz2 by subscribing to the marker topics.

RViz2 Setup:
  1. Open RViz2
  2. Add a "Marker" display
  3. Set the topic to /walkie/viz_markers (or /walkie/viz_markers_array)
  4. Markers will appear in the 3D view

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

# --- Deletion Examples ---

time.sleep(3)

# 7. Delete a specific marker
print(f"\nDeleting sphere marker (id={sphere_id})...")
robot.viz.delete_marker(marker_id=sphere_id)

time.sleep(2)

# 8. Clear all markers
print("Clearing all markers...")
robot.viz.clear_markers()

print("\nDone! Disconnecting...")
robot.disconnect()
