import time

from walkie_sdk import WalkieRobot, SPHERE, TEXT_VIEW_FACING

# Initialize Robot (Zenoh Protocol)
bot = WalkieRobot(
    ip="100.87.162.10",
    ros_protocol="rosbridge",
    camera_protocol="zenoh",
)

# Use the new Tools module
coords_2d = [
    [320.0, 240.0, 50.0, 50.0],
    [220.0, 140.0, 50.0, 50.0],
    [600.0, 300.0, 50.0, 50.0],
]
result_3d = bot.tools.bboxes_to_positions(coords_2d)

print(f"3D Result: {result_3d}")

# Visualize returned 3D positions as markers in RViz2
if result_3d:
    # Color palette for each detection (RGBA)
    colors = [
        [1.0, 0.0, 0.0, 1.0],  # red
        [0.0, 1.0, 0.0, 1.0],  # green
        [0.0, 0.0, 1.0, 1.0],  # blue
        [1.0, 1.0, 0.0, 1.0],  # yellow
        [1.0, 0.0, 1.0, 1.0],  # magenta
    ]

    for i, pos in enumerate(result_3d):
        color = colors[i % len(colors)]

        # Draw a sphere at the 3D position
        marker_id = bot.draw_marker(
            position=pos,
            quaternion=[0.0, 0.0, 0.0, 1.0],
            marker_type=SPHERE,
            color=color,
            scale=[0.1, 0.1, 0.1],
            frame_id="map",
            ns="detections",
        )
        print(f"  Marker id={marker_id} at pos={pos}")

        # Draw a text label above the sphere
        bot.draw_marker(
            position=[pos[0], pos[1], float(pos[2]) + 0.15],
            quaternion=[0.0, 0.0, 0.0, 1.0],
            marker_type=TEXT_VIEW_FACING,
            text=f"det_{i} ({pos[0]:.2f}, {pos[1]:.2f}, {pos[2]:.2f})",
            color=[1.0, 1.0, 1.0, 1.0],
            scale=[0.1, 0.1, 0.1],
            ns="detections_label",
            frame_id="map",
        )

    # Draw an axis triad at each detection position
    for i, pos in enumerate(result_3d):
        bot.draw_axis(
            position=pos,
            quaternion=[0.0, 0.0, 0.0, 1.0],
            axis_name=f"detection_{i}",
            scale=0.15,
        )

    print(f"Visualized {len(result_3d)} detections in RViz2")
    print("  - Sphere markers on topic: /walkie/viz_markers")
    print("  - Axis triads on topic:    /walkie/viz_axis")

    # Keep markers visible for a few seconds before disconnecting
    time.sleep(3)
else:
    print("No 3D positions returned, nothing to visualize.")

bot.disconnect()
