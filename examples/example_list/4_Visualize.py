#!/usr/bin/env python3
from walkie_sdk import WalkieRobot
# Import shape constants
from walkie_sdk.modules.visualization import ARROW, SPHERE

def main():
    print("--- 4. Visualization (RViz2) ---")
    bot = WalkieRobot(ip="127.0.0.1", camera_protocol="none")

    # 1. Single Marker
    m_id = bot.viz.draw_marker(
        position=[1.0, 0.0, 0.0], 
        quaternion=[0.0, 0.0, 0.0, 1.0], 
        marker_type=ARROW, 
        color=[1.0, 0.0, 0.0, 1.0],
        scale=[0.2, 0.05, 0.05]
    )
    
    # Update single marker
    bot.viz.update_marker(m_id, position=[1.5, 0.0, 0.0])

    # 2. Marker Arrays
    bot.viz.draw_markers([
        {"position": [0.0, 1.0, 0.0], "marker_type": SPHERE, "color": [0,1,0,1]},
        {"position": [0.0, 2.0, 0.0], "marker_type": SPHERE, "color": [0,0,1,1]}
    ])

    # 3. Pose Stamped
    p_topic = bot.viz.draw_pose(
        position=[0.5, 0.0, 0.5], 
        topic="walkie/target_pose/left_arm"
    )
    bot.viz.update_pose(position=[0.5, 0.5, 0.5], topic=p_topic)

    # 4. Cleanup
    bot.viz.delete_marker(m_id)
    bot.viz.clear_markers()

    bot.disconnect()

if __name__ == "__main__":
    main()