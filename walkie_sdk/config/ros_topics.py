"""
Walkie SDK - Centralized ROS 2 Topics Configuration

This module stores all default ROS 2 topics, actions, and services used by the SDK.
Topics can be overridden via environment variables without modifying the source code.

Example:
    export WALKIE_CAM_HEAD="/my_robot/custom_camera/image"
"""
import os

# ── Camera Topics ──────────────────────────────────────────────
CAMERA_TOPICS = {
    "head": os.getenv("WALKIE_CAM_HEAD", "/zed_head/zed_node/rgb/color/rect/image"),
    "left": os.getenv("WALKIE_CAM_LEFT", "/walkie/camera/left"),
    "right": os.getenv("WALKIE_CAM_RIGHT", "/walkie/camera/right"),
    "default": os.getenv("WALKIE_CAM_DEFAULT", "walkie/camera/image"),
}

# ── Arm Topics ─────────────────────────────────────────────────
ARM_TOPICS = {
    # Topics
    "commands": os.getenv("WALKIE_ARM_COMMANDS", "walkie/arm/commands"),
    "states": os.getenv("WALKIE_ARM_STATES", "joint_states"),
    "target_pose": os.getenv("WALKIE_ARM_TARGET_POSE", "/target_pose"),
    # Types
    "commands_type": os.getenv("WALKIE_ARM_COMMANDS_TYPE", "sensor_msgs/msg/JointState"),
    "states_type": os.getenv("WALKIE_ARM_STATES_TYPE", "sensor_msgs/msg/JointState"),
    "target_pose_type": os.getenv("WALKIE_ARM_TARGET_POSE_TYPE", "geometry_msgs/msg/PoseStamped"),
}

ARM_ACTIONS = {
    "interface": os.getenv("WALKIE_ARM_ACTION_INTERFACE", "my_robot_interfaces/action"),
    "move_group": os.getenv("WALKIE_ARM_ACTION_MOVE_GROUP", "moveit_msgs/action/MoveGroup"),
}

# ── Navigation Topics & Actions ────────────────────────────────
NAV_TOPICS = {
    # Topics
    "cmd_vel": os.getenv("WALKIE_NAV_CMD_VEL", "cmd_vel"),
    # Types
    "cmd_vel_type": os.getenv("WALKIE_NAV_CMD_VEL_TYPE", "geometry_msgs/msg/Twist"),
}

NAV_ACTIONS = {
    # Actions
    "navigate_to_pose": os.getenv("WALKIE_NAV_ACTION_NAV2", "navigate_to_pose"),
    # Types
    "navigate_to_pose_type": os.getenv("WALKIE_NAV_ACTION_NAV2_TYPE", "nav2_msgs/action/NavigateToPose"),
}

# ── Telemetry Topics ───────────────────────────────────────────
TELEMETRY_TOPICS = {
    # Topics
    "odom": os.getenv("WALKIE_TELEMETRY_ODOM", "current_pose"),
    # Types
    "odom_type": os.getenv("WALKIE_TELEMETRY_ODOM_TYPE", "nav_msgs/msg/Odometry"),
}

# ── Visualization Topics ───────────────────────────────────────
VIZ_TOPICS = {
    # Topics
    "markers": os.getenv("WALKIE_VIZ_MARKERS", "walkie/viz_markers"),
    "markers_array": os.getenv("WALKIE_VIZ_MARKERS_ARRAY", "walkie/viz_markers_array"),
    "target_pose": os.getenv("WALKIE_VIZ_TARGET_POSE", "walkie/target_pose"),
    # Types
    "markers_type": os.getenv("WALKIE_VIZ_MARKERS_TYPE", "visualization_msgs/msg/Marker"),
    "markers_array_type": os.getenv("WALKIE_VIZ_MARKERS_ARRAY_TYPE", "visualization_msgs/msg/MarkerArray"),
    "target_pose_type": os.getenv("WALKIE_VIZ_TARGET_POSE_TYPE", "geometry_msgs/msg/PoseStamped"),
}

OB_POSE_TOPIC = {
    # Topics
    "object_pose": os.getenv("WALKIE_OBJECT_POSE_TOPIC", "/yolo/detections_2d"),
    "object_pose_response": os.getenv("WALKIE_OBJECT_POSE_RESPONSE_TOPIC", "/ob_detection/poses"),

    # Types
    "object_pose_type": os.getenv("WALKIE_OBJECT_POSE_TYPE", "vision_msgs/msg/Detection2DArray"),
    "object_pose_response_type": os.getenv("WALKIE_OBJECT_POSE_RESPONSE_TYPE", "geometry_msgs/msg/PoseArray"),
}