"""
Walkie SDK - Centralized ROS 2 Topics Configuration
"""
import os
import yaml

# ── Camera Topics ──────────────────────────────────────────────
CAMERA_TOPICS = {
    "head": os.getenv("WALKIE_CAM_HEAD", "/zed_head/zed_node/rgb/color/rect/image"),
    "left": os.getenv("WALKIE_CAM_LEFT", "/walkie/camera/left"),
    "right": os.getenv("WALKIE_CAM_RIGHT", "/walkie/camera/right"),
    "default": os.getenv("WALKIE_CAM_DEFAULT", "walkie/camera/image"),
}

# ── Arm Topics ─────────────────────────────────────────────────
ARM_TOPICS = {
    "states": os.getenv("WALKIE_ARM_STATES", "joint_states"),
    "states_type": os.getenv("WALKIE_ARM_STATES_TYPE", "sensor_msgs/msg/JointState"),
    "jtc_left": os.getenv("WALKIE_ARM_JTC_LEFT", "left_joint_trajectory_controller/joint_trajectory"),
    "jtc_right": os.getenv("WALKIE_ARM_JTC_RIGHT", "right_joint_trajectory_controller/joint_trajectory"),
    "jtc_type": os.getenv("WALKIE_ARM_JTC_TYPE", "trajectory_msgs/msg/JointTrajectory"),
}

ARM_ACTIONS = {
    "interface": os.getenv("WALKIE_ARM_ACTION_INTERFACE", "my_robot_interfaces/action"),
    "go_to_pose":             os.getenv("WALKIE_ARM_ACTION_GO_TO_POSE", "go_to_pose"),
    "go_to_pose_quat":        os.getenv("WALKIE_ARM_ACTION_GO_TO_POSE_QUAT", "go_to_pose_quat"),
    "go_to_pose_relative":    os.getenv("WALKIE_ARM_ACTION_GO_TO_POSE_RELATIVE", "go_to_pose_relative"),
    "go_to_home":             os.getenv("WALKIE_ARM_ACTION_GO_TO_HOME", "go_to_home"),
    "control_gripper":        os.getenv("WALKIE_ARM_ACTION_CONTROL_GRIPPER", "control_gripper"),
    "set_joint_position":     os.getenv("WALKIE_ARM_ACTION_SET_JOINT_POSITION", "set_joint_position"),
}

ARM_SERVICES = {
    "get_ee_pose":           os.getenv("WALKIE_ARM_SVC_GET_EE_POSE", "get_ee_pose"),
    "get_ee_pose_type":      os.getenv("WALKIE_ARM_SVC_GET_EE_POSE_TYPE", "my_robot_interfaces/srv/GetEEPose"),
    "get_joint_states":      os.getenv("WALKIE_ARM_SVC_GET_JOINT_STATES", "get_joint_states"),
    "get_joint_states_type": os.getenv("WALKIE_ARM_SVC_GET_JOINT_STATES_TYPE", "my_robot_interfaces/srv/GetJointStates"),
}

# ── Navigation Topics & Actions ────────────────────────────────
NAV_TOPICS = {
    "cmd_vel": os.getenv("WALKIE_NAV_CMD_VEL", "cmd_vel"),
    "cmd_vel_type": os.getenv("WALKIE_NAV_CMD_VEL_TYPE", "geometry_msgs/msg/Twist"),
}

NAV_ACTIONS = {
    "navigate_to_pose": os.getenv("WALKIE_NAV_ACTION_NAV2", "navigate_to_pose"),
    "navigate_to_pose_type": os.getenv("WALKIE_NAV_ACTION_NAV2_TYPE", "nav2_msgs/action/NavigateToPose"),
}

# ── Telemetry Topics ───────────────────────────────────────────
TELEMETRY_TOPICS = {
    "odom": os.getenv("WALKIE_TELEMETRY_ODOM", "current_pose"),
    "odom_type": os.getenv("WALKIE_TELEMETRY_ODOM_TYPE", "nav_msgs/msg/Odometry"),
}

# ── Visualization Topics ───────────────────────────────────────
VIZ_TOPICS = {
    "markers": os.getenv("WALKIE_VIZ_MARKERS", "walkie/viz_markers"),
    "markers_array": os.getenv("WALKIE_VIZ_MARKERS_ARRAY", "walkie/viz_markers_array"),
    "target_pose": os.getenv("WALKIE_VIZ_TARGET_POSE", "walkie/target_pose"),
    "markers_type": os.getenv("WALKIE_VIZ_MARKERS_TYPE", "visualization_msgs/msg/Marker"),
    "markers_array_type": os.getenv("WALKIE_VIZ_MARKERS_ARRAY_TYPE", "visualization_msgs/msg/MarkerArray"),
    "target_pose_type": os.getenv("WALKIE_VIZ_TARGET_POSE_TYPE", "geometry_msgs/msg/PoseStamped"),
}

# ── Object Pose Service ────────────────────────────────────────
OB_POSE_SERVICE = {
    "service_name": os.getenv("WALKIE_OB_POSE_SERVICE_NAME", "get_3d_poses"),
    "service_type": os.getenv("WALKIE_OB_POSE_SERVICE_TYPE", "perception/srv/GetObPose"),
}

# ── Lift Topics ────────────────────────────────────────────────
LIFT_TOPICS = {
    "cmd":         os.getenv("WALKIE_LIFT_CMD",          "lift/cmd"),
    "cmd_type":    os.getenv("WALKIE_LIFT_CMD_TYPE",     "std_msgs/msg/Float64MultiArray"),
    "states":      os.getenv("WALKIE_LIFT_STATES",       "lift/joint_states"),
    "states_type": os.getenv("WALKIE_LIFT_STATES_TYPE",  "sensor_msgs/msg/JointState"),
}

# ── Head Topics ────────────────────────────────────────────────
HEAD_TOPICS = {
    "cmd":      os.getenv("WALKIE_HEAD_CMD",      "head_servo_controller/commands"),
    "cmd_type": os.getenv("WALKIE_HEAD_CMD_TYPE", "std_msgs/msg/Float64MultiArray"),
}

ROS_DOMAIN_ID = int(os.getenv("WALKIE_ROS_DOMAIN_ID", "23"))

def load_config(yaml_path: str):
    """
    Load topics from a YAML file and update the global dictionaries in-place.
    """
    if not os.path.isfile(yaml_path):
        print(f"[Walkie SDK] Config file '{yaml_path}' not found. Using defaults/env vars.")
        return

    try:
        with open(yaml_path, 'r') as file:
            config = yaml.safe_load(file)
            
        if not config:
            return
            
        # Update dictionaries in-place so all imported references reflect the new YAML values
        if "CAMERA_TOPICS" in config: CAMERA_TOPICS.update(config["CAMERA_TOPICS"])
        if "ARM_TOPICS" in config: ARM_TOPICS.update(config["ARM_TOPICS"])
        if "ARM_ACTIONS" in config: ARM_ACTIONS.update(config["ARM_ACTIONS"])
        if "ARM_SERVICES" in config: ARM_SERVICES.update(config["ARM_SERVICES"])
        if "NAV_TOPICS" in config: NAV_TOPICS.update(config["NAV_TOPICS"])
        if "NAV_ACTIONS" in config: NAV_ACTIONS.update(config["NAV_ACTIONS"])
        if "TELEMETRY_TOPICS" in config: TELEMETRY_TOPICS.update(config["TELEMETRY_TOPICS"])
        if "VIZ_TOPICS" in config: VIZ_TOPICS.update(config["VIZ_TOPICS"])
        if "OB_POSE_SERVICE" in config: OB_POSE_SERVICE.update(config["OB_POSE_SERVICE"])
        if "LIFT_TOPICS" in config: LIFT_TOPICS.update(config["LIFT_TOPICS"])
        if "HEAD_TOPICS" in config: HEAD_TOPICS.update(config["HEAD_TOPICS"])
        
        print(f"[Walkie SDK] Loaded custom topics from '{yaml_path}'")

    except Exception as e:
        print(f"[Walkie SDK] Failed to load config from '{yaml_path}': {e}")
