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

# ── Depth Camera Topics ────────────────────────────────────────
# Raw depth streams (sensor_msgs/msg/Image). Keyed by the same camera names
# as CAMERA_TOPICS so depth and color can be addressed alike. The ZED head
# publishes 32FC1 (float32 metres, NaN = invalid) on depth_registered.
DEPTH_TOPICS = {
    "head": os.getenv("WALKIE_DEPTH_HEAD", "/zed_head/zed_node/depth/depth_registered"),
}

# ── Camera Info Topics ─────────────────────────────────────────
# Intrinsics (sensor_msgs/msg/CameraInfo), published alongside each image
# topic. Keyed by the same camera names as CAMERA_TOPICS. The ZED head's
# depth_registered is aligned to the left rectified image, so the one set of
# intrinsics serves both colour and depth projection.
CAMERA_INFO_TOPICS = {
    "head": os.getenv("WALKIE_CAMERA_INFO_HEAD", "/zed_head/zed_node/rgb/color/rect/camera_info"),
}
# ── Point Cloud Topics ─────────────────────────────────────────
# Keys are source names → topic strings. Message type is always
# sensor_msgs/msg/PointCloud2 (hardcoded in the transport).
POINT_CLOUD_TOPICS = {
    "head": os.getenv(
        "WALKIE_PC_HEAD",
        "/zed_head/zed_node/point_cloud/cloud_registered/filtered_map",
    ),
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
    "clear_objects":         os.getenv("WALKIE_ARM_SVC_CLEAR_OBJECTS", "clear_collision_objects"),
    "clear_objects_type":    os.getenv("WALKIE_ARM_SVC_CLEAR_OBJECTS_TYPE", "std_srvs/srv/Trigger"),
    "toggle_collision":      os.getenv("WALKIE_ARM_SVC_TOGGLE_COLLISION", "toggle_gripper_collision"),
    "toggle_collision_type": os.getenv("WALKIE_ARM_SVC_TOGGLE_COLLISION_TYPE", "my_robot_interfaces/srv/ToggleGripperCollision"),
    "clear_octomap":         os.getenv("WALKIE_ARM_SVC_CLEAR_OCTOMAP", "clear_octomap"),
    "clear_octomap_type":    os.getenv("WALKIE_ARM_SVC_CLEAR_OCTOMAP_TYPE", "std_srvs/srv/Empty"),
    "toggle_all_collision":      os.getenv("WALKIE_ARM_SVC_TOGGLE_ALL_COLLISION", "toggle_all_collision_checking"),
    "toggle_all_collision_type": os.getenv("WALKIE_ARM_SVC_TOGGLE_ALL_COLLISION_TYPE", "std_srvs/srv/SetBool"),
}

# Standard ROS 2 parameter services exposed by the commander node, used to
# read/write its tuning params live (gripper_speed, planner_id, grasp_*, ...).
# "node" is the commander node name; the param services live under it.
ARM_PARAMS = {
    "node":                 os.getenv("WALKIE_ARM_PARAM_NODE", "bimanual_commander"),
    "set":                  os.getenv("WALKIE_ARM_PARAM_SET", "set_parameters"),
    "set_type":             os.getenv("WALKIE_ARM_PARAM_SET_TYPE", "rcl_interfaces/srv/SetParameters"),
    "get":                  os.getenv("WALKIE_ARM_PARAM_GET", "get_parameters"),
    "get_type":             os.getenv("WALKIE_ARM_PARAM_GET_TYPE", "rcl_interfaces/srv/GetParameters"),
    "list":                 os.getenv("WALKIE_ARM_PARAM_LIST", "list_parameters"),
    "list_type":            os.getenv("WALKIE_ARM_PARAM_LIST_TYPE", "rcl_interfaces/srv/ListParameters"),
}

# ── Navigation Topics & Actions ────────────────────────────────
NAV_TOPICS = {
    "cmd_vel": os.getenv("WALKIE_NAV_CMD_VEL", "cmd_vel_controller"),
    "cmd_vel_type": os.getenv("WALKIE_NAV_CMD_VEL_TYPE", "geometry_msgs/msg/TwistStamped"),
}

NAV_ACTIONS = {
    "navigate_to_pose":        os.getenv("WALKIE_NAV_ACTION_NAV2",         "navigate_to_pose"),
    "navigate_to_pose_type":   os.getenv("WALKIE_NAV_ACTION_NAV2_TYPE",    "nav2_msgs/action/NavigateToPose"),
    "navigate_to_object":      os.getenv("WALKIE_NAV_ACTION_NAV_OBJ",      "navigate_to_object"),
    "navigate_to_object_type": os.getenv("WALKIE_NAV_ACTION_NAV_OBJ_TYPE", "robot_navigation/action/NavigateToObject"),
}

# ── Map Topics & Services ──────────────────────────────────────
MAP_TOPICS = {
    "map":      os.getenv("WALKIE_MAP_TOPIC", "map"),
    "map_type": os.getenv("WALKIE_MAP_TYPE",  "nav_msgs/msg/OccupancyGrid"),
}

MAP_SERVICES = {
    "get_map":      os.getenv("WALKIE_MAP_SERVICE",      "map_server/map"),
    "get_map_type": os.getenv("WALKIE_MAP_SERVICE_TYPE", "nav_msgs/srv/GetMap"),
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
    "service_type": os.getenv("WALKIE_OB_POSE_SERVICE_TYPE", "walkie_perception/srv/GetObPose"),
}

# ── Transform Service ──────────────────────────────────────────
TF_SERVICE = {
    "service_name": os.getenv("WALKIE_TF_SERVICE_NAME", "get_transform"),
    "service_type": os.getenv("WALKIE_TF_SERVICE_TYPE", "walkie_tf_interfaces/srv/GetTransform"),
}

# ── Grasp Services (unified GraspNet server) ───────────────────
# One node ("grasp_server") serves all three grasp services + a shared
# standby/status pair. Names are stored without a leading slash so namespacing
# resolves them like every other service (empty namespace → "grasp/from_mask").
GRASP_SERVICES = {
    "from_mask":       os.getenv("WALKIE_GRASP_FROM_MASK",       "grasp/from_mask"),
    "from_mask_type":  os.getenv("WALKIE_GRASP_FROM_MASK_TYPE",  "walkie_perception/srv/GraspFromMask"),
    "from_cloud":      os.getenv("WALKIE_GRASP_FROM_CLOUD",      "grasp/from_cloud"),
    "from_cloud_type": os.getenv("WALKIE_GRASP_FROM_CLOUD_TYPE", "walkie_perception/srv/GraspFromCloud"),
    "pos":             os.getenv("WALKIE_GRASP_POS",             "grasp/pos"),
    "pos_type":        os.getenv("WALKIE_GRASP_POS_TYPE",        "walkie_perception/srv/GraspPos"),
    "standby":         os.getenv("WALKIE_GRASP_STANDBY",         "grasp/standby"),
    "standby_type":    os.getenv("WALKIE_GRASP_STANDBY_TYPE",    "std_srvs/srv/SetBool"),
    "status":          os.getenv("WALKIE_GRASP_STATUS",          "grasp/status"),
    "status_type":     os.getenv("WALKIE_GRASP_STATUS_TYPE",     "std_srvs/srv/Trigger"),
}

# ── Joint State Topics (shared hub) ───────────────────────────
JOINT_STATE_TOPICS = {
    "states":      os.getenv("WALKIE_JOINT_STATES",      "joint_states"),
    "states_type": os.getenv("WALKIE_JOINT_STATES_TYPE", "sensor_msgs/msg/JointState"),
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
    "cmd":         os.getenv("WALKIE_HEAD_CMD",          "head_servo_controller/commands"),
    "cmd_type":    os.getenv("WALKIE_HEAD_CMD_TYPE",     "std_msgs/msg/Float64MultiArray"),
    "state_joint":               os.getenv("WALKIE_HEAD_STATE_JOINT",              "head_servo_joint"),
    "auto_tilt_enable_service":  os.getenv("WALKIE_HEAD_AUTO_TILT_ENABLE_SRV",      "head_tilt_near_goal/enable"),
    "auto_tilt_enable_service_type": os.getenv("WALKIE_HEAD_AUTO_TILT_ENABLE_SRV_TYPE", "std_srvs/srv/SetBool"),
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
        if "DEPTH_TOPICS" in config: DEPTH_TOPICS.update(config["DEPTH_TOPICS"])
        if "CAMERA_INFO_TOPICS" in config: CAMERA_INFO_TOPICS.update(config["CAMERA_INFO_TOPICS"])
        if "POINT_CLOUD_TOPICS" in config: POINT_CLOUD_TOPICS.update(config["POINT_CLOUD_TOPICS"])
        if "ARM_TOPICS" in config: ARM_TOPICS.update(config["ARM_TOPICS"])
        if "ARM_ACTIONS" in config: ARM_ACTIONS.update(config["ARM_ACTIONS"])
        if "ARM_SERVICES" in config: ARM_SERVICES.update(config["ARM_SERVICES"])
        if "ARM_PARAMS" in config: ARM_PARAMS.update(config["ARM_PARAMS"])
        if "NAV_TOPICS" in config: NAV_TOPICS.update(config["NAV_TOPICS"])
        if "NAV_ACTIONS" in config: NAV_ACTIONS.update(config["NAV_ACTIONS"])
        if "MAP_TOPICS" in config: MAP_TOPICS.update(config["MAP_TOPICS"])
        if "MAP_SERVICES" in config: MAP_SERVICES.update(config["MAP_SERVICES"])
        if "TELEMETRY_TOPICS" in config: TELEMETRY_TOPICS.update(config["TELEMETRY_TOPICS"])
        if "VIZ_TOPICS" in config: VIZ_TOPICS.update(config["VIZ_TOPICS"])
        if "OB_POSE_SERVICE" in config: OB_POSE_SERVICE.update(config["OB_POSE_SERVICE"])
        if "TF_SERVICE" in config: TF_SERVICE.update(config["TF_SERVICE"])
        if "GRASP_SERVICES" in config: GRASP_SERVICES.update(config["GRASP_SERVICES"])
        if "LIFT_TOPICS" in config: LIFT_TOPICS.update(config["LIFT_TOPICS"])
        if "HEAD_TOPICS" in config: HEAD_TOPICS.update(config["HEAD_TOPICS"])
        if "JOINT_STATE_TOPICS" in config: JOINT_STATE_TOPICS.update(config["JOINT_STATE_TOPICS"])
        
        print(f"[Walkie SDK] Loaded custom topics from '{yaml_path}'")

    except Exception as e:
        print(f"[Walkie SDK] Failed to load config from '{yaml_path}': {e}")


def _default_config_path():
    """
    Resolve the YAML config to auto-load on import, in priority order:
      1. $WALKIE_CONFIG_PATH (explicit override)
      2. ./ros_topics.yaml in the current working directory
      3. ros_topics.yaml at the SDK repo root (two levels above this package)
    Returns the first existing path, or None if no config file is found.
    """
    env_path = os.getenv("WALKIE_CONFIG_PATH")
    if env_path:
        return env_path

    cwd_path = os.path.join(os.getcwd(), "ros_topics.yaml")
    if os.path.isfile(cwd_path):
        return cwd_path

    repo_root = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
    repo_path = os.path.join(repo_root, "ros_topics.yaml")
    if os.path.isfile(repo_path):
        return repo_path

    return None


# Auto-load a default YAML config on import if one is present. An explicit
# config_path passed to WalkieRobot(...) is applied afterwards and overrides this.
_auto_config_path = _default_config_path()
if _auto_config_path:
    load_config(_auto_config_path)
