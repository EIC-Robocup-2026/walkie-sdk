"""
Converters - Utility functions for coordinate transformations.

Provides quaternion <-> euler angle conversions for working with ROS orientations.
"""

import math
from typing import Tuple,List
import time


def quaternion_to_euler(
    x: float, y: float, z: float, w: float
) -> Tuple[float, float, float]:
    """
    Convert quaternion to euler angles (roll, pitch, yaw).

    Args:
        x: Quaternion x component
        y: Quaternion y component
        z: Quaternion z component
        w: Quaternion w component

    Returns:
        Tuple of (roll, pitch, yaw) in radians
        - roll: Rotation around X axis
        - pitch: Rotation around Y axis
        - yaw: Rotation around Z axis (heading)
    """
    # Roll (x-axis rotation)
    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    # Pitch (y-axis rotation)
    sinp = 2.0 * (w * y - z * x)
    if abs(sinp) >= 1:
        # Use 90 degrees if out of range
        pitch = math.copysign(math.pi / 2, sinp)
    else:
        pitch = math.asin(sinp)

    # Yaw (z-axis rotation) - this is the heading for 2D navigation
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)

    return (roll, pitch, yaw)


def euler_to_quaternion(
    roll: float, pitch: float, yaw: float
) -> Tuple[float, float, float, float]:
    """
    Convert euler angles to quaternion.

    Args:
        roll: Rotation around X axis in radians
        pitch: Rotation around Y axis in radians
        yaw: Rotation around Z axis in radians (heading)

    Returns:
        Tuple of (x, y, z, w) quaternion components
    """
    # Abbreviations for the various angular functions
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)

    x = sr * cp * cy - cr * sp * sy
    y = cr * sp * cy + sr * cp * sy
    z = cr * cp * sy - sr * sp * cy
    w = cr * cp * cy + sr * sp * sy

    return (x, y, z, w)


def normalize_angle(angle: float) -> float:
    """
    Normalize an angle to the range [-pi, pi].

    Args:
        angle: Angle in radians

    Returns:
        Normalized angle in radians
    """
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


def degrees_to_radians(degrees: float) -> float:
    """Convert degrees to radians."""
    return degrees * math.pi / 180.0


def radians_to_degrees(radians: float) -> float:
    """Convert radians to degrees."""
    return radians * 180.0 / math.pi


def quaternion_multiply(
    q1: Tuple[float, float, float, float],
    q2: Tuple[float, float, float, float],
) -> Tuple[float, float, float, float]:
    """
    Multiply two quaternions (Hamilton product).

    Computes q1 * q2, which applies rotation q2 first, then q1.
    Use this to combine orientations: result = base_orientation * delta_rotation.

    Args:
        q1: First quaternion (x, y, z, w)
        q2: Second quaternion (x, y, z, w)

    Returns:
        Tuple of (x, y, z, w) representing the combined rotation
    """
    x1, y1, z1, w1 = q1
    x2, y2, z2, w2 = q2

    x = w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2
    y = w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2
    z = w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2
    w = w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2

    return (x, y, z, w)

def convert_bboxes_to_detection_array(
    bboxes: List[List[float]], 
    frame_id: str = "camera_frame"
) -> dict:
    """
    Converts a list of [cx, cy, w, h] to a vision_msgs/Detection2DArray dictionary.
    
    Args:
        bboxes: List of [cx, cy, w, h] where:
                cx, cy = center x, y
                w, h = width, height
        frame_id: The frame reference (e.g., 'head_camera')
        
    Returns:
        Dictionary representing vision_msgs/msg/Detection2DArray
    """
    
    # Get current time (approximate for Zenoh/Python)
    now = time.time()
    sec = int(now)
    nanosec = int((now - sec) * 1e9)
    
    detection_list = []
    
    for bbox in bboxes:
        cx, cy, w, h = bbox
        
        # Create a single Detection2D
        detection = {
            "header": {
                "stamp": {"sec": sec, "nanosec": nanosec},
                "frame_id": frame_id
            },
            "results": [], # ObjectHypothesisWithPose[] (empty if no classification)
            "bbox": {
                "center": {
                    'position':{"x": float(cx),"y": float(cy)},
                    "theta": 0.0
                },
                "size_x": float(w),
                "size_y": float(h)
            },
            "id": "" # Optional ID
        }
        detection_list.append(detection)

    # Create the final Detection2DArray
    msg = {
        "header": {
            "stamp": {"sec": sec, "nanosec": nanosec},
            "frame_id": frame_id
        },
        "detections": detection_list
    }
    
    return msg

def convert_poses_to_array(data):
    """
    Extracts [x, y, z] coordinates from a dictionary of poses.
    """
    return [[p['position']['x'], p['position']['y'], p['position']['z']] for p in data.get('poses', [])]

