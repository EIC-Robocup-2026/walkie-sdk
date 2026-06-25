"""
Converters - Utility functions for coordinate transformations.

Provides quaternion <-> euler angle conversions for working with ROS orientations.
"""

import math
from typing import Any, Dict, List, Optional, Tuple
import time

import numpy as np


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

def quaternion_to_matrix(x: float, y: float, z: float, w: float) -> np.ndarray:
    """
    Convert a quaternion to a 3x3 rotation matrix.

    Useful for transforming point clouds: with the camera pose in the map
    frame as (R, t), camera-frame points map to the world via
    ``points @ R.T + t``.

    Args:
        x: Quaternion x component
        y: Quaternion y component
        z: Quaternion z component
        w: Quaternion w component

    Returns:
        3x3 rotation matrix as a numpy array (float64)
    """
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    if norm == 0.0:
        raise ValueError("Cannot convert zero quaternion to rotation matrix")
    x, y, z, w = x / norm, y / norm, z / norm, w / norm

    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ]
    )


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


def bbox_to_bounding_box2d(bbox: List[float]) -> dict:
    """
    Convert a 2D bounding box [cx, cy, w, h] to a vision_msgs/BoundingBox2D dict.

    Used as the fallback region for the grasp services when no mask is supplied.
    Matches the bbox sub-structure produced by convert_bboxes_to_detection_array.

    Args:
        bbox: [cx, cy, w, h] — centre x/y and width/height, in pixels.

    Returns:
        Dictionary representing vision_msgs/msg/BoundingBox2D.
    """
    cx, cy, w, h = bbox
    return {
        "center": {
            "position": {"x": float(cx), "y": float(cy)},
            "theta": 0.0,
        },
        "size_x": float(w),
        "size_y": float(h),
    }


def numpy_to_mono8_image(mask, frame_id: str = "") -> dict:
    """
    Convert a 2D numpy mask into a sensor_msgs/Image dict (mono8 encoding).

    Any non-zero pixel marks the object. Use the pixel value as the tracker id
    for multi-object masks (then pass that id to Grasp.from_mask); a plain
    boolean/0-255 mask works for the single-object case.

    ``data`` is emitted as a list of ints so it serializes over both rosbridge
    (JSON) and zenoh transports.

    Args:
        mask: 2D array-like (H, W). Cast to uint8; values are kept as-is so
              label masks (pixel value = tracker id) survive.
        frame_id: optional header frame id.

    Returns:
        Dictionary representing sensor_msgs/msg/Image (encoding "mono8").
    """
    arr = np.asarray(mask)
    if arr.ndim != 2:
        raise ValueError(f"mask must be 2D (H, W), got shape {arr.shape}")
    arr = arr.astype(np.uint8)
    h, w = arr.shape
    now = time.time()
    sec = int(now)
    nanosec = int((now - sec) * 1e9)
    return {
        "header": {
            "stamp": {"sec": sec, "nanosec": nanosec},
            "frame_id": frame_id,
        },
        "height": int(h),
        "width": int(w),
        "encoding": "mono8",
        "is_bigendian": 0,
        "step": int(w),
        "data": arr.reshape(-1).tolist(),
    }


def parse_point_cloud_xyz(
    cloud: Dict[str, Any],
    remove_nan: bool = True,
) -> "Optional[Any]":
    """
    Parse a ROS PointCloud2 message dict into a numpy (N, 3) float32 XYZ array.

    Handles the binary data formats produced by rosbridge (list of ints)
    and the zenoh transport (bytes / bytearray / numpy array).
    Field offsets are read dynamically from the message's ``fields`` list,
    so the function works regardless of point layout.

    Args:
        cloud: PointCloud2 message dict from bot.point_cloud.get_cloud()
               or get_once().
        remove_nan: If True (default), discard points where any coordinate
                    is NaN or infinite. ZED outputs NaN for unmeasured pixels.

    Returns:
        numpy float32 array of shape (N, 3) with columns [x, y, z],
        or None if the cloud cannot be parsed (missing fields, empty data, …).

    Example:
        ```python
        cloud = bot.point_cloud.get_once()
        pts = parse_point_cloud_xyz(cloud)   # (N, 3) float32
        print(pts.shape)                     # e.g. (184320, 3)
        ```
    """
    try:
        import numpy as np

        # -- normalise raw bytes -----------------------------------------------
        data = cloud.get("data")
        if data is None:
            return None

        if isinstance(data, str):
            # rosbridge v2 sends uint8[] fields as base64-encoded strings
            import base64
            raw = base64.b64decode(data)
        elif isinstance(data, list):
            raw = bytes(data)
        elif isinstance(data, (bytes, bytearray)):
            raw = bytes(data)
        elif hasattr(data, "tobytes"):   # numpy array
            raw = data.tobytes()
        else:
            return None

        if not raw:
            return None

        # -- geometry metadata -------------------------------------------------
        point_step: int = cloud.get("point_step", 0)
        height: int = cloud.get("height", 1)
        width: int = cloud.get("width", 0)
        n_points: int = height * width

        if point_step == 0 or n_points == 0:
            return None
        if len(raw) < n_points * point_step:
            return None

        # -- field offset map --------------------------------------------------
        # ROS PointCloud2 field datatypes: 7 = FLOAT32 (4 bytes)
        fields = cloud.get("fields", [])
        offsets: Dict[str, int] = {}
        for f in fields:
            if isinstance(f, dict):
                name = f.get("name", "")
                if name in ("x", "y", "z"):
                    offsets[name] = int(f.get("offset", 0))

        # Fallback to standard XYZ layout (offset 0, 4, 8)
        x_off = offsets.get("x", 0)
        y_off = offsets.get("y", 4)
        z_off = offsets.get("z", 8)

        # -- extract float32 columns -------------------------------------------
        arr = np.frombuffer(raw, dtype=np.uint8)[:n_points * point_step].reshape(n_points, point_step)

        x = np.frombuffer(arr[:, x_off:x_off + 4].tobytes(), dtype=np.float32)
        y = np.frombuffer(arr[:, y_off:y_off + 4].tobytes(), dtype=np.float32)
        z = np.frombuffer(arr[:, z_off:z_off + 4].tobytes(), dtype=np.float32)

        pts = np.column_stack([x, y, z])

        if remove_nan:
            pts = pts[np.isfinite(pts).all(axis=1)]

        return pts

    except Exception:
        return None

