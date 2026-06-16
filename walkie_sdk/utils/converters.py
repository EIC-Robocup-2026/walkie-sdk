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


def rotate_vector_by_quaternion(
    v: Tuple[float, float, float],
    q: Tuple[float, float, float, float],
) -> Tuple[float, float, float]:
    """Rotate vector *v* = (vx, vy, vz) by quaternion *q* = (x, y, z, w)."""
    q_conj = (-q[0], -q[1], -q[2], q[3])
    v_quat = (v[0], v[1], v[2], 0.0)
    tmp = quaternion_multiply(q, v_quat)
    r = quaternion_multiply(tmp, q_conj)
    return (r[0], r[1], r[2])


def compose_transforms(
    parent_tf: Tuple,
    child_tf: Tuple,
) -> Tuple:
    """Compose two rigid transforms, each a 7-tuple (tx, ty, tz, qx, qy, qz, qw).

    Returns the composed transform T_parent * T_child as a 7-tuple.
    """
    pt, pq = parent_tf[:3], parent_tf[3:]
    ct, cq = child_tf[:3], child_tf[3:]
    rotated = rotate_vector_by_quaternion(ct, pq)
    tx, ty, tz = pt[0] + rotated[0], pt[1] + rotated[1], pt[2] + rotated[2]
    cq_out = quaternion_multiply(pq, cq)
    return (tx, ty, tz, cq_out[0], cq_out[1], cq_out[2], cq_out[3])


def invert_transform(tf: Tuple) -> Tuple:
    """Invert a rigid transform 7-tuple (tx, ty, tz, qx, qy, qz, qw)."""
    tx, ty, tz, qx, qy, qz, qw = tf
    q_inv = (-qx, -qy, -qz, qw)
    t_inv = rotate_vector_by_quaternion((-tx, -ty, -tz), q_inv)
    return (t_inv[0], t_inv[1], t_inv[2], q_inv[0], q_inv[1], q_inv[2], q_inv[3])


def resolve_tf_chain(
    tf_data: Dict[Tuple[str, str], Tuple],
    source: str,
    target: str,
) -> Optional[Tuple]:
    """Walk the TF tree (BFS) from *source* to *target*.

    *tf_data* maps (parent_frame, child_frame) → 7-tuple (tx,ty,tz,qx,qy,qz,qw).
    Edges are traversed in both directions; going child→parent inverts the transform.

    Returns the composed 7-tuple or None if no path exists.
    """
    if source == target:
        return (0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0)

    adjacency: Dict[str, list] = {}
    for parent, child in tf_data:
        adjacency.setdefault(parent, []).append((child, True))
        adjacency.setdefault(child, []).append((parent, False))

    visited = {source}
    queue = [(source, [])]
    while queue:
        node, path = queue.pop(0)
        if node == target:
            composed: Tuple = (0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0)
            for parent, child, forward in path:
                edge_tf = tf_data[(parent, child)]
                composed = compose_transforms(
                    composed, edge_tf if forward else invert_transform(edge_tf)
                )
            return composed
        for neighbour, forward in adjacency.get(node, []):
            if neighbour not in visited:
                visited.add(neighbour)
                edge = (node, neighbour, True) if forward else (neighbour, node, False)
                queue.append((neighbour, path + [edge]))
    return None


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

