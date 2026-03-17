"""
Visualization - RViz2 marker publishing module.

Provides draw_marker(), draw_markers(), draw_pose(), and
clear_markers() for publishing visualization markers and poses to RViz2.

This module uses ROSTransportInterface abstraction, allowing it
to work with any transport implementation (rosbridge, zenoh).
"""

import threading
from typing import Any, Dict, List, Optional, Tuple, Union

from walkie_sdk.core.interfaces import ROSTransportInterface
from walkie_sdk.utils.namespace import apply_namespace

# visualization_msgs/msg/Marker type constants
ARROW = 0
CUBE = 1
SPHERE = 2
CYLINDER = 3
LINE_STRIP = 4
LINE_LIST = 5
CUBE_LIST = 6
SPHERE_LIST = 7
POINTS = 8
TEXT_VIEW_FACING = 9
MESH_RESOURCE = 10
TRIANGLE_LIST = 11

# Marker action constants
ADD = 0
MODIFY = 0  # Same as ADD
DELETE = 2
DELETEALL = 3

# ROS message types
MARKER_MSG_TYPE = "visualization_msgs/msg/Marker"
MARKER_ARRAY_MSG_TYPE = "visualization_msgs/msg/MarkerArray"
POSE_STAMPED_MSG_TYPE = "geometry_msgs/msg/PoseStamped"

# Default topic names
DEFAULT_MARKER_TOPIC = "walkie/viz_markers"
DEFAULT_MARKER_ARRAY_TOPIC = "walkie/viz_markers_array"
DEFAULT_POSE_TOPIC = "walkie/target_pose"


def _build_marker_msg(
    marker_id: int,
    position: List[float],
    quaternion: List[float],
    frame_id: str = "base_link",
    marker_type: int = ARROW,
    action: int = ADD,
    scale: Optional[List[float]] = None,
    color: Optional[List[float]] = None,
    lifetime: float = 0.0,
    ns: str = "",
    text: str = "",
    frame_locked: bool = False,
) -> Dict[str, Any]:
    """
    Build a visualization_msgs/msg/Marker dict.

    Args:
        marker_id: Unique marker ID.
        position: [x, y, z] position.
        quaternion: [x, y, z, w] orientation.
        frame_id: TF reference frame (default: "base_link").
        marker_type: Marker type constant (ARROW, CUBE, SPHERE, etc.).
        action: Marker action (ADD, DELETE, DELETEALL).
        scale: [sx, sy, sz] scale in meters. Defaults depend on marker type.
        color: [r, g, b, a] color values (0.0-1.0). Default: red, fully opaque.
        lifetime: Duration in seconds (0 = forever).
        ns: Marker namespace string for grouping.
        text: Text content (only used for TEXT_VIEW_FACING type).
        frame_locked: If True, marker is re-transformed on each frame.

    Returns:
        Dict representing the Marker message.
    """
    if scale is None:
        if marker_type == ARROW:
            scale = [0.2, 0.05, 0.05]  # length, shaft diameter, head diameter
        elif marker_type == TEXT_VIEW_FACING:
            scale = [0.0, 0.0, 0.1]  # only z matters for text height
        else:
            scale = [0.1, 0.1, 0.1]

    if color is None:
        color = [1.0, 0.0, 0.0, 1.0]  # red, fully opaque

    # Compute lifetime sec/nanosec
    lifetime_sec = int(lifetime)
    lifetime_nanosec = int((lifetime - lifetime_sec) * 1e9)

    return {
        "header": {
            "frame_id": str(frame_id),
            "stamp": {"sec": 0, "nanosec": 0},
        },
        "ns": str(ns),
        "id": int(marker_id),
        "type": int(marker_type),
        "action": int(action),
        "pose": {
            "position": {
                "x": float(position[0]),
                "y": float(position[1]),
                "z": float(position[2]),
            },
            "orientation": {
                "x": float(quaternion[0]),
                "y": float(quaternion[1]),
                "z": float(quaternion[2]),
                "w": float(quaternion[3]),
            },
        },
        "scale": {
            "x": float(scale[0]),
            "y": float(scale[1]),
            "z": float(scale[2]),
        },
        "color": {
            "r": float(color[0]),
            "g": float(color[1]),
            "b": float(color[2]),
            "a": float(color[3]),
        },
        "lifetime": {"sec": lifetime_sec, "nanosec": lifetime_nanosec},
        "frame_locked": bool(frame_locked),
        "text": str(text),
    }


def _build_pose_stamped_msg(
    position: List[float],
    quaternion: List[float],
    frame_id: str = "base_link",
) -> Dict[str, Any]:
    """
    Build a geometry_msgs/msg/PoseStamped dict.

    Args:
        position: [x, y, z] position.
        quaternion: [x, y, z, w] orientation.
        frame_id: TF reference frame (default: "base_link").

    Returns:
        Dict representing the PoseStamped message.
    """
    return {
        "header": {
            "frame_id": str(frame_id),
            "stamp": {"sec": 0, "nanosec": 0},
        },
        "pose": {
            "position": {
                "x": float(position[0]),
                "y": float(position[1]),
                "z": float(position[2]),
            },
            "orientation": {
                "x": float(quaternion[0]),
                "y": float(quaternion[1]),
                "z": float(quaternion[2]),
                "w": float(quaternion[3]),
            },
        },
    }


class Visualization:
    """
    RViz2 marker visualization controller.

    Provides methods to publish visualization markers that can be
    displayed in RViz2. Supports single markers and marker arrays.

    This class works with any transport that implements ROSTransportInterface,
    making it protocol-agnostic (works with rosbridge, zenoh, etc.).

    Args:
        transport: Transport instance implementing ROSTransportInterface
        namespace: ROS namespace prefix for topics (default: "" = no namespace)

    Example:
        ```python
        viz = Visualization(transport, namespace="robot1")
        viz.draw_marker(
            position=[1.0, 2.0, 0.0],
            quaternion=[0.0, 0.0, 0.0, 1.0],
            frame_id="base_link",
        )
        ```
    """

    def __init__(self, transport: ROSTransportInterface, namespace: str = ""):
        self._transport = transport
        self._namespace = namespace
        self._next_id = 0
        self._id_lock = threading.Lock()
        # Cache of marker parameters keyed by (topic, marker_id) for update_marker()
        self._markers: Dict[tuple, Dict[str, Any]] = {}
        self._markers_lock = threading.Lock()
        # Cache of pose parameters keyed by topic for update_pose()
        self._poses: Dict[str, Dict[str, Any]] = {}
        self._poses_lock = threading.Lock()

    @property
    def namespace(self) -> str:
        """Current ROS namespace."""
        return self._namespace

    @namespace.setter
    def namespace(self, value: str) -> None:
        """Set ROS namespace for topics."""
        self._namespace = value

    def _get_next_id(self) -> int:
        """Thread-safe auto-incrementing marker ID."""
        with self._id_lock:
            mid = self._next_id
            self._next_id += 1
            return mid

    def _resolve_topic(self, topic: str) -> str:
        """Apply namespace to topic name."""
        return apply_namespace(topic, self._namespace)

    def draw_marker(
        self,
        position: List[float],
        quaternion: Optional[List[float]] = None,
        frame_id: str = "base_link",
        marker_type: int = ARROW,
        scale: Optional[List[float]] = None,
        color: Optional[List[float]] = None,
        marker_id: Optional[int] = None,
        lifetime: float = 0.0,
        ns: str = "",
        text: str = "",
        frame_locked: bool = False,
        topic: str = DEFAULT_MARKER_TOPIC,
    ) -> int:
        """
        Publish a single visualization marker to RViz2.

        Args:
            position: [x, y, z] position in the reference frame.
            quaternion: [x, y, z, w] orientation quaternion. Defaults to identity [0,0,0,1].
            frame_id: TF reference frame (default: "base_link").
            marker_type: Marker shape type (default: ARROW). Use module constants:
                ARROW (0), CUBE (1), SPHERE (2), etc.
            scale: [sx, sy, sz] marker scale in meters.
                Defaults: ARROW=[0.2, 0.05, 0.05], others=[0.1, 0.1, 0.1].
            color: [r, g, b, a] color (0.0-1.0). Default: [1.0, 0.0, 0.0, 1.0] (red).
            marker_id: Unique ID for this marker. Auto-assigned if None.
                Publishing with the same ID replaces the previous marker.
            lifetime: How long the marker persists in seconds (0 = forever).
            ns: Namespace string for grouping markers in RViz2.
            text: Text content (only for TEXT_VIEW_FACING markers).
            frame_locked: If True, marker is re-transformed each frame.
            topic: ROS topic to publish on (default: "walkie/viz_markers").

        Returns:
            The marker ID that was used (useful when auto-assigned).

        Raises:
            ConnectionError: If not connected to ROS transport.

        Example:
            ```python
            # Draw a red arrow at position (1, 2, 0)
            viz.draw_marker([1.0, 2.0, 0.0])

            # Draw a green sphere with custom scale
            viz.draw_marker(
                position=[3.0, 0.0, 0.5],
                quaternion=[0.0, 0.0, 0.0, 1.0],
                frame_id="map",
                marker_type=SPHERE,
                color=[0.0, 1.0, 0.0, 0.8],
                scale=[0.2, 0.2, 0.2],
            )
            ```
        """
        if not self._transport.is_connected:
            raise ConnectionError("Not connected to robot")

        if quaternion is None:
            quaternion = [0.0, 0.0, 0.0, 1.0]

        if marker_id is None:
            marker_id = self._get_next_id()

        # Resolve defaults for caching (so update_marker has concrete values)
        if scale is None:
            if marker_type == ARROW:
                scale = [0.2, 0.05, 0.05]
            elif marker_type == TEXT_VIEW_FACING:
                scale = [0.0, 0.0, 0.1]
            else:
                scale = [0.1, 0.1, 0.1]
        if color is None:
            color = [1.0, 0.0, 0.0, 1.0]

        msg = _build_marker_msg(
            marker_id=marker_id,
            position=position,
            quaternion=quaternion,
            frame_id=frame_id,
            marker_type=marker_type,
            action=ADD,
            scale=scale,
            color=color,
            lifetime=lifetime,
            ns=ns,
            text=text,
            frame_locked=frame_locked,
        )

        full_topic = self._resolve_topic(topic)
        self._transport.publish(full_topic, MARKER_MSG_TYPE, msg)

        # Cache params for update_marker()
        cache_key = (topic, marker_id)
        with self._markers_lock:
            self._markers[cache_key] = {
                "position": list(position),
                "quaternion": list(quaternion),
                "frame_id": frame_id,
                "marker_type": marker_type,
                "scale": list(scale),
                "color": list(color),
                "lifetime": lifetime,
                "ns": ns,
                "text": text,
                "frame_locked": frame_locked,
                "topic": topic,
            }

        return marker_id

    def update_marker(
        self,
        marker_id: int,
        position: Optional[List[float]] = None,
        quaternion: Optional[List[float]] = None,
        frame_id: Optional[str] = None,
        marker_type: Optional[int] = None,
        scale: Optional[List[float]] = None,
        color: Optional[List[float]] = None,
        lifetime: Optional[float] = None,
        ns: Optional[str] = None,
        text: Optional[str] = None,
        frame_locked: Optional[bool] = None,
        topic: str = DEFAULT_MARKER_TOPIC,
    ) -> None:
        """
        Update an existing marker with only the changed fields.

        Merges the provided parameters with the cached values from the
        original draw_marker() call, then republishes. Only pass the
        fields you want to change.

        Args:
            marker_id: ID of the marker to update (must have been created with draw_marker()).
            position: New [x, y, z] position (or None to keep current).
            quaternion: New [x, y, z, w] orientation (or None to keep current).
            frame_id: New reference frame (or None to keep current).
            marker_type: New marker type (or None to keep current).
            scale: New [sx, sy, sz] scale (or None to keep current).
            color: New [r, g, b, a] color (or None to keep current).
            lifetime: New lifetime in seconds (or None to keep current).
            ns: New namespace (or None to keep current).
            text: New text content (or None to keep current).
            frame_locked: New frame_locked flag (or None to keep current).
            topic: ROS topic the marker was published on.

        Raises:
            ConnectionError: If not connected to ROS transport.
            KeyError: If the marker_id was not previously created with draw_marker().

        Example:
            ```python
            # Create a marker
            mid = viz.draw_marker([0, 0, 0], [0, 0, 0, 1])

            # Update only its position (everything else stays the same)
            viz.update_marker(mid, position=[1.0, 2.0, 0.0])

            # Update position and color together
            viz.update_marker(mid, position=[3.0, 0.0, 0.0], color=[0, 1, 0, 1])
            ```
        """
        if not self._transport.is_connected:
            raise ConnectionError("Not connected to robot")

        cache_key = (topic, marker_id)
        with self._markers_lock:
            if cache_key not in self._markers:
                raise KeyError(
                    f"Marker id={marker_id} on topic='{topic}' not found. "
                    f"Create it first with draw_marker()."
                )
            cached = self._markers[cache_key].copy()

        # Merge: only override fields that were explicitly provided
        if position is not None:
            cached["position"] = list(position)
        if quaternion is not None:
            cached["quaternion"] = list(quaternion)
        if frame_id is not None:
            cached["frame_id"] = frame_id
        if marker_type is not None:
            cached["marker_type"] = marker_type
        if scale is not None:
            cached["scale"] = list(scale)
        if color is not None:
            cached["color"] = list(color)
        if lifetime is not None:
            cached["lifetime"] = lifetime
        if ns is not None:
            cached["ns"] = ns
        if text is not None:
            cached["text"] = text
        if frame_locked is not None:
            cached["frame_locked"] = frame_locked

        msg = _build_marker_msg(
            marker_id=marker_id,
            position=cached["position"],
            quaternion=cached["quaternion"],
            frame_id=cached["frame_id"],
            marker_type=cached["marker_type"],
            action=ADD,
            scale=cached["scale"],
            color=cached["color"],
            lifetime=cached["lifetime"],
            ns=cached["ns"],
            text=cached["text"],
            frame_locked=cached["frame_locked"],
        )

        full_topic = self._resolve_topic(topic)
        self._transport.publish(full_topic, MARKER_MSG_TYPE, msg)

        # Update the cache with merged values
        with self._markers_lock:
            self._markers[cache_key] = cached

    def draw_markers(
        self,
        markers: List[Dict[str, Any]],
        topic: str = DEFAULT_MARKER_ARRAY_TOPIC,
    ) -> List[int]:
        """
        Publish multiple markers as a MarkerArray to RViz2.

        Args:
            markers: List of marker parameter dicts. Each dict accepts the same
                keyword arguments as draw_marker():
                - position (required): [x, y, z]
                - quaternion: [x, y, z, w] (optional, default identity)
                - frame_id: str (default: "base_link")
                - marker_type: int (default: ARROW)
                - scale: [sx, sy, sz]
                - color: [r, g, b, a]
                - marker_id: int (auto-assigned if omitted)
                - lifetime: float
                - ns: str
                - text: str
                - frame_locked: bool
            topic: ROS topic to publish on (default: "walkie/viz_markers_array").

        Returns:
            List of marker IDs that were used.

        Raises:
            ConnectionError: If not connected to ROS transport.

        Example:
            ```python
            viz.draw_markers([
                {
                    "position": [1.0, 0.0, 0.0],
                    # quaternion defaults to identity
                    # marker_type defaults to ARROW
                    "color": [1.0, 0.0, 0.0, 1.0],
                },
                {
                    "position": [2.0, 0.0, 0.0],
                    "marker_type": SPHERE,
                    "color": [0.0, 1.0, 0.0, 1.0],
                },
            ])
            ```
        """
        if not self._transport.is_connected:
            raise ConnectionError("Not connected to robot")

        marker_msgs = []
        used_ids = []

        for params in markers:
            mid = params.get("marker_id")
            if mid is None:
                mid = self._get_next_id()

            msg = _build_marker_msg(
                marker_id=mid,
                position=params["position"],
                quaternion=params.get("quaternion", [0.0, 0.0, 0.0, 1.0]),
                frame_id=params.get("frame_id", "base_link"),
                marker_type=params.get("marker_type", ARROW),
                action=ADD,
                scale=params.get("scale"),
                color=params.get("color"),
                lifetime=params.get("lifetime", 0.0),
                ns=params.get("ns", ""),
                text=params.get("text", ""),
                frame_locked=params.get("frame_locked", False),
            )

            marker_msgs.append(msg)
            used_ids.append(mid)

        array_msg = {"markers": marker_msgs}
        full_topic = self._resolve_topic(topic)
        self._transport.publish(full_topic, MARKER_ARRAY_MSG_TYPE, array_msg)
        return used_ids

    def delete_marker(
        self,
        marker_id: int,
        ns: str = "",
        topic: str = DEFAULT_MARKER_TOPIC,
    ) -> None:
        """
        Delete a specific marker by ID.

        Args:
            marker_id: ID of the marker to delete.
            ns: Namespace of the marker to delete.
            topic: ROS topic the marker was published on.

        Raises:
            ConnectionError: If not connected to ROS transport.
        """
        if not self._transport.is_connected:
            raise ConnectionError("Not connected to robot")

        msg = _build_marker_msg(
            marker_id=marker_id,
            position=[0.0, 0.0, 0.0],
            quaternion=[0.0, 0.0, 0.0, 1.0],
            action=DELETE,
            ns=ns,
        )

        full_topic = self._resolve_topic(topic)
        self._transport.publish(full_topic, MARKER_MSG_TYPE, msg)

        # Remove from cache
        cache_key = (topic, marker_id)
        with self._markers_lock:
            self._markers.pop(cache_key, None)

    def clear_markers(
        self,
        ns: str = "",
        topic: str = DEFAULT_MARKER_TOPIC,
    ) -> None:
        """
        Clear all markers from RViz2.

        Publishes a DELETEALL marker action to remove all markers
        on the specified topic and namespace.

        Args:
            ns: Namespace of markers to clear (default: "" = all).
            topic: ROS topic to publish the clear command on.

        Raises:
            ConnectionError: If not connected to ROS transport.

        Example:
            ```python
            viz.clear_markers()
            ```
        """
        if not self._transport.is_connected:
            raise ConnectionError("Not connected to robot")

        msg = _build_marker_msg(
            marker_id=0,
            position=[0.0, 0.0, 0.0],
            quaternion=[0.0, 0.0, 0.0, 1.0],
            action=DELETEALL,
            ns=ns,
        )

        full_topic = self._resolve_topic(topic)
        self._transport.publish(full_topic, MARKER_MSG_TYPE, msg)

        # Reset auto-increment counter and clear cache
        with self._id_lock:
            self._next_id = 0
        with self._markers_lock:
            self._markers.clear()

    def draw_pose(
        self,
        position: List[float],
        quaternion: Optional[List[float]] = None,
        frame_id: str = "base_link",
        topic: str = DEFAULT_POSE_TOPIC,
    ) -> str:
        """
        Publish a PoseStamped message to RViz2.

        This publishes a geometry_msgs/msg/PoseStamped on the specified topic,
        which can be visualized in RViz2 using the built-in "Pose" display type.
        Use different topics for different poses (e.g. one per arm group).

        Args:
            position: [x, y, z] position in the reference frame.
            quaternion: [x, y, z, w] orientation quaternion. Defaults to identity [0,0,0,1].
            frame_id: TF reference frame (default: "base_link").
            topic: ROS topic to publish on (default: "walkie/target_pose").
                Use unique topics for multiple simultaneous poses, e.g.
                "walkie/target_pose/left_arm".

        Returns:
            The topic string that was used (useful for later update_pose() calls).

        Raises:
            ConnectionError: If not connected to ROS transport.

        Example:
            ```python
            # Draw pose with default identity orientation
            viz.draw_pose([1.0, 2.0, 0.0])
            # returns 'walkie/target_pose'

            # Multiple poses on different topics
            viz.draw_pose(
                position=[0.5, 0.0, 0.3],
                quaternion=[0.0, 0.0, 0.0, 1.0],
                topic="walkie/target_pose/left_arm",
            )
            # returns 'walkie/target_pose/left_arm'
            ```
        """
        if not self._transport.is_connected:
            raise ConnectionError("Not connected to robot")

        if quaternion is None:
            quaternion = [0.0, 0.0, 0.0, 1.0]

        msg = _build_pose_stamped_msg(
            position=position,
            quaternion=quaternion,
            frame_id=frame_id,
        )

        full_topic = self._resolve_topic(topic)
        self._transport.publish(full_topic, POSE_STAMPED_MSG_TYPE, msg)

        # Cache params for update_pose()
        with self._poses_lock:
            self._poses[topic] = {
                "position": list(position),
                "quaternion": list(quaternion),
                "frame_id": frame_id,
            }

        return topic

    def update_pose(
        self,
        position: Optional[List[float]] = None,
        quaternion: Optional[List[float]] = None,
        frame_id: Optional[str] = None,
        topic: str = DEFAULT_POSE_TOPIC,
    ) -> None:
        """
        Update an existing PoseStamped with only the changed fields.

        Merges the provided parameters with the cached values from the
        original draw_pose() call, then republishes. Only pass the
        fields you want to change.

        Args:
            position: New [x, y, z] position (or None to keep current).
            quaternion: New [x, y, z, w] orientation (or None to keep current).
            frame_id: New reference frame (or None to keep current).
            topic: ROS topic the pose was published on.

        Raises:
            ConnectionError: If not connected to ROS transport.
            KeyError: If the topic was not previously used with draw_pose().

        Example:
            ```python
            topic = viz.draw_pose([0, 0, 0], [0, 0, 0, 1])

            # Update only position (orientation stays the same)
            viz.update_pose(position=[1.0, 2.0, 0.0], topic=topic)
            ```
        """
        if not self._transport.is_connected:
            raise ConnectionError("Not connected to robot")

        with self._poses_lock:
            if topic not in self._poses:
                raise KeyError(
                    f"Pose on topic='{topic}' not found. "
                    f"Create it first with draw_pose()."
                )
            cached = self._poses[topic].copy()

        # Merge: only override fields that were explicitly provided
        if position is not None:
            cached["position"] = list(position)
        if quaternion is not None:
            cached["quaternion"] = list(quaternion)
        if frame_id is not None:
            cached["frame_id"] = frame_id

        msg = _build_pose_stamped_msg(
            position=cached["position"],
            quaternion=cached["quaternion"],
            frame_id=cached["frame_id"],
        )

        full_topic = self._resolve_topic(topic)
        self._transport.publish(full_topic, POSE_STAMPED_MSG_TYPE, msg)

        # Update the cache with merged values
        with self._poses_lock:
            self._poses[topic] = cached
