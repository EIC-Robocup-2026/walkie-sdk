"""Bundled ROS interface definitions for project-specific (custom) packages.

The zenoh transport uses ``zenoh_ros2_sdk``, which resolves **standard** ROS
types (``std_msgs``, ``geometry_msgs``, ``sensor_msgs``, ``vision_msgs``, …) on
its own by cloning known message repositories. Project-specific packages
(``walkie_tf_interfaces``, ``walkie_perception``, ``my_robot_interfaces``) are
not in any known repository, so a zenoh subscribe/publish/service-call for one
of those types fails with *"not found in registry"*.

To keep the client ROS-free, the ``.srv``/``.msg`` text for those custom types
is bundled here and passed explicitly to ``zenoh_ros2_sdk`` (which accepts
``request_definition``/``response_definition`` for services and
``msg_definition`` for topics). Lookups return ``None`` for any type not listed
here, which is the signal to let ``zenoh_ros2_sdk`` resolve it automatically.

Definitions may reference standard nested types (e.g. ``GetObPose`` uses
``vision_msgs/Detection2DArray`` and ``geometry_msgs/PoseArray``);
``zenoh_ros2_sdk`` resolves those automatically. Only nested *custom* types
would need their own entry here.
"""

from __future__ import annotations

from typing import Dict, Optional, Tuple

# ── Custom service definitions ──────────────────────────────────────────────
# Keyed by full service type "<pkg>/srv/<Name>". Raw .srv text: everything
# before the "---" separator line is the request, everything after is the
# response.
_SRV_DEFS: Dict[str, str] = {
    "walkie_tf_interfaces/srv/GetTransform": """\
string source_frame
string target_frame
float64 timeout_sec
---
bool success
string message
float64 x
float64 y
float64 z
float64 qx
float64 qy
float64 qz
float64 qw
""",
    "walkie_perception/srv/GetObPose": """\
vision_msgs/Detection2DArray detections
---
geometry_msgs/PoseArray poses
bool success
""",
    "my_robot_interfaces/srv/GetEEPose": """\
string group_name
string frame_id
---
float64 x
float64 y
float64 z
float64 qx
float64 qy
float64 qz
float64 qw
string frame_id
bool success
string status
""",
    "my_robot_interfaces/srv/GetJointStates": """\
string group_name
---
string[] joint_names
float64[] joint_positions
bool success
string status
""",
    "my_robot_interfaces/srv/ToggleGripperCollision": """\
string group_name
bool enable
---
bool success
string status
""",
}

# ── Custom message definitions ──────────────────────────────────────────────
# Keyed by full message type "<pkg>/msg/<Name>". All topics the SDK currently
# uses are standard types (resolved automatically), so this is empty; add custom
# message defs here if a future topic uses a project-specific message type.
_MSG_DEFS: Dict[str, str] = {}


def _split_srv(text: str) -> Tuple[str, str]:
    """Split raw .srv text into (request_definition, response_definition)."""
    lines = text.splitlines()
    for i, line in enumerate(lines):
        if line.strip() == "---":
            request = "\n".join(lines[:i]).strip()
            response = "\n".join(lines[i + 1:]).strip()
            return request, response
    # No separator: treat the whole thing as the request, empty response.
    return text.strip(), ""


def srv_definitions(srv_type: str) -> Tuple[Optional[str], Optional[str]]:
    """Return (request_definition, response_definition) for a custom service.

    Returns ``(None, None)`` for standard/unknown types so the caller can let
    ``zenoh_ros2_sdk`` resolve them automatically.
    """
    text = _SRV_DEFS.get(srv_type)
    if text is None:
        return None, None
    return _split_srv(text)


def msg_definition(msg_type: str) -> Optional[str]:
    """Return the .msg definition for a custom message type, else ``None``."""
    return _MSG_DEFS.get(msg_type)
