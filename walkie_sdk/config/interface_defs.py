"""Bundled ROS interface definitions for the zenoh transport.

The zenoh transport uses ``zenoh_ros2_sdk``, which auto-resolves the *standard*
ROS repos (``std_msgs``, ``geometry_msgs``, ``nav_msgs``, ``sensor_msgs`` …) by
cloning them. It can NOT resolve:

* project-specific packages (``walkie_tf_interfaces``, ``walkie_perception``,
  ``my_robot_interfaces``, ``robot_navigation``), and
* a few standard-but-unclonable packages — notably ``vision_msgs``, which is
  nested inside ``walkie_perception`` services.

So the ``.srv``/``.msg`` text for those is **bundled as real files** under
``walkie_sdk/config/interfaces/<pkg>/{srv,msg}/`` (kept in sync from the ROS
packages by ``scripts/sync_interfaces.py``) and consumed two ways:

1. :func:`srv_definitions` / :func:`msg_definition` return the bundled text for a
   type (the SDK passes it explicitly when creating subscribers/publishers/
   service clients). Returns ``None`` for anything not bundled — the signal to
   let ``zenoh_ros2_sdk`` resolve it itself.
2. :func:`seed_registry` points ``zenoh_ros2_sdk``'s message registry at this
   same directory so it can resolve types that appear **nested** inside a
   service definition (e.g. ``vision_msgs/Detection2DArray`` inside
   ``walkie_perception/srv/GetObPose``), which step 1 alone does not cover.

The client stays ROS-free: everything needed is committed text, no ROS install
or workspace required at runtime.
"""

from __future__ import annotations

from pathlib import Path
from typing import Dict, Optional, Tuple

# Directory holding the bundled interface files (populated by
# scripts/sync_interfaces.py), laid out as <pkg>/{srv,msg}/<Name>.{srv,msg}.
INTERFACES_DIR = Path(__file__).parent / "interfaces"


def _load_defs(kind: str) -> Dict[str, str]:
    """Load every bundled ``.{kind}`` file into a ``{"<pkg>/<kind>/<Name>": text}``
    dict (kind is "srv" or "msg")."""
    defs: Dict[str, str] = {}
    if not INTERFACES_DIR.is_dir():
        return defs
    for path in INTERFACES_DIR.glob(f"*/{kind}/*.{kind}"):
        pkg = path.parents[1].name
        defs[f"{pkg}/{kind}/{path.stem}"] = path.read_text()
    return defs


_SRV_DEFS: Dict[str, str] = _load_defs("srv")
_MSG_DEFS: Dict[str, str] = _load_defs("msg")

_registry_seeded = False


def seed_registry() -> None:
    """Point ``zenoh_ros2_sdk``'s message registry at the bundled interfaces.

    The registry resolves nested message types inside service definitions from a
    ``messages_dir`` (then falls back to git-cloning the standard repos). Seeding
    it with our bundle makes project-specific and non-clonable types (notably
    ``vision_msgs``) resolvable; standard types still clone as before, since our
    bundle doesn't shadow them.

    Must run before the SDK creates its registry singleton (the first call wins)
    and after the Zenoh session exists (preloading registers into it), so the
    zenoh transport calls this in ``connect()`` right after the session is up.
    Idempotent, best-effort.
    """
    global _registry_seeded
    if _registry_seeded:
        return
    _registry_seeded = True
    try:
        from zenoh_ros2_sdk.message_registry import get_registry

        reg = get_registry(messages_dir=str(INTERFACES_DIR))
        # The registry only *finds* files lazily; a type nested inside a service
        # definition (e.g. vision_msgs/Detection2DArray inside GetObPose) must be
        # registered into the rosbags store up front or (de)serialization raises
        # KeyError. Preload every bundled message type (load pulls its deps too).
        for msg_type in _MSG_DEFS:
            try:
                reg.load_message_type(msg_type)
            except Exception:
                pass
    except Exception as e:  # pragma: no cover - best effort; zenoh may be absent
        print(f"[interface_defs] could not seed message registry: {e}")


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

    Returns ``(None, None)`` for standard/unbundled types so the caller can let
    ``zenoh_ros2_sdk`` resolve them automatically.
    """
    text = _SRV_DEFS.get(srv_type)
    if text is None:
        return None, None
    return _split_srv(text)


def msg_definition(msg_type: str) -> Optional[str]:
    """Return the .msg definition for a bundled message type, else ``None``."""
    return _MSG_DEFS.get(msg_type)
