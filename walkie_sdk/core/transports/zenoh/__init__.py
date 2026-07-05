"""
Walkie SDK - Zenoh Transport

Zenoh-based transport implementation for ROS2 communication using the zenoh_ros2_sdk.
This handles CDR serialization, discovery, and type support automatically.
"""

from __future__ import annotations

import dataclasses
import re
import threading
import time
from typing import Any, Callable, Dict, List, Optional, Tuple

import numpy as np

# Import Zenoh and the SDK components
try:
    import zenoh
    from zenoh_ros2_sdk import (
        ROS2Publisher,
        ROS2ServiceClient,
        ROS2Subscriber,
        ZenohSession,
    )

    ZENOH_AVAILABLE = True
except ImportError:
    ZENOH_AVAILABLE = False
    zenoh = None

try:
    import cv2
except ImportError:
    cv2 = None

from walkie_sdk.core.interfaces import CameraTransportInterface, ROSTransportInterface
from walkie_sdk.config.ros_topics import CAMERA_TOPICS, DEPTH_TOPICS, ROS_DOMAIN_ID
from walkie_sdk.config.interface_defs import msg_definition, srv_definitions, seed_registry


def _msg_to_dict(msg: Any) -> Dict[str, Any]:
    """
    Recursively convert a rosbags/SDK message object to a dictionary.
    This bridges the gap between the SDK's object-oriented returns and the
    Walkie SDK's dictionary-based interface.
    """
    if hasattr(msg, "__dataclass_fields__"):
        result = {}
        for field in msg.__dataclass_fields__:
            value = getattr(msg, field)
            result[field] = _msg_to_dict(value)
        return result
    elif isinstance(msg, list):
        return [_msg_to_dict(x) for x in msg]
    elif isinstance(msg, (bytes, bytearray)):
        return msg
    elif hasattr(msg, "tolist"):  # numpy arrays
        return msg.tolist()
    else:
        return msg


# ── Message completion ─────────────────────────────────────────────────────
# The rosbags message classes used by zenoh_ros2_sdk are dataclasses whose
# fields have NO defaults — constructing one requires every field, and the CDR
# serializer needs real nested instances (not dicts) and numpy arrays (not
# lists). Modules, however, publish *partial* dicts (rosbridge silently filled
# the rest). These helpers complete a partial dict into full constructor kwargs
# so any module's message publishes correctly over Zenoh.

_PRIMITIVE_DEFAULTS: Dict[str, Any] = {
    "str": "",
    "bool": False,
    "int": 0,
    "float": 0.0,
    "bytes": b"",
}


def _np_dtype(type_str: str):
    """Extract the numpy dtype from an annotation like
    'np.ndarray[tuple[int, ...], np.dtype[np.float64]]'."""
    m = re.search(r"np\.dtype\[np\.(\w+)\]", type_str)
    return getattr(np, m.group(1)) if m else np.float64


def _is_message_type(type_str: str) -> bool:
    """True if the field annotation refers to a nested ROS message dataclass."""
    return not (
        type_str in _PRIMITIVE_DEFAULTS
        or type_str.startswith("np.ndarray")
        or type_str.startswith("list[")
        or type_str.startswith("ClassVar")
    )


def _coerce_value(type_str: str, value: Any, resolve: Callable[[str], Any]) -> Any:
    """Convert a provided value to what the dataclass/serializer expects."""
    if type_str.startswith("np.ndarray"):
        return np.asarray(value, dtype=_np_dtype(type_str))
    if type_str.startswith("list[") and isinstance(value, list):
        inner = type_str[5:-1]
        return [_coerce_value(inner, v, resolve) for v in value]
    if _is_message_type(type_str) and isinstance(value, dict):
        return _build_instance(resolve(type_str), value, resolve)
    return value


def _default_value(type_str: str, resolve: Callable[[str], Any]) -> Any:
    """Build a type-appropriate default for a missing required field."""
    if type_str in _PRIMITIVE_DEFAULTS:
        return _PRIMITIVE_DEFAULTS[type_str]
    if type_str.startswith("np.ndarray"):
        return np.array([], dtype=_np_dtype(type_str))
    if type_str.startswith("list["):
        return []
    if _is_message_type(type_str):
        return _build_instance(resolve(type_str), {}, resolve)
    return None


def _complete_kwargs(
    msg_class: Any, data: Any, resolve: Callable[[str], Any]
) -> Dict[str, Any]:
    """Return full constructor kwargs for ``msg_class`` from a partial ``data``
    dict: provided fields are coerced, missing required fields get defaults, and
    ClassVar constants / dunder fields are left to the dataclass."""
    out: Dict[str, Any] = {}
    for f in dataclasses.fields(msg_class):
        if f.name.startswith("__"):
            continue
        has_default = (
            f.default is not dataclasses.MISSING
            or f.default_factory is not dataclasses.MISSING
        )
        if isinstance(data, dict) and f.name in data:
            out[f.name] = _coerce_value(f.type, data[f.name], resolve)
        elif not has_default:
            out[f.name] = _default_value(f.type, resolve)
    return out


def _build_instance(msg_class: Any, data: Any, resolve: Callable[[str], Any]) -> Any:
    return msg_class(**_complete_kwargs(msg_class, data, resolve))


class ZenohTransport(ROSTransportInterface[Any]):
    """
    ROS transport implementation using Zenoh SDK.
    """

    def __init__(
        self,
        host: str,
        port: int = 7447,
        timeout: float = 10.0,
        **kwargs,
    ):
        if not ZENOH_AVAILABLE:
            raise ImportError(
                "zenoh_ros2_sdk is not installed or accessible. Please install zenoh and the SDK."
            )

        self._host = host
        self._port = port
        self._timeout = timeout

        self._session_mgr = None
        self._lock = threading.Lock()

        # Cache SDK entities
        self._subscribers: Dict[str, ROS2Subscriber] = {}
        # Keyed by (topic, latch) — latched publishers use different QoS
        self._publishers: Dict[Tuple[str, bool], ROS2Publisher] = {}
        self._service_clients: Dict[str, ROS2ServiceClient] = {}
        # Cache resolved nested message classes (for message completion)
        self._msg_class_cache: Dict[str, Any] = {}
        # Cache discovered service type-hashes (service_name -> RIHS01 hash | None)
        self._service_hash_cache: Dict[str, Optional[str]] = {}
        # Whether the rosbags-store deserialization alias patch is installed
        self._msgdef_patched = False

    @property
    def host(self) -> str:
        return self._host

    @property
    def port(self) -> int:
        return self._port

    @property
    def is_connected(self) -> bool:
        return self._session_mgr is not None

    def connect(self) -> None:
        """Connect to Zenoh router via SDK Session."""
        print(f"  → Connecting to Zenoh Router at {self._host}:{self._port}...")

        try:
            # Initialize the Singleton Session
            self._session_mgr = ZenohSession.get_instance(
                router_ip=self._host, router_port=self._port
            )
            # Point zenoh_ros2_sdk's message registry at our bundled interface
            # files and preload them, so nested non-clonable types (e.g.
            # vision_msgs inside walkie_perception services) resolve. Done after
            # the session exists because preloading registers into it.
            seed_registry()
            print(
                f"  ✓ Connected to Zenoh (Session ID: {self._session_mgr.session_id})"
            )
        except Exception as e:
            self._session_mgr = None
            raise ConnectionError(f"Failed to connect to Zenoh: {e}")

    def disconnect(self) -> None:
        """Close all entities and session."""
        with self._lock:
            for sub in self._subscribers.values():
                sub.close()
            for pub in self._publishers.values():
                pub.close()
            for client in self._service_clients.values():
                client.close()

            self._subscribers.clear()
            self._publishers.clear()
            self._service_clients.clear()

            # Reset session reference but don't close singleton (Camera might use it)
            self._session_mgr = None

    def subscribe(
        self,
        topic: str,
        message_type: str,
        callback: Callable[[Dict[str, Any]], None],
        throttle_rate: int = 0,
        queue_size: int = 1,
    ) -> Any:
        """Subscribe using ROS2Subscriber."""
        if not self._session_mgr:
            raise ConnectionError("Not connected to Zenoh")

        with self._lock:
            if topic in self._subscribers:
                return self._subscribers[topic]

            print(f"[ZenohTransport] Subscribing to: {topic} ({message_type})")

            # Wrapper to convert SDK Object -> Dict
            def callback_wrapper(msg_obj):
                try:
                    msg_dict = _msg_to_dict(msg_obj)
                    callback(msg_dict)
                except Exception as e:
                    print(f"[ZenohTransport] Error in callback for {topic}: {e}")

            # Create Subscriber. Standard ROS 2 types resolve automatically;
            # custom (project-specific) types need their .msg text supplied.
            #
            # type_hash="*" wildcards the REP-2011 type hash in the data keyexpr
            # (<domain>/<topic>/<dds_type>/<type_hash>). The SDK's bundled message
            # definitions can hash differently from the robot's ROS distro (e.g.
            # sensor_msgs/PointCloud2), and an exact-hash subscription then matches
            # nothing even though the topic is publishing. Matching any hash keeps
            # the SDK distro-agnostic; the CDR wire layout for standard messages is
            # stable, so deserialization with our definition still works.
            sub = ROS2Subscriber(
                topic=topic,
                msg_type=message_type,
                callback=callback_wrapper,
                msg_definition=msg_definition(message_type),
                domain_id=ROS_DOMAIN_ID,
                router_ip=self._host,
                router_port=self._port,
                type_hash="*",
            )
            self._subscribers[topic] = sub
            return sub

    def unsubscribe(self, handle: Any) -> None:
        """Unsubscribe and close the SDK subscriber."""
        with self._lock:
            # Handle is the ROS2Subscriber instance
            if hasattr(handle, "topic") and handle.topic in self._subscribers:
                handle.close()
                del self._subscribers[handle.topic]
            elif hasattr(handle, "close"):
                handle.close()

    def publish(
        self,
        topic: str,
        message_type: str,
        message: Dict[str, Any],
        latch: bool = False,
    ) -> None:
        """Publish using ROS2Publisher.

        latch=True creates the publisher with reliable + transient_local QoS so
        it matches transient_local subscribers (e.g. Nav2 selector topics) and
        late joiners receive the last message.
        """
        if not self._session_mgr:
            raise ConnectionError("Not connected to Zenoh")

        # Latched and volatile publishers on the same topic must not share a
        # cache slot — their QoS differs.
        cache_key = (topic, latch)
        with self._lock:
            if cache_key not in self._publishers:
                qos = None
                if latch:
                    from zenoh_ros2_sdk.qos import (
                        QosDurability,
                        QosProfile,
                        QosReliability,
                    )
                    qos = QosProfile(
                        reliability=QosReliability.RELIABLE,
                        durability=QosDurability.TRANSIENT_LOCAL,
                    )
                # Create Publisher. Custom types need their .msg text supplied.
                self._publishers[cache_key] = ROS2Publisher(
                    topic=topic,
                    msg_type=message_type,
                    msg_definition=msg_definition(message_type),
                    domain_id=ROS_DOMAIN_ID,
                    router_ip=self._host,
                    router_port=self._port,
                    qos=qos,
                )

            pub = self._publishers[cache_key]

        # Complete the (often partial) message dict into full constructor kwargs:
        # fill missing required fields with defaults and turn nested dicts/lists
        # into proper message instances / numpy arrays. Best-effort — if anything
        # goes wrong, fall back to the raw message (original behavior).
        try:
            message = _complete_kwargs(pub.msg_class, message, self._resolve_msg_class)
        except Exception as e:
            print(f"[ZenohTransport] Warning: could not complete message for {topic}: {e}")

        # Publish using kwargs
        try:
            pub.publish(**message)
        except Exception as e:
            print(f"[ZenohTransport] Error publishing to {topic}: {e}")
            raise

    def _resolve_msg_class(self, type_str: str) -> Any:
        """Resolve a nested-message annotation (e.g. 'geometry_msgs__msg__Pose')
        to its rosbags dataclass, caching the result. Tolerates the service
        namespace mangling ('pkg__srv__msg__X' -> the real 'pkg/msg/X')."""
        cls = self._msg_class_cache.get(type_str)
        if cls is None:
            ros2_type = type_str.replace("__", "/")
            if "/srv/msg/" in ros2_type:
                ros2_type = ros2_type.replace("/srv/msg/", "/msg/")
            cls = self._session_mgr.register_message_type(None, ros2_type)
            self._msg_class_cache[type_str] = cls
        return cls

    def call_service(
        self,
        service_name: str,
        service_type: str,
        request: Dict[str, Any],
        timeout: float = 5.0,
    ) -> Dict[str, Any]:
        """Call service using ROS2ServiceClient."""
        if not self._session_mgr:
            raise ConnectionError("Not connected to Zenoh")

        # Services whose response embeds a nested message (e.g. nav_msgs/srv/GetMap
        # -> OccupancyGrid) deserialize the nested type under the service namespace
        # ('nav_msgs/srv/msg/OccupancyGrid'), which doesn't exist. Patch the store
        # once so those names alias the real 'nav_msgs/msg/OccupancyGrid'.
        self._install_msgdef_alias_patch()

        with self._lock:
            if service_name not in self._service_clients:
                # Standard service types resolve automatically; custom
                # (project-specific) types need their .srv text supplied.
                request_def, response_def = srv_definitions(service_type)
                # The SDK's bundled message defs can hash differently from the
                # robot's distro, so the computed service type-hash won't match
                # and the request never reaches the server. Discover the robot's
                # actual hash from the liveliness graph (None -> fall back to the
                # SDK's computed hash, which is correct for our custom services).
                real_hash = self._discover_service_hash(service_name)
                self._service_clients[service_name] = ROS2ServiceClient(
                    service_name=service_name,
                    srv_type=service_type,
                    request_definition=request_def,
                    response_definition=response_def,
                    domain_id=ROS_DOMAIN_ID,
                    timeout=timeout,
                    router_ip=self._host,
                    router_port=self._port,
                    **({"type_hash": real_hash} if real_hash else {}),
                )
            client = self._service_clients[service_name]

        # Complete the request like publishes: turn nested dicts into proper
        # message instances (the CDR serializer needs instances, not dicts) and
        # fill any missing required fields. Best-effort — fall back to the raw
        # request if completion fails.
        try:
            request = _complete_kwargs(client.request_msg_class, request, self._resolve_msg_class)
        except Exception as e:
            print(f"[ZenohTransport] Warning: could not complete request for {service_name}: {e}")

        try:
            # SDK call returns an object
            response_obj = client.call(**request)

            if response_obj is None:
                raise TimeoutError(
                    f"Service call to {service_name} timed out or failed"
                )

            return _msg_to_dict(response_obj)
        except Exception as e:
            print(f"[ZenohTransport] Error calling service {service_name}: {e}")
            raise

    def _install_msgdef_alias_patch(self) -> None:
        """Make the rosbags store resilient when (de)serializing service messages.

        Two problems show up only for services whose request/response embed nested
        messages:
        1. The nested field is named under the service namespace
           ('pkg/srv/msg/Other' instead of the real 'pkg/msg/Other').
        2. A referenced type may not be registered in the store yet (e.g. a
           standard ``geometry_msgs/msg/PoseArray`` that no topic uses).

        Wrap ``get_msgdef`` so a failed lookup (a) maps the service-namespaced
        name to the real one and (b) lazily loads the type from the registry
        (bundled dir, then clone) before retrying. Installed once per session."""
        if self._msgdef_patched:
            return
        store = self._session_mgr.store
        original = store.get_msgdef

        def get_msgdef(typename):
            try:
                return original(typename)
            except KeyError:
                real = (
                    typename.replace("/srv/msg/", "/msg/")
                    if "/srv/msg/" in typename
                    else typename
                )
                fielddefs = getattr(store, "fielddefs", {})
                if real not in fielddefs:
                    try:
                        from zenoh_ros2_sdk.message_registry import get_registry

                        get_registry().load_message_type(real)
                    except Exception:
                        pass
                if real in fielddefs:
                    if real != typename:  # alias the service-namespaced name
                        fielddefs[typename] = fielddefs[real]
                        types = getattr(store, "types", None)
                        if types is not None and real in types:
                            types[typename] = types[real]
                    return original(typename)
                raise

        store.get_msgdef = get_msgdef
        self._msgdef_patched = True

    def _discover_service_hash(self, service_name: str) -> Optional[str]:
        """Find the robot's actual REP-2011 type hash for a service by reading the
        Zenoh liveliness graph. rmw_zenoh advertises each service server (``SS``)
        with a token ending in ``.../<mangled_name>/<dds_type>/<RIHS01_hash>``.
        Returns the hash or None (cached per service_name)."""
        if service_name in self._service_hash_cache:
            return self._service_hash_cache[service_name]

        mangled = "%" + service_name.strip("/").replace("/", "%")
        found: Dict[str, Optional[str]] = {"hash": None}
        done = threading.Event()

        def on_token(sample):
            key = str(sample.key_expr)
            if "/SS/" in key and mangled in key:
                after = key.split(mangled, 1)[1]
                m = re.search(r"RIHS01_[0-9a-f]+", after)
                if m:
                    found["hash"] = m.group(0)
                    done.set()

        sub = None
        try:
            sub = self._session_mgr.session.liveliness().declare_subscriber(
                "@ros2_lv/**", on_token, history=True
            )
            done.wait(timeout=2.0)
        except Exception as e:
            print(f"[ZenohTransport] service-hash discovery failed for {service_name}: {e}")
        finally:
            if sub is not None:
                try:
                    sub.undeclare()
                except Exception:
                    pass

        self._service_hash_cache[service_name] = found["hash"]
        return found["hash"]

    def call_action(
        self,
        action_name: str,
        action_type: str,
        goal: Dict[str, Any],
        feedback_callback: Optional[Callable[[Dict[str, Any]], None]] = None,
        timeout: Optional[float] = None,
    ) -> Dict[str, Any]:
        """
        Call a ROS2 action.
        Note: The current zenoh_ros2_sdk does not explicitly expose a high-level ActionClient yet.
        """
        print(
            f"[ZenohTransport] WARNING: Action {action_name} call not fully supported in this SDK version."
        )
        return {"result": None, "status": "NOT_IMPLEMENTED"}

    def cancel_action(self) -> None:
        pass


class ZenohCamera(CameraTransportInterface):
    """
    Camera transport implementation using Zenoh SDK.
    Subscribes to standard 'sensor_msgs/msg/Image' (raw, uncompressed frames).
    """

    def __init__(
        self,
        host: str,
        port: int = 7447,
        topic: str = "walkie/camera/image",
        camera_name: str = "head",
        multi_camera: bool = False,
        **kwargs,
    ):
        if not ZENOH_AVAILABLE:
            raise ImportError("zenoh_ros2_sdk is not installed.")
        if cv2 is None:
            raise ImportError("opencv-python is required for ZenohCamera.")

        self._host = host
        self._port = port
        self._default_topic = topic
        self._camera_name = camera_name
        self._multi_camera = multi_camera

        self._session_mgr = None
        self._subscribers: Dict[str, ROS2Subscriber] = {}

        self._latest_frames: Dict[str, np.ndarray] = {}
        # Depth frames are kept separate from color: they share camera-name keys
        # but carry single-channel metric data (float32 metres / uint16 mm).
        self._latest_depth: Dict[str, np.ndarray] = {}
        self._frame_lock = threading.Lock()
        self._streaming = False

    @property
    def is_streaming(self) -> bool:
        return self._streaming

    @property
    def frame_shape(self) -> Optional[Tuple[int, int, int]]:
        with self._frame_lock:
            for frame in self._latest_frames.values():
                if frame is not None:
                    return frame.shape
        return None

    def connect(self) -> None:
        if self._streaming:
            return

        print(f"  → Connecting Camera to Zenoh at {self._host}:{self._port}...")
        # Reuse existing session singleton if available
        self._session_mgr = ZenohSession.get_instance(self._host, self._port)

        # Determine which topics to subscribe to
        topics = {}
        if self._multi_camera:
            topics = CAMERA_TOPICS
        else:
            # Use specific topic for requested camera
            if self._camera_name in CAMERA_TOPICS:
                topics[self._camera_name] = CAMERA_TOPICS[self._camera_name]
            else:
                topics[self._camera_name] = self._default_topic

        # Create subscribers for each camera topic
        for name, topic in topics.items():
            print(f"  → Subscribing to camera topic: {topic}")
            
            # Closure to capture camera name for the callback
            def make_cb(cam_id):
                return lambda msg: self._on_frame(cam_id, msg)

            self._subscribers[name] = ROS2Subscriber(
                topic=topic,
                msg_type="sensor_msgs/msg/Image",
                domain_id=ROS_DOMAIN_ID,
                callback=make_cb(name),
                router_ip=self._host,
                router_port=self._port,
            )

        # Depth streams. Subscribed under a "depth:" prefix so the callbacks /
        # handles never collide with the color subscribers above.
        depth_topics = DEPTH_TOPICS
        if not self._multi_camera:
            # Only the requested camera's depth, when it has a depth topic.
            depth_topics = {
                self._camera_name: DEPTH_TOPICS[self._camera_name]
            } if self._camera_name in DEPTH_TOPICS else {}

        for name, topic in depth_topics.items():
            print(f"  → Subscribing to depth topic: {topic}")

            def make_depth_cb(cam_id):
                return lambda msg: self._on_depth(cam_id, msg)

            self._subscribers[f"depth:{name}"] = ROS2Subscriber(
                topic=topic,
                msg_type="sensor_msgs/msg/Image",
                domain_id=ROS_DOMAIN_ID,
                callback=make_depth_cb(name),
                router_ip=self._host,
                router_port=self._port,
            )

        self._streaming = True
        print("  ✓ Camera Stream Started")

    def disconnect(self) -> None:
        self._streaming = False
        for sub in self._subscribers.values():
            sub.close()
        self._subscribers.clear()

        with self._frame_lock:
            self._latest_frames.clear()
            self._latest_depth.clear()

    def _on_frame(self, name: str, msg: Any) -> None:
        """Handle raw sensor_msgs/msg/Image message."""
        try:
            # Extract metadata
            height = getattr(msg, "height", None)
            width = getattr(msg, "width", None)
            encoding = getattr(msg, "encoding", "")
            data = getattr(msg, "data", None)

            if data is None or height is None or width is None:
                return

            # Handle different byte representations (rosbags vs standard)
            if hasattr(data, "tobytes"):
                data = data.tobytes()
            elif isinstance(data, list):
                data = bytes(data)
            elif not isinstance(data, (bytes, bytearray)):
                # Fallback
                data = bytes(data)

            # Convert raw bytes to flat numpy array
            np_arr = np.frombuffer(data, dtype=np.uint8)

            # Reshape based on image encoding
            if encoding == "rgb8":
                frame = np_arr.reshape((height, width, 3))
                frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
            elif encoding == "bgr8":
                frame = np_arr.reshape((height, width, 3))
            elif encoding == "bgra8":
                frame = np_arr.reshape((height, width, 4))
            elif encoding == "rgba8":
                frame = np_arr.reshape((height, width, 4))
                frame = cv2.cvtColor(frame, cv2.COLOR_RGBA2BGRA)
            elif encoding == "mono8":
                frame = np_arr.reshape((height, width))
            else:
                # Unsupported format
                print(f"Unsupported encoding '{encoding}' for camera '{name}'")
                return

            if frame is not None:
                with self._frame_lock:
                    self._latest_frames[name] = frame

        except Exception as e:
            # print(f"Frame decode error: {e}")
            pass

    def _on_depth(self, name: str, msg: Any) -> None:
        """Handle a raw sensor_msgs/msg/Image carrying single-channel depth.

        Stores the frame in its native units (no conversion):
          - 32FC1 / mono16-as-float -> float32, metres (NaN = invalid pixel)
          - 16UC1  / mono16          -> uint16, millimetres
        """
        try:
            height = getattr(msg, "height", None)
            width = getattr(msg, "width", None)
            encoding = getattr(msg, "encoding", "")
            data = getattr(msg, "data", None)

            if data is None or height is None or width is None:
                return

            # Normalize the byte representation (rosbags numpy vs list vs bytes)
            if hasattr(data, "tobytes"):
                data = data.tobytes()
            elif isinstance(data, list):
                data = bytes(data)
            elif not isinstance(data, (bytes, bytearray)):
                data = bytes(data)

            if encoding == "32FC1":
                depth = np.frombuffer(data, dtype=np.float32).reshape((height, width))
            elif encoding in ("16UC1", "mono16"):
                depth = np.frombuffer(data, dtype=np.uint16).reshape((height, width))
            else:
                print(f"Unsupported depth encoding '{encoding}' for camera '{name}'")
                return

            with self._frame_lock:
                self._latest_depth[name] = depth

        except Exception:
            # Swallow decode errors -- get_depth() simply returns the last good frame.
            pass

    def get_frame(self, camera_name: Optional[str] = None) -> Optional[np.ndarray]:
        target = camera_name or self._camera_name
        with self._frame_lock:
            frame = self._latest_frames.get(target)
            return frame.copy() if frame is not None else None

    def get_head_frame(self) -> Optional[np.ndarray]:
        return self.get_frame("head")

    def get_left_frame(self) -> Optional[np.ndarray]:
        return self.get_frame("left")

    def get_right_frame(self) -> Optional[np.ndarray]:
        return self.get_frame("right")

    def get_all_frames(self) -> Dict[str, np.ndarray]:
        with self._frame_lock:
            return {
                k: v.copy() for k, v in self._latest_frames.items() if v is not None
            }

    def get_depth(self, camera_name: Optional[str] = None) -> Optional[np.ndarray]:
        """Latest depth frame for a camera (None if depth disabled/unavailable)."""
        target = camera_name or self._camera_name
        with self._frame_lock:
            depth = self._latest_depth.get(target)
            return depth.copy() if depth is not None else None

    def get_all_depth(self) -> Dict[str, np.ndarray]:
        with self._frame_lock:
            return {
                k: v.copy() for k, v in self._latest_depth.items() if v is not None
            }


__all__ = ["ZenohTransport", "ZenohCamera"]