"""Unit tests for camera intrinsics (CameraInfo) support (offline).

Covers:
  - Camera.get_camera_info() one-shot fetch, caching, and unsubscribe.
  - Graceful None on missing transport, unknown camera, subscribe failure,
    and timeout.
  - Camera.get_intrinsics() extraction of fx/fy/cx/cy from the 'k' matrix.
"""

from walkie_sdk.config.ros_topics import CAMERA_INFO_TOPICS
from walkie_sdk.modules.camera import Camera

CAMERA_INFO_MSG = {
    "header": {"frame_id": "zed_head_left_camera_frame_optical"},
    "width": 1280,
    "height": 720,
    "distortion_model": "plumb_bob",
    "d": [0.0, 0.0, 0.0, 0.0, 0.0],
    "k": [500.0, 0.0, 640.0, 0.0, 510.0, 360.0, 0.0, 0.0, 1.0],
}


class FakeROSTransport:
    """Replays a canned CameraInfo message immediately on subscribe."""

    def __init__(self, message=CAMERA_INFO_MSG, fail_subscribe=False):
        self._message = message
        self._fail = fail_subscribe
        self.subscriptions = []
        self.unsubscribed = []

    def subscribe(self, topic, message_type, callback, **kwargs):
        if self._fail:
            raise ConnectionError("not connected")
        self.subscriptions.append((topic, message_type))
        if self._message is not None:
            callback(dict(self._message))
        return f"handle:{topic}"

    def unsubscribe(self, handle):
        self.unsubscribed.append(handle)


def _make_camera(**kwargs):
    # The camera transport is unused by the info path; only the ROS transport
    # matters here.
    return Camera(transport=None, **kwargs)


def test_get_camera_info_fetches_and_unsubscribes():
    ros = FakeROSTransport()
    cam = _make_camera(ros_transport=ros)

    info = cam.get_camera_info(timeout=1.0)
    assert info is not None
    assert info["width"] == 1280 and info["height"] == 720
    assert info["k"][0] == 500.0

    assert ros.subscriptions == [
        (CAMERA_INFO_TOPICS["head"], "sensor_msgs/msg/CameraInfo")
    ]
    assert len(ros.unsubscribed) == 1


def test_get_camera_info_is_cached():
    ros = FakeROSTransport()
    cam = _make_camera(ros_transport=ros)

    first = cam.get_camera_info(timeout=1.0)
    second = cam.get_camera_info(timeout=1.0)
    assert first is second
    # Cached call does not subscribe again.
    assert len(ros.subscriptions) == 1


def test_get_camera_info_none_without_ros_transport():
    assert _make_camera().get_camera_info(timeout=0.1) is None


def test_get_camera_info_none_for_unknown_camera():
    ros = FakeROSTransport()
    cam = _make_camera(ros_transport=ros, camera_name="no_such_camera")
    assert cam.get_camera_info(timeout=0.1) is None
    assert ros.subscriptions == []


def test_get_camera_info_none_on_subscribe_failure():
    cam = _make_camera(ros_transport=FakeROSTransport(fail_subscribe=True))
    assert cam.get_camera_info(timeout=0.1) is None


def test_get_camera_info_none_on_timeout_and_not_cached():
    ros = FakeROSTransport(message=None)  # never delivers a message
    cam = _make_camera(ros_transport=ros)

    assert cam.get_camera_info(timeout=0.05) is None
    # Timed-out subscription is cleaned up and the failure is not cached.
    assert len(ros.unsubscribed) == 1
    assert cam.get_camera_info(timeout=0.05) is None
    assert len(ros.subscriptions) == 2


def test_get_intrinsics_extracts_pinhole_parameters():
    cam = _make_camera(ros_transport=FakeROSTransport())

    intr = cam.get_intrinsics(timeout=1.0)
    assert intr == {
        "fx": 500.0,
        "fy": 510.0,
        "cx": 640.0,
        "cy": 360.0,
        "width": 1280,
        "height": 720,
    }


def test_get_intrinsics_none_when_info_unavailable():
    assert _make_camera().get_intrinsics(timeout=0.1) is None


def test_get_intrinsics_none_on_malformed_k():
    bad = dict(CAMERA_INFO_MSG, k=[1.0, 2.0])
    cam = _make_camera(ros_transport=FakeROSTransport(message=bad))
    assert cam.get_intrinsics(timeout=1.0) is None
