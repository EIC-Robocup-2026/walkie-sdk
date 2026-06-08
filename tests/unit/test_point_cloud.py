"""Unit tests for the PointCloud module (offline).

Covers:
  - PointCloud subscription setup (calls transport.subscribe for each source).
  - get_cloud / get_all_clouds delegation.
  - get_once happy path and timeout.
  - _make_cb closure correctly keyed by source name.
  - stop() calls unsubscribe for all handles.
  - repr formatting.
"""

import threading
import time
from typing import Any, Callable, Dict, List, Optional

import pytest

from walkie_sdk.core.interfaces import ROSTransportInterface
from walkie_sdk.modules.point_cloud import PointCloud


# ---------------------------------------------------------------------------
# Fake ROS transport
# ---------------------------------------------------------------------------

class FakeROSTransport(ROSTransportInterface):
    """Minimal in-memory transport that records subscribe/unsubscribe calls."""

    def __init__(self):
        self._connected = True
        self.subscriptions: List[Dict] = []   # [{topic, message_type, callback, handle}]
        self.unsubscribed: List[Any] = []

    # -- required abstract surface --
    def connect(self): pass
    def disconnect(self): pass
    @property
    def is_connected(self): return self._connected
    def publish(self, topic, message_type, message): pass
    def call_action(self, *a, **kw): return {}
    def cancel_action(self): pass
    def call_service(self, *a, **kw): return {}

    def subscribe(self, topic, message_type, callback,
                  throttle_rate=0, queue_size=1):
        handle = object()
        self.subscriptions.append({
            "topic": topic, "message_type": message_type,
            "callback": callback, "handle": handle,
        })
        return handle

    def unsubscribe(self, handle):
        self.unsubscribed.append(handle)

    def fire(self, topic: str, msg: Dict) -> None:
        """Trigger the callback registered for *topic* with *msg*."""
        for sub in self.subscriptions:
            if sub["topic"] == topic:
                sub["callback"](msg)


_SAMPLE_CLOUD = {
    "header": {"frame_id": "map", "stamp": {"sec": 1, "nanosec": 0}},
    "height": 1,
    "width": 100,
    "fields": [{"name": "x"}, {"name": "y"}, {"name": "z"}],
    "is_bigendian": False,
    "point_step": 16,
    "row_step": 1600,
    "data": b"\x00" * 1600,
    "is_dense": False,
}


def _make_pc(transport=None):
    t = transport or FakeROSTransport()
    pc = PointCloud(t)
    pc._setup_subscription()
    return pc, t


# ---------------------------------------------------------------------------
# Subscription setup
# ---------------------------------------------------------------------------

def test_setup_subscription_subscribes_to_all_sources():
    from walkie_sdk.config.ros_topics import POINT_CLOUD_TOPICS
    pc, t = _make_pc()
    subscribed_topics = {s["topic"] for s in t.subscriptions}
    for topic in POINT_CLOUD_TOPICS.values():
        assert topic in subscribed_topics


def test_setup_subscription_uses_pointcloud2_type():
    pc, t = _make_pc()
    for sub in t.subscriptions:
        assert sub["message_type"] == "sensor_msgs/msg/PointCloud2"


def test_setup_subscription_idempotent():
    pc, t = _make_pc()
    pc._setup_subscription()
    assert len(t.subscriptions) == len(pc.source_names)


def test_setup_subscription_skipped_when_not_connected():
    t = FakeROSTransport()
    t._connected = False
    pc = PointCloud(t)
    pc._setup_subscription()
    assert len(t.subscriptions) == 0
    assert not pc.is_subscribed


# ---------------------------------------------------------------------------
# get_cloud / get_all_clouds
# ---------------------------------------------------------------------------

def test_get_cloud_none_before_first_message():
    pc, _ = _make_pc()
    assert pc.get_cloud("head") is None


def test_get_cloud_returns_message_after_callback():
    from walkie_sdk.config.ros_topics import POINT_CLOUD_TOPICS
    pc, t = _make_pc()
    t.fire(POINT_CLOUD_TOPICS["head"], _SAMPLE_CLOUD)
    out = pc.get_cloud("head")
    assert out is not None
    assert out["width"] == 100


def test_get_cloud_returns_copy():
    from walkie_sdk.config.ros_topics import POINT_CLOUD_TOPICS
    pc, t = _make_pc()
    t.fire(POINT_CLOUD_TOPICS["head"], _SAMPLE_CLOUD)
    a = pc.get_cloud("head")
    b = pc.get_cloud("head")
    assert a is not b


def test_get_cloud_none_for_unknown_source():
    pc, _ = _make_pc()
    assert pc.get_cloud("unknown") is None


def test_get_all_clouds_empty_before_any_message():
    pc, _ = _make_pc()
    assert pc.get_all_clouds() == {}


def test_get_all_clouds_returns_received_sources():
    from walkie_sdk.config.ros_topics import POINT_CLOUD_TOPICS
    pc, t = _make_pc()
    t.fire(POINT_CLOUD_TOPICS["head"], _SAMPLE_CLOUD)
    all_c = pc.get_all_clouds()
    assert "head" in all_c
    assert all_c["head"]["width"] == 100


# ---------------------------------------------------------------------------
# get_once
# ---------------------------------------------------------------------------

def test_get_once_returns_immediately_if_already_cached():
    from walkie_sdk.config.ros_topics import POINT_CLOUD_TOPICS
    pc, t = _make_pc()
    t.fire(POINT_CLOUD_TOPICS["head"], _SAMPLE_CLOUD)
    out = pc.get_once("head", timeout=1.0)
    assert out is not None
    assert out["width"] == 100


def test_get_once_returns_none_on_timeout():
    pc, _ = _make_pc()
    start = time.monotonic()
    out = pc.get_once("head", timeout=0.2)
    assert out is None
    assert time.monotonic() - start >= 0.2


def test_get_once_waits_for_late_arrival():
    from walkie_sdk.config.ros_topics import POINT_CLOUD_TOPICS
    pc, t = _make_pc()

    def _deliver():
        time.sleep(0.15)
        t.fire(POINT_CLOUD_TOPICS["head"], _SAMPLE_CLOUD)

    threading.Thread(target=_deliver, daemon=True).start()
    out = pc.get_once("head", timeout=1.0)
    assert out is not None


def test_get_once_default_source_is_head():
    from walkie_sdk.config.ros_topics import POINT_CLOUD_TOPICS
    pc, t = _make_pc()
    t.fire(POINT_CLOUD_TOPICS["head"], _SAMPLE_CLOUD)
    assert pc.get_once(timeout=1.0) is not None


# ---------------------------------------------------------------------------
# stop / unsubscribe
# ---------------------------------------------------------------------------

def test_stop_unsubscribes_all_handles():
    pc, t = _make_pc()
    expected_handles = [s["handle"] for s in t.subscriptions]
    pc.stop()
    assert not pc.is_subscribed
    for h in expected_handles:
        assert h in t.unsubscribed


# ---------------------------------------------------------------------------
# source_names / repr
# ---------------------------------------------------------------------------

def test_source_names_matches_config():
    from walkie_sdk.config.ros_topics import POINT_CLOUD_TOPICS
    pc, _ = _make_pc()
    assert set(pc.source_names) == set(POINT_CLOUD_TOPICS.keys())


def test_repr_stopped():
    t = FakeROSTransport()
    pc = PointCloud(t)
    r = repr(pc)
    assert "stopped" in r


def test_repr_subscribed():
    pc, _ = _make_pc()
    assert "subscribed" in repr(pc)
