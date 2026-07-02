"""Unit tests for the Lidar module (offline).

Covers:
  - Lidar subscription setup (subscribes to the scan topic, correct type).
  - Namespacing of the scan topic.
  - get_scan / get_ranges delegation and copy semantics.
  - get_once happy path, timeout, and late arrival.
  - stop() unsubscribes.
  - repr formatting.
"""

import threading
import time
from typing import Any, Dict, List

from walkie_sdk.core.interfaces import ROSTransportInterface
from walkie_sdk.modules.lidar import Lidar


# ---------------------------------------------------------------------------
# Fake ROS transport
# ---------------------------------------------------------------------------

class FakeROSTransport(ROSTransportInterface):
    """Minimal in-memory transport that records subscribe/unsubscribe calls."""

    def __init__(self):
        self._connected = True
        self.subscriptions: List[Dict] = []
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
        for sub in self.subscriptions:
            if sub["topic"] == topic:
                sub["callback"](msg)


_SAMPLE_SCAN = {
    "header": {"frame_id": "laser", "stamp": {"sec": 1, "nanosec": 0}},
    "angle_min": -3.14,
    "angle_max": 3.14,
    "angle_increment": 0.01,
    "range_min": 0.1,
    "range_max": 10.0,
    "ranges": [1.0, 2.0, 3.0],
    "intensities": [],
}


def _make_lidar(transport=None, namespace=""):
    t = transport or FakeROSTransport()
    lidar = Lidar(t, namespace=namespace)
    lidar._setup_subscription()
    return lidar, t


# ---------------------------------------------------------------------------
# Subscription setup
# ---------------------------------------------------------------------------

def test_setup_subscription_subscribes_to_scan():
    lidar, t = _make_lidar()
    assert len(t.subscriptions) == 1
    assert t.subscriptions[0]["topic"] == lidar.scan_topic
    assert lidar.is_subscribed


def test_setup_subscription_uses_laserscan_type():
    lidar, t = _make_lidar()
    assert t.subscriptions[0]["message_type"] == "sensor_msgs/msg/LaserScan"


def test_setup_subscription_idempotent():
    lidar, t = _make_lidar()
    lidar._setup_subscription()
    assert len(t.subscriptions) == 1


def test_setup_subscription_skipped_when_not_connected():
    t = FakeROSTransport()
    t._connected = False
    lidar = Lidar(t)
    lidar._setup_subscription()
    assert len(t.subscriptions) == 0
    assert not lidar.is_subscribed


# ---------------------------------------------------------------------------
# Namespacing
# ---------------------------------------------------------------------------

def test_scan_topic_namespaced():
    lidar, t = _make_lidar(namespace="robot1")
    assert lidar.scan_topic == "robot1/scan"
    assert t.subscriptions[0]["topic"] == "robot1/scan"


def test_scan_topic_no_namespace():
    lidar, _ = _make_lidar()
    assert lidar.scan_topic == "scan"


# ---------------------------------------------------------------------------
# get_scan / get_ranges
# ---------------------------------------------------------------------------

def test_get_scan_none_before_first_message():
    lidar, _ = _make_lidar()
    assert lidar.get_scan() is None
    assert lidar.get_ranges() is None


def test_get_scan_returns_message_after_callback():
    lidar, t = _make_lidar()
    t.fire(lidar.scan_topic, _SAMPLE_SCAN)
    out = lidar.get_scan()
    assert out is not None
    assert out["range_max"] == 10.0
    assert lidar.get_ranges() == [1.0, 2.0, 3.0]


def test_get_scan_returns_copy():
    lidar, t = _make_lidar()
    t.fire(lidar.scan_topic, _SAMPLE_SCAN)
    assert lidar.get_scan() is not lidar.get_scan()


# ---------------------------------------------------------------------------
# get_once
# ---------------------------------------------------------------------------

def test_get_once_returns_immediately_if_already_cached():
    lidar, t = _make_lidar()
    t.fire(lidar.scan_topic, _SAMPLE_SCAN)
    out = lidar.get_once(timeout=1.0)
    assert out is not None
    assert out["range_max"] == 10.0


def test_get_once_returns_none_on_timeout():
    lidar, _ = _make_lidar()
    start = time.monotonic()
    assert lidar.get_once(timeout=0.2) is None
    assert time.monotonic() - start >= 0.2


def test_get_once_waits_for_late_arrival():
    lidar, t = _make_lidar()

    def _deliver():
        time.sleep(0.15)
        t.fire(lidar.scan_topic, _SAMPLE_SCAN)

    threading.Thread(target=_deliver, daemon=True).start()
    assert lidar.get_once(timeout=1.0) is not None


# ---------------------------------------------------------------------------
# stop / repr
# ---------------------------------------------------------------------------

def test_stop_unsubscribes():
    lidar, t = _make_lidar()
    handle = t.subscriptions[0]["handle"]
    lidar.stop()
    assert not lidar.is_subscribed
    assert handle in t.unsubscribed


def test_repr_stopped():
    lidar = Lidar(FakeROSTransport())
    assert "stopped" in repr(lidar)


def test_repr_subscribed():
    lidar, _ = _make_lidar()
    assert "subscribed" in repr(lidar)
