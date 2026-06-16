"""Unit tests for walkie_sdk.modules.telemetry.Telemetry.get_position().

`get_position()` is the robot's "current position" accessor (bot.status.get_position()).
It returns the cached pose extracted from the latest odometry message, or None
before any odometry has arrived.
"""

import math
from unittest.mock import MagicMock
import pytest

from walkie_sdk.modules.telemetry import Telemetry
from walkie_sdk.utils.converters import euler_to_quaternion
from walkie_sdk.config.ros_topics import TELEMETRY_TOPICS


def _make_telemetry(namespace: str = "") -> tuple[Telemetry, MagicMock]:
    transport = MagicMock()
    transport.is_connected = True
    return Telemetry(transport=transport, namespace=namespace), transport


def _odom_msg(x: float, y: float, heading: float) -> dict:
    """Build a ROS Odometry-shaped dict with the given pose."""
    qx, qy, qz, qw = euler_to_quaternion(0.0, 0.0, heading)
    return {
        "pose": {
            "pose": {
                "position": {"x": x, "y": y, "z": 0.0},
                "orientation": {"x": qx, "y": qy, "z": qz, "w": qw},
            }
        },
        "twist": {
            "twist": {
                "linear": {"x": 0.0, "y": 0.0, "z": 0.0},
                "angular": {"x": 0.0, "y": 0.0, "z": 0.0},
            }
        },
    }


# ---------------------------------------------------------------------------
# No data yet
# ---------------------------------------------------------------------------


class TestGetPositionNoData:
    def test_returns_none_before_any_odom(self):
        tel, _ = _make_telemetry()
        assert tel.get_position() is None

    def test_has_data_false_before_any_odom(self):
        tel, _ = _make_telemetry()
        assert tel.has_data is False


# ---------------------------------------------------------------------------
# Position extraction from odometry
# ---------------------------------------------------------------------------


class TestGetPositionFromOdom:
    def test_returns_x_and_y(self):
        tel, _ = _make_telemetry()
        tel._on_odom(_odom_msg(1.2, 3.5, 0.0))
        pos = tel.get_position()
        assert pos["x"] == pytest.approx(1.2)
        assert pos["y"] == pytest.approx(3.5)

    def test_zero_heading_yaw_is_zero(self):
        tel, _ = _make_telemetry()
        tel._on_odom(_odom_msg(0.0, 0.0, 0.0))
        assert tel.get_position()["heading"] == pytest.approx(0.0, abs=1e-9)

    def test_heading_decoded_from_quaternion(self):
        tel, _ = _make_telemetry()
        tel._on_odom(_odom_msg(0.0, 0.0, math.pi / 2))
        assert tel.get_position()["heading"] == pytest.approx(math.pi / 2)

    def test_negative_heading_decoded(self):
        tel, _ = _make_telemetry()
        tel._on_odom(_odom_msg(0.0, 0.0, -math.pi / 4))
        assert tel.get_position()["heading"] == pytest.approx(-math.pi / 4)

    def test_negative_coordinates(self):
        tel, _ = _make_telemetry()
        tel._on_odom(_odom_msg(-2.0, -7.25, 0.0))
        pos = tel.get_position()
        assert pos["x"] == pytest.approx(-2.0)
        assert pos["y"] == pytest.approx(-7.25)

    def test_keys_are_exactly_x_y_heading(self):
        tel, _ = _make_telemetry()
        tel._on_odom(_odom_msg(1.0, 2.0, 0.5))
        assert set(tel.get_position().keys()) == {"x", "y", "heading"}

    def test_has_data_true_after_odom(self):
        tel, _ = _make_telemetry()
        tel._on_odom(_odom_msg(1.0, 2.0, 0.0))
        assert tel.has_data is True

    def test_latest_message_wins(self):
        tel, _ = _make_telemetry()
        tel._on_odom(_odom_msg(1.0, 1.0, 0.0))
        tel._on_odom(_odom_msg(9.0, 8.0, 0.0))
        pos = tel.get_position()
        assert pos["x"] == pytest.approx(9.0)
        assert pos["y"] == pytest.approx(8.0)


# ---------------------------------------------------------------------------
# Returned dict is an isolated copy
# ---------------------------------------------------------------------------


class TestGetPositionReturnsCopy:
    def test_mutating_result_does_not_affect_cache(self):
        tel, _ = _make_telemetry()
        tel._on_odom(_odom_msg(1.0, 2.0, 0.0))
        pos = tel.get_position()
        pos["x"] = 999.0
        assert tel.get_position()["x"] == pytest.approx(1.0)

    def test_successive_calls_return_distinct_objects(self):
        tel, _ = _make_telemetry()
        tel._on_odom(_odom_msg(1.0, 2.0, 0.0))
        assert tel.get_position() is not tel.get_position()


# ---------------------------------------------------------------------------
# Malformed messages are ignored (degrade gracefully)
# ---------------------------------------------------------------------------


class TestGetPositionMalformed:
    def test_missing_pose_keeps_position_none(self):
        tel, _ = _make_telemetry()
        tel._on_odom({"twist": {"twist": {"linear": {"x": 0.0}, "angular": {"z": 0.0}}}})
        assert tel.get_position() is None

    def test_malformed_after_valid_keeps_last_good(self):
        tel, _ = _make_telemetry()
        tel._on_odom(_odom_msg(4.0, 5.0, 0.0))
        tel._on_odom({})  # malformed — must not clobber cached pose
        pos = tel.get_position()
        assert pos["x"] == pytest.approx(4.0)
        assert pos["y"] == pytest.approx(5.0)


# ---------------------------------------------------------------------------
# Subscription wiring (start) and namespacing
# ---------------------------------------------------------------------------


class TestTelemetrySubscription:
    def test_start_subscribes_to_odom_topic(self):
        tel, transport = _make_telemetry()
        tel.start()
        assert transport.subscribe.call_args[1]["topic"] == TELEMETRY_TOPICS["odom"]
        assert transport.subscribe.call_args[1]["message_type"] == TELEMETRY_TOPICS["odom_type"]

    def test_start_noop_when_not_connected(self):
        tel, transport = _make_telemetry()
        transport.is_connected = False
        tel.start()
        transport.subscribe.assert_not_called()

    def test_start_is_idempotent(self):
        tel, transport = _make_telemetry()
        tel.start()
        tel.start()
        assert transport.subscribe.call_count == 1

    def test_odom_topic_namespaced(self):
        tel, _ = _make_telemetry(namespace="robot1")
        assert tel.odom_topic == f"robot1/{TELEMETRY_TOPICS['odom']}"

    def test_start_subscribes_to_namespaced_topic(self):
        tel, transport = _make_telemetry(namespace="robot1")
        tel.start()
        assert transport.subscribe.call_args[1]["topic"] == f"robot1/{TELEMETRY_TOPICS['odom']}"
