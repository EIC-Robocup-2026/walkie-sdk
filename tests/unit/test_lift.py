"""Unit tests for walkie_sdk.modules.lift.Lift."""

import threading
import time
from unittest.mock import MagicMock
import pytest

from walkie_sdk.modules.lift import Lift, LIFT_MAX_CM, LIFT_DEFAULT_SPEED, LIFT_DEFAULT_ACCEL
from walkie_sdk.config.ros_topics import LIFT_TOPICS


def _approx(value: float, rel: float = 1e-6) -> pytest.approx:
    return pytest.approx(value, rel=rel)


def _make_lift(namespace: str = "") -> tuple[Lift, MagicMock, MagicMock]:
    transport = MagicMock()
    hub = MagicMock()
    hub.get.return_value = None  # no data by default
    return Lift(transport=transport, namespace=namespace, joint_state_hub=hub), transport, hub


def _inject_position_m(lift: Lift, pos_m: float) -> None:
    """Simulate the hub reporting a position in meters."""
    lift._joint_state_hub.get.return_value = pos_m


# ---------------------------------------------------------------------------
# set() — payload correctness (non-blocking to avoid waiting)
# ---------------------------------------------------------------------------


class TestLiftSetNormalized:
    def test_midpoint_sends_half_max_cm(self):
        lift, transport, _ = _make_lift()
        lift.set(0.5, blocking=False)
        msg = transport.publish.call_args[0][2]
        assert msg["data"][0] == _approx(LIFT_MAX_CM * 0.5)

    def test_bottom_sends_zero(self):
        lift, transport, _ = _make_lift()
        lift.set(0.0, blocking=False)
        msg = transport.publish.call_args[0][2]
        assert msg["data"][0] == _approx(0.0)

    def test_top_sends_max_cm(self):
        lift, transport, _ = _make_lift()
        lift.set(1.0, blocking=False)
        msg = transport.publish.call_args[0][2]
        assert msg["data"][0] == _approx(LIFT_MAX_CM)

    def test_over_one_is_clamped_to_max(self):
        lift, transport, _ = _make_lift()
        lift.set(1.5, blocking=False)
        msg = transport.publish.call_args[0][2]
        assert msg["data"][0] == _approx(LIFT_MAX_CM)

    def test_negative_is_clamped_to_zero(self):
        lift, transport, _ = _make_lift()
        lift.set(-0.5, blocking=False)
        msg = transport.publish.call_args[0][2]
        assert msg["data"][0] == _approx(0.0)

    def test_default_speed_and_accel(self):
        lift, transport, _ = _make_lift()
        lift.set(0.5, blocking=False)
        msg = transport.publish.call_args[0][2]
        assert msg["data"][1] == _approx(LIFT_DEFAULT_SPEED)
        assert msg["data"][2] == _approx(LIFT_DEFAULT_ACCEL)

    def test_custom_speed_and_accel(self):
        lift, transport, _ = _make_lift()
        lift.set(0.5, speed=5.0, accel=3.0, blocking=False)
        msg = transport.publish.call_args[0][2]
        assert msg["data"][1] == _approx(5.0)
        assert msg["data"][2] == _approx(3.0)

    def test_publishes_to_correct_topic(self):
        lift, transport, _ = _make_lift()
        lift.set(0.5, blocking=False)
        topic = transport.publish.call_args[0][0]
        assert topic == LIFT_TOPICS["cmd"]

    def test_publishes_correct_message_type(self):
        lift, transport, _ = _make_lift()
        lift.set(0.5, blocking=False)
        msg_type = transport.publish.call_args[0][1]
        assert msg_type == LIFT_TOPICS["cmd_type"]


# ---------------------------------------------------------------------------
# set() — real-position mode
# ---------------------------------------------------------------------------


class TestLiftSetRealPos:
    def test_sends_raw_cm(self):
        lift, transport, _ = _make_lift()
        lift.set(37.0, norm_pos=False, blocking=False)
        msg = transport.publish.call_args[0][2]
        assert msg["data"][0] == _approx(37.0)

    def test_zero_cm(self):
        lift, transport, _ = _make_lift()
        lift.set(0.0, norm_pos=False, blocking=False)
        msg = transport.publish.call_args[0][2]
        assert msg["data"][0] == _approx(0.0)

    def test_max_cm(self):
        lift, transport, _ = _make_lift()
        lift.set(LIFT_MAX_CM, norm_pos=False, blocking=False)
        msg = transport.publish.call_args[0][2]
        assert msg["data"][0] == _approx(LIFT_MAX_CM)

    def test_over_max_is_clamped(self):
        lift, transport, _ = _make_lift()
        lift.set(100.0, norm_pos=False, blocking=False)
        msg = transport.publish.call_args[0][2]
        assert msg["data"][0] == _approx(LIFT_MAX_CM)

    def test_negative_is_clamped_to_zero(self):
        lift, transport, _ = _make_lift()
        lift.set(-10.0, norm_pos=False, blocking=False)
        msg = transport.publish.call_args[0][2]
        assert msg["data"][0] == _approx(0.0)


# ---------------------------------------------------------------------------
# Blocking behaviour
# ---------------------------------------------------------------------------


class TestLiftBlocking:
    def test_non_blocking_returns_in_progress(self):
        lift, _, _ = _make_lift()
        result = lift.set(0.5, blocking=False)
        assert result == "IN_PROGRESS"

    def test_blocking_returns_succeeded_when_position_reached(self):
        lift, _, _ = _make_lift()
        _inject_position_m(lift, LIFT_MAX_CM / 2 / 100)  # already at midpoint

        result = lift.set(0.5, blocking=True, timeout=2.0, tolerance=0.02)
        assert result == "SUCCEEDED"

    def test_blocking_returns_timeout_when_no_feedback(self):
        lift, _, _ = _make_lift()
        # hub returns None — get() never has data

        result = lift.set(0.5, blocking=True, timeout=0.1, tolerance=0.02)
        assert result == "TIMEOUT"

    def test_blocking_returns_timeout_when_position_never_close_enough(self):
        lift, _, _ = _make_lift()
        _inject_position_m(lift, 0.0)  # stuck at bottom

        result = lift.set(1.0, blocking=True, timeout=0.1, tolerance=0.02)
        assert result == "TIMEOUT"

    def test_blocking_resolves_when_position_arrives_mid_wait(self):
        """Position arrives 0.15 s after command — should still SUCCEED."""
        lift, _, _ = _make_lift()
        target_m = LIFT_MAX_CM / 2 / 100

        def _deliver():
            time.sleep(0.15)
            _inject_position_m(lift, target_m)

        threading.Thread(target=_deliver, daemon=True).start()
        result = lift.set(0.5, blocking=True, timeout=2.0, tolerance=0.02)
        assert result == "SUCCEEDED"

    def test_non_blocking_status_eventually_updates(self):
        """Non-blocking: status transitions to SUCCEEDED once position arrives."""
        lift, _, _ = _make_lift()
        target_m = LIFT_MAX_CM / 2 / 100

        def _deliver():
            time.sleep(0.1)
            _inject_position_m(lift, target_m)

        threading.Thread(target=_deliver, daemon=True).start()
        lift.set(0.5, blocking=False, timeout=2.0, tolerance=0.02)
        assert lift.status == "IN_PROGRESS"

        deadline = time.time() + 2.0
        while time.time() < deadline:
            if lift.status == "SUCCEEDED":
                break
            time.sleep(0.05)
        assert lift.status == "SUCCEEDED"

    def test_status_property_reflects_last_result(self):
        lift, _, _ = _make_lift()
        _inject_position_m(lift, LIFT_MAX_CM / 2 / 100)

        lift.set(0.5, blocking=True, timeout=2.0)
        assert lift.status == "SUCCEEDED"

    def test_is_moving_true_while_non_blocking(self):
        lift, _, _ = _make_lift()
        lift.set(1.0, blocking=False, timeout=5.0)
        assert lift.is_moving is True


# ---------------------------------------------------------------------------
# get() — position read from hub
# ---------------------------------------------------------------------------


class TestLiftGet:
    def test_returns_none_before_any_data(self):
        lift, _, _ = _make_lift()
        assert lift.get() is None
        assert lift.get(norm_pos=False) is None

    def test_get_normalized_after_data(self):
        lift, _, _ = _make_lift()
        _inject_position_m(lift, LIFT_MAX_CM / 2 / 100)
        assert lift.get(norm_pos=True) == _approx(0.5)

    def test_get_real_pos_after_data(self):
        lift, _, _ = _make_lift()
        _inject_position_m(lift, LIFT_MAX_CM / 2 / 100)
        assert lift.get(norm_pos=False) == _approx(LIFT_MAX_CM / 2)

    def test_get_bottom_normalized(self):
        lift, _, _ = _make_lift()
        _inject_position_m(lift, 0.0)
        assert lift.get() == _approx(0.0)

    def test_get_top_normalized(self):
        lift, _, _ = _make_lift()
        _inject_position_m(lift, LIFT_MAX_CM / 100.0)
        assert lift.get() == _approx(1.0)

    def test_get_default_is_normalized(self):
        lift, _, _ = _make_lift()
        _inject_position_m(lift, 0.0)
        assert lift.get() == lift.get(norm_pos=True)


# ---------------------------------------------------------------------------
# Topics and namespace
# ---------------------------------------------------------------------------


class TestLiftTopics:
    def test_publishes_to_namespaced_topic(self):
        lift, transport, _ = _make_lift(namespace="robot1")
        lift.set(0.5, blocking=False)
        pub_topic = transport.publish.call_args[0][0]
        assert pub_topic == f"robot1/{LIFT_TOPICS['cmd']}"

    def test_namespace_setter_updates_publish_topic(self):
        lift, transport, _ = _make_lift()
        lift.namespace = "robot2"
        lift.set(0.5, blocking=False)
        pub_topic = transport.publish.call_args[0][0]
        assert pub_topic == f"robot2/{LIFT_TOPICS['cmd']}"
