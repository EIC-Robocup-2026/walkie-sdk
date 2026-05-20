"""Unit tests for walkie_sdk.modules.lift.Lift."""

import threading
import time
from unittest.mock import MagicMock
import pytest

from walkie_sdk.modules.lift import Lift, LIFT_MAX_CM, LIFT_DEFAULT_SPEED, LIFT_DEFAULT_ACCEL
from walkie_sdk.config.ros_topics import LIFT_TOPICS


def _approx(value: float, rel: float = 1e-6) -> pytest.approx:
    return pytest.approx(value, rel=rel)


def _make_lift(namespace: str = "") -> tuple[Lift, MagicMock]:
    transport = MagicMock()
    return Lift(transport=transport, namespace=namespace), transport


def _inject_position_m(lift: Lift, pos_m: float) -> None:
    """Simulate receiving a joint state message at pos_m meters."""
    lift._setup_state_subscription()
    callback = lift._transport.subscribe.call_args[0][2]
    callback({"name": ["lift_joint"], "position": [pos_m], "velocity": [0.0], "effort": [0.0]})


# ---------------------------------------------------------------------------
# set() — payload correctness (non-blocking to avoid waiting)
# ---------------------------------------------------------------------------


class TestLiftSetNormalized:
    def test_midpoint_sends_half_max_cm(self):
        lift, transport = _make_lift()
        lift.set(0.5, blocking=False)
        msg = transport.publish.call_args[0][2]
        assert msg["data"][0] == _approx(LIFT_MAX_CM * 0.5)

    def test_bottom_sends_zero(self):
        lift, transport = _make_lift()
        lift.set(0.0, blocking=False)
        msg = transport.publish.call_args[0][2]
        assert msg["data"][0] == _approx(0.0)

    def test_top_sends_max_cm(self):
        lift, transport = _make_lift()
        lift.set(1.0, blocking=False)
        msg = transport.publish.call_args[0][2]
        assert msg["data"][0] == _approx(LIFT_MAX_CM)

    def test_over_one_is_clamped_to_max(self):
        lift, transport = _make_lift()
        lift.set(1.5, blocking=False)
        msg = transport.publish.call_args[0][2]
        assert msg["data"][0] == _approx(LIFT_MAX_CM)

    def test_negative_is_clamped_to_zero(self):
        lift, transport = _make_lift()
        lift.set(-0.5, blocking=False)
        msg = transport.publish.call_args[0][2]
        assert msg["data"][0] == _approx(0.0)

    def test_default_speed_and_accel(self):
        lift, transport = _make_lift()
        lift.set(0.5, blocking=False)
        msg = transport.publish.call_args[0][2]
        assert msg["data"][1] == _approx(LIFT_DEFAULT_SPEED)
        assert msg["data"][2] == _approx(LIFT_DEFAULT_ACCEL)

    def test_custom_speed_and_accel(self):
        lift, transport = _make_lift()
        lift.set(0.5, speed=5.0, accel=3.0, blocking=False)
        msg = transport.publish.call_args[0][2]
        assert msg["data"][1] == _approx(5.0)
        assert msg["data"][2] == _approx(3.0)

    def test_publishes_to_correct_topic(self):
        lift, transport = _make_lift()
        lift.set(0.5, blocking=False)
        topic = transport.publish.call_args[0][0]
        assert topic == LIFT_TOPICS["cmd"]

    def test_publishes_correct_message_type(self):
        lift, transport = _make_lift()
        lift.set(0.5, blocking=False)
        msg_type = transport.publish.call_args[0][1]
        assert msg_type == LIFT_TOPICS["cmd_type"]


# ---------------------------------------------------------------------------
# set() — real-position mode
# ---------------------------------------------------------------------------


class TestLiftSetRealPos:
    def test_sends_raw_cm(self):
        lift, transport = _make_lift()
        lift.set(37.0, norm_pos=False, blocking=False)
        msg = transport.publish.call_args[0][2]
        assert msg["data"][0] == _approx(37.0)

    def test_zero_cm(self):
        lift, transport = _make_lift()
        lift.set(0.0, norm_pos=False, blocking=False)
        msg = transport.publish.call_args[0][2]
        assert msg["data"][0] == _approx(0.0)

    def test_max_cm(self):
        lift, transport = _make_lift()
        lift.set(LIFT_MAX_CM, norm_pos=False, blocking=False)
        msg = transport.publish.call_args[0][2]
        assert msg["data"][0] == _approx(LIFT_MAX_CM)

    def test_over_max_is_clamped(self):
        lift, transport = _make_lift()
        lift.set(100.0, norm_pos=False, blocking=False)
        msg = transport.publish.call_args[0][2]
        assert msg["data"][0] == _approx(LIFT_MAX_CM)

    def test_negative_is_clamped_to_zero(self):
        lift, transport = _make_lift()
        lift.set(-10.0, norm_pos=False, blocking=False)
        msg = transport.publish.call_args[0][2]
        assert msg["data"][0] == _approx(0.0)


# ---------------------------------------------------------------------------
# Blocking behaviour
# ---------------------------------------------------------------------------


class TestLiftBlocking:
    def test_non_blocking_returns_in_progress(self):
        lift, _ = _make_lift()
        result = lift.set(0.5, blocking=False)
        assert result == "IN_PROGRESS"

    def test_blocking_returns_succeeded_when_position_reached(self):
        lift, _ = _make_lift()
        _inject_position_m(lift, LIFT_MAX_CM / 2 / 100)  # already at midpoint

        result = lift.set(0.5, blocking=True, timeout=2.0, tolerance=0.02)
        assert result == "SUCCEEDED"

    def test_blocking_returns_timeout_when_no_feedback(self):
        lift, _ = _make_lift()
        # No position data injected — get() returns None forever

        result = lift.set(0.5, blocking=True, timeout=0.1, tolerance=0.02)
        assert result == "TIMEOUT"

    def test_blocking_returns_timeout_when_position_never_close_enough(self):
        lift, _ = _make_lift()
        _inject_position_m(lift, 0.0)  # stuck at bottom

        result = lift.set(1.0, blocking=True, timeout=0.1, tolerance=0.02)
        assert result == "TIMEOUT"

    def test_blocking_resolves_when_position_arrives_mid_wait(self):
        """Position arrives 0.15s after command — should still SUCCEED."""
        lift, _ = _make_lift()
        target_m = LIFT_MAX_CM / 2 / 100

        def _deliver():
            time.sleep(0.15)
            _inject_position_m(lift, target_m)

        threading.Thread(target=_deliver, daemon=True).start()
        result = lift.set(0.5, blocking=True, timeout=2.0, tolerance=0.02)
        assert result == "SUCCEEDED"

    def test_non_blocking_status_eventually_updates(self):
        """Non-blocking: status transitions to SUCCEEDED once position arrives."""
        lift, _ = _make_lift()
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
        lift, _ = _make_lift()
        _inject_position_m(lift, LIFT_MAX_CM / 2 / 100)

        lift.set(0.5, blocking=True, timeout=2.0)
        assert lift.status == "SUCCEEDED"

    def test_is_moving_true_while_non_blocking(self):
        lift, _ = _make_lift()
        lift.set(1.0, blocking=False, timeout=5.0)
        # Status is IN_PROGRESS immediately after fire-and-forget
        assert lift.is_moving is True


# ---------------------------------------------------------------------------
# get() — before and after subscription data
# ---------------------------------------------------------------------------


class TestLiftGet:
    def test_returns_none_before_any_data(self):
        lift, _ = _make_lift()
        assert lift.get() is None
        assert lift.get(norm_pos=False) is None

    def test_get_normalized_after_data(self):
        lift, _ = _make_lift()
        _inject_position_m(lift, LIFT_MAX_CM / 2 / 100)
        assert lift.get(norm_pos=True) == _approx(0.5)

    def test_get_real_pos_after_data(self):
        lift, _ = _make_lift()
        _inject_position_m(lift, LIFT_MAX_CM / 2 / 100)
        assert lift.get(norm_pos=False) == _approx(LIFT_MAX_CM / 2)

    def test_get_bottom_normalized(self):
        lift, _ = _make_lift()
        _inject_position_m(lift, 0.0)
        assert lift.get() == _approx(0.0)

    def test_get_top_normalized(self):
        lift, _ = _make_lift()
        _inject_position_m(lift, LIFT_MAX_CM / 100.0)
        assert lift.get() == _approx(1.0)

    def test_get_default_is_normalized(self):
        lift, _ = _make_lift()
        _inject_position_m(lift, 0.0)
        assert lift.get() == lift.get(norm_pos=True)


# ---------------------------------------------------------------------------
# Subscription setup
# ---------------------------------------------------------------------------


class TestLiftSubscription:
    def test_subscribes_to_correct_topic(self):
        lift, transport = _make_lift()
        lift._setup_state_subscription()
        topic = transport.subscribe.call_args[0][0]
        assert topic == LIFT_TOPICS["states"]

    def test_subscribes_with_correct_message_type(self):
        lift, transport = _make_lift()
        lift._setup_state_subscription()
        msg_type = transport.subscribe.call_args[0][1]
        assert msg_type == LIFT_TOPICS["states_type"]

    def test_subscribes_only_once(self):
        lift, transport = _make_lift()
        lift._setup_state_subscription()
        lift._setup_state_subscription()
        assert transport.subscribe.call_count == 1

    def test_namespace_prefixed_topics(self):
        lift, transport = _make_lift(namespace="robot1")
        lift._setup_state_subscription()
        lift.set(0.5, blocking=False)

        sub_topic = transport.subscribe.call_args[0][0]
        pub_topic = transport.publish.call_args[0][0]

        assert sub_topic == f"robot1/{LIFT_TOPICS['states']}"
        assert pub_topic == f"robot1/{LIFT_TOPICS['cmd']}"

    def test_namespace_setter_resubscribes(self):
        lift, transport = _make_lift()
        lift._setup_state_subscription()
        assert transport.subscribe.call_count == 1

        lift.namespace = "robot2"
        assert transport.subscribe.call_count == 2
        new_topic = transport.subscribe.call_args[0][0]
        assert new_topic == f"robot2/{LIFT_TOPICS['states']}"
