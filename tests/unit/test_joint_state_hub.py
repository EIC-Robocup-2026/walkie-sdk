"""Unit tests for walkie_sdk.modules.joint_state_hub.JointStateHub."""

from unittest.mock import MagicMock

import pytest

from walkie_sdk.modules.joint_state_hub import JointStateHub
from walkie_sdk.config.ros_topics import JOINT_STATE_TOPICS


_SAMPLE_MSG = {
    "name":     ["bl_wheel_joint", "head_servo_joint", "lift_joint", "openarm_left_joint1"],
    "position": [0.0,              0.42,               0.7435,       0.1],
    "velocity": [1.0,              0.0,                0.0,          0.2],
    "effort":   [float("nan"),     float("nan"),       float("nan"), 0.0],
}


def _make_hub(namespace: str = "") -> tuple[JointStateHub, MagicMock]:
    transport = MagicMock()
    return JointStateHub(transport=transport, namespace=namespace), transport


def _fire_callback(hub: JointStateHub, msg: dict = None) -> None:
    """Trigger the subscription callback directly."""
    hub._setup_subscription()
    cb = hub._transport.subscribe.call_args[0][2]
    cb(msg or _SAMPLE_MSG)


class TestHubSubscription:
    def test_subscribes_to_correct_topic(self):
        hub, transport = _make_hub()
        hub._setup_subscription()
        topic = transport.subscribe.call_args[0][0]
        assert topic == JOINT_STATE_TOPICS["states"]

    def test_subscribes_with_correct_message_type(self):
        hub, transport = _make_hub()
        hub._setup_subscription()
        msg_type = transport.subscribe.call_args[0][1]
        assert msg_type == JOINT_STATE_TOPICS["states_type"]

    def test_subscribes_only_once(self):
        hub, transport = _make_hub()
        hub._setup_subscription()
        hub._setup_subscription()
        assert transport.subscribe.call_count == 1

    def test_namespace_prefixes_topic(self):
        hub, transport = _make_hub(namespace="robot1")
        hub._setup_subscription()
        topic = transport.subscribe.call_args[0][0]
        assert topic == f"robot1/{JOINT_STATE_TOPICS['states']}"

    def test_namespace_setter_resubscribes(self):
        hub, transport = _make_hub()
        hub._setup_subscription()
        hub.namespace = "robot2"
        assert transport.subscribe.call_count == 2
        assert transport.subscribe.call_args[0][0] == f"robot2/{JOINT_STATE_TOPICS['states']}"


class TestHubGet:
    def test_returns_none_before_any_data(self):
        hub, _ = _make_hub()
        assert hub.get("head_servo_joint") is None
        assert hub.get("lift_joint") is None
        assert hub.get("nonexistent") is None

    def test_get_position_after_callback(self):
        hub, _ = _make_hub()
        _fire_callback(hub)
        assert hub.get("head_servo_joint") == pytest.approx(0.42)
        assert hub.get("lift_joint") == pytest.approx(0.7435)
        assert hub.get("openarm_left_joint1") == pytest.approx(0.1)

    def test_get_unknown_joint_returns_none(self):
        hub, _ = _make_hub()
        _fire_callback(hub)
        assert hub.get("not_a_real_joint") is None

    def test_get_velocity(self):
        hub, _ = _make_hub()
        _fire_callback(hub)
        assert hub.get_velocity("bl_wheel_joint") == pytest.approx(1.0)
        assert hub.get_velocity("head_servo_joint") == pytest.approx(0.0)

    def test_get_velocity_unknown_returns_none(self):
        hub, _ = _make_hub()
        _fire_callback(hub)
        assert hub.get_velocity("fake_joint") is None

    def test_get_effort_nan_is_replaced_with_zero(self):
        hub, _ = _make_hub()
        _fire_callback(hub)
        # head_servo_joint effort was NaN in the sample — should be stored as 0.0
        assert hub.get_effort("head_servo_joint") == pytest.approx(0.0)

    def test_get_effort_known_value(self):
        hub, _ = _make_hub()
        _fire_callback(hub)
        assert hub.get_effort("openarm_left_joint1") == pytest.approx(0.0)


class TestHubGetAll:
    def test_returns_empty_dict_before_data(self):
        hub, _ = _make_hub()
        assert hub.get_all() == {}

    def test_returns_all_joints_after_callback(self):
        hub, _ = _make_hub()
        _fire_callback(hub)
        all_joints = hub.get_all()
        assert set(all_joints.keys()) == set(_SAMPLE_MSG["name"])

    def test_snapshot_is_independent_copy(self):
        hub, _ = _make_hub()
        _fire_callback(hub)
        snap = hub.get_all()
        snap["head_servo_joint"]["position"] = 99.0
        assert hub.get("head_servo_joint") == pytest.approx(0.42)

    def test_data_updates_on_new_callback(self):
        hub, _ = _make_hub()
        _fire_callback(hub)
        assert hub.get("head_servo_joint") == pytest.approx(0.42)

        updated_msg = dict(_SAMPLE_MSG)
        updated_msg["position"] = [0.0, 0.78, 0.5, 0.0]
        _fire_callback(hub, updated_msg)
        assert hub.get("head_servo_joint") == pytest.approx(0.78)


class TestHubInterleavedPublishers:
    """Separate publishers (lift, head) each emit only their own joint on the
    shared topic. Messages interleave, and the cache must merge them rather than
    replace — otherwise each message wipes out the other publisher's joint."""

    _LIFT_ONLY = {
        "name":     ["lift_joint"],
        "position": [0.7435],
        "velocity": [0.0],
        "effort":   [0.0],
    }
    _HEAD_ONLY = {
        "name":     ["head_servo_joint"],
        "position": [0.42],
        "velocity": [0.0],
        "effort":   [0.0],
    }

    def test_both_joints_available_after_interleaved_messages(self):
        hub, _ = _make_hub()
        _fire_callback(hub, self._LIFT_ONLY)
        _fire_callback(hub, self._HEAD_ONLY)
        # Both must be present even though each arrived in its own message.
        assert hub.get("lift_joint") == pytest.approx(0.7435)
        assert hub.get("head_servo_joint") == pytest.approx(0.42)

    def test_order_independent(self):
        hub, _ = _make_hub()
        _fire_callback(hub, self._HEAD_ONLY)
        _fire_callback(hub, self._LIFT_ONLY)
        assert hub.get("lift_joint") == pytest.approx(0.7435)
        assert hub.get("head_servo_joint") == pytest.approx(0.42)

    def test_repeated_single_publisher_keeps_other_joint(self):
        hub, _ = _make_hub()
        _fire_callback(hub, self._LIFT_ONLY)
        _fire_callback(hub, self._HEAD_ONLY)
        # Another lift-only message must not erase the cached head joint.
        _fire_callback(hub, self._LIFT_ONLY)
        assert hub.get("head_servo_joint") == pytest.approx(0.42)
