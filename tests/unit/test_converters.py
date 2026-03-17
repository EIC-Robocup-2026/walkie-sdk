"""Unit tests for walkie_sdk.utils.converters."""

import math
import pytest

from walkie_sdk.utils.converters import (
    quaternion_to_euler,
    euler_to_quaternion,
    normalize_angle,
    degrees_to_radians,
    radians_to_degrees,
    quaternion_multiply,
    convert_bboxes_to_detection_array,
    convert_poses_to_array,
)

# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------


def _approx(a, b, tol=1e-6):
    """Assert two floats or tuples are approximately equal."""
    if isinstance(a, (tuple, list)):
        assert len(a) == len(b)
        for x, y in zip(a, b):
            assert abs(x - y) < tol, f"{x} != {y} (tol={tol})"
    else:
        assert abs(a - b) < tol, f"{a} != {b} (tol={tol})"


# ---------------------------------------------------------------------------
# quaternion_to_euler / euler_to_quaternion round-trip
# ---------------------------------------------------------------------------


class TestQuaternionEulerRoundTrip:
    @pytest.mark.parametrize(
        "roll, pitch, yaw",
        [
            (0.0, 0.0, 0.0),
            (math.pi / 4, 0.0, 0.0),
            (0.0, math.pi / 6, 0.0),
            (0.0, 0.0, math.pi / 3),
            (0.3, -0.2, 1.0),
            (-1.0, 0.5, -0.7),
        ],
    )
    def test_round_trip(self, roll, pitch, yaw):
        qx, qy, qz, qw = euler_to_quaternion(roll, pitch, yaw)
        r2, p2, y2 = quaternion_to_euler(qx, qy, qz, qw)
        _approx((r2, p2, y2), (roll, pitch, yaw), tol=1e-5)


class TestIdentityQuaternion:
    def test_identity(self):
        """Identity quaternion (0,0,0,1) should give zero euler angles."""
        r, p, y = quaternion_to_euler(0, 0, 0, 1)
        _approx((r, p, y), (0.0, 0.0, 0.0))

    def test_zero_euler_gives_identity(self):
        qx, qy, qz, qw = euler_to_quaternion(0, 0, 0)
        _approx((qx, qy, qz, qw), (0.0, 0.0, 0.0, 1.0))


# ---------------------------------------------------------------------------
# quaternion_multiply
# ---------------------------------------------------------------------------


class TestQuaternionMultiply:
    def test_identity_left(self):
        identity = (0.0, 0.0, 0.0, 1.0)
        q = (0.1, 0.2, 0.3, 0.9)
        result = quaternion_multiply(identity, q)
        _approx(result, q)

    def test_identity_right(self):
        identity = (0.0, 0.0, 0.0, 1.0)
        q = (0.1, 0.2, 0.3, 0.9)
        result = quaternion_multiply(q, identity)
        _approx(result, q)

    def test_known_product(self):
        """90-degree yaw * 90-degree yaw = 180-degree yaw."""
        q90 = euler_to_quaternion(0, 0, math.pi / 2)
        q180 = quaternion_multiply(q90, q90)
        r, p, y = quaternion_to_euler(*q180)
        _approx(y, math.pi, tol=1e-5)


# ---------------------------------------------------------------------------
# normalize_angle
# ---------------------------------------------------------------------------


class TestNormalizeAngle:
    def test_already_normalized(self):
        assert normalize_angle(1.0) == pytest.approx(1.0)

    def test_positive_overflow(self):
        # 3*pi wraps to pi (function uses strict > pi)
        assert normalize_angle(3 * math.pi) == pytest.approx(math.pi, abs=1e-10)

    def test_negative_overflow(self):
        # -3*pi wraps to -pi (function uses strict < -pi)
        assert normalize_angle(-3 * math.pi) == pytest.approx(-math.pi, abs=1e-10)

    def test_exactly_pi(self):
        assert normalize_angle(math.pi) == pytest.approx(math.pi, abs=1e-10)


# ---------------------------------------------------------------------------
# degrees / radians
# ---------------------------------------------------------------------------


class TestDegreeRadianConversions:
    def test_degrees_to_radians(self):
        assert degrees_to_radians(180) == pytest.approx(math.pi)

    def test_radians_to_degrees(self):
        assert radians_to_degrees(math.pi) == pytest.approx(180.0)

    def test_round_trip(self):
        assert radians_to_degrees(degrees_to_radians(42.0)) == pytest.approx(42.0)


# ---------------------------------------------------------------------------
# convert_bboxes_to_detection_array
# ---------------------------------------------------------------------------


class TestConvertBboxes:
    def test_empty(self):
        result = convert_bboxes_to_detection_array([])
        assert result["detections"] == []
        assert "header" in result

    def test_single_bbox(self):
        bboxes = [[320.0, 240.0, 50.0, 50.0]]
        result = convert_bboxes_to_detection_array(bboxes, frame_id="head_camera")
        assert len(result["detections"]) == 1
        det = result["detections"][0]
        assert det["bbox"]["center"]["position"]["x"] == 320.0
        assert det["bbox"]["size_x"] == 50.0
        assert result["header"]["frame_id"] == "head_camera"

    def test_multiple_bboxes(self):
        bboxes = [[1, 2, 3, 4], [5, 6, 7, 8]]
        result = convert_bboxes_to_detection_array(bboxes)
        assert len(result["detections"]) == 2


# ---------------------------------------------------------------------------
# convert_poses_to_array
# ---------------------------------------------------------------------------


class TestConvertPosesToArray:
    def test_empty(self):
        assert convert_poses_to_array({"poses": []}) == []
        assert convert_poses_to_array({}) == []

    def test_extracts_xyz(self):
        data = {
            "poses": [
                {"position": {"x": 1.0, "y": 2.0, "z": 3.0}},
                {"position": {"x": 4.0, "y": 5.0, "z": 6.0}},
            ]
        }
        result = convert_poses_to_array(data)
        assert result == [[1.0, 2.0, 3.0], [4.0, 5.0, 6.0]]
