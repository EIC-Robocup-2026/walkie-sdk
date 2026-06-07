"""Unit tests for depth-camera support (offline).

Covers:
  - CameraTransportInterface optional-depth defaults (None / {}).
  - Camera / MultiCamera delegation to the transport's get_depth().
  - ZenohCamera depth decode for 32FC1 (float32 m) and 16UC1 (uint16 mm),
    guarded so the suite still runs without zenoh/cv2 installed.
"""

import numpy as np
import pytest

from walkie_sdk.core.interfaces import CameraTransportInterface
from walkie_sdk.modules.camera import Camera
from walkie_sdk.modules.multi_camera import MultiCamera


class FakeCamera(CameraTransportInterface):
    """Minimal in-memory transport exposing canned color + depth frames."""

    def __init__(self, depth=None, depth_map=None):
        self._depth = depth
        self._depth_map = depth_map or {}
        self._streaming = True

    # --- required abstract surface (unused color path) ---
    def connect(self):
        self._streaming = True

    def disconnect(self):
        self._streaming = False

    @property
    def is_streaming(self):
        return self._streaming

    def get_frame(self):
        return None

    @property
    def frame_shape(self):
        return None

    # --- depth overrides ---
    def get_depth(self, camera_name=None):
        if camera_name is not None:
            return self._depth_map.get(camera_name, self._depth)
        return self._depth

    def get_all_depth(self):
        return dict(self._depth_map)


def test_interface_depth_defaults_to_none_and_empty():
    """A transport that doesn't override depth reports no depth, not an error."""

    class NoDepth(FakeCamera):
        pass

    # Use the base defaults rather than FakeCamera's overrides.
    t = NoDepth()
    assert CameraTransportInterface.get_depth(t) is None
    assert CameraTransportInterface.get_all_depth(t) == {}


def test_camera_get_depth_delegates_and_passes_through():
    depth = np.full((4, 6), 1.5, dtype=np.float32)
    cam = Camera(FakeCamera(depth=depth))
    out = cam.get_depth()
    assert out is depth  # Camera does not copy; transport owns the copy semantics
    assert out.dtype == np.float32 and out.shape == (4, 6)


def test_camera_get_depth_none_when_unavailable():
    assert Camera(FakeCamera(depth=None)).get_depth() is None


def test_multicamera_get_depth_by_name():
    head = np.zeros((2, 2), np.float32)
    left = np.ones((2, 2), np.float32)
    cams = MultiCamera(FakeCamera(depth_map={"head": head, "left": left}))
    assert cams.get_depth("head") is head
    assert cams.get_depth("left") is left
    assert cams.get_all_depth() == {"head": head, "left": left}


def test_multicamera_dict_transport_depth():
    head = np.zeros((2, 2), np.float32)
    transports = {"head": FakeCamera(depth=head), "left": FakeCamera(depth=None)}
    cams = MultiCamera(transports)
    assert cams.get_depth("head") is head
    assert cams.get_depth("left") is None
    assert cams.get_depth("missing") is None
    # get_all_depth skips cameras with no depth frame yet.
    assert cams.get_all_depth() == {"head": head}


# --- Real decode path (skipped if zenoh is not installed) ---
# The depth decoder is pure NumPy; only ZenohCamera's color path needs cv2.
# So we require only zenoh here and stub the unused cv2 in _make_zenoh_camera,
# letting these run even where the cv2 wheel has a NumPy ABI mismatch.

try:
    import zenoh  # noqa: F401

    _ZENOH_OK = True
except Exception:
    _ZENOH_OK = False

zenoh_required = pytest.mark.skipif(not _ZENOH_OK, reason="zenoh not installed")


class _FakeMsg:
    def __init__(self, height, width, encoding, data):
        self.height = height
        self.width = width
        self.encoding = encoding
        self.data = data


def _make_zenoh_camera():
    import walkie_sdk.core.transports.zenoh as zt

    # __init__ requires a non-None cv2 (for the color path) but the depth
    # decoder never touches it -- stub it so the decode tests don't depend
    # on a working cv2 build. __init__ does no network I/O.
    if zt.cv2 is None:
        zt.cv2 = object()
    return zt.ZenohCamera(host="127.0.0.1")


@zenoh_required
def test_zenoh_decode_32fc1_metres():
    cam = _make_zenoh_camera()
    src = np.array([[0.5, 1.0, np.nan], [2.0, 3.5, 4.0]], dtype=np.float32)
    cam._on_depth("head", _FakeMsg(2, 3, "32FC1", src.tobytes()))

    out = cam.get_depth("head")
    assert out is not None
    assert out.dtype == np.float32 and out.shape == (2, 3)
    np.testing.assert_array_equal(out[~np.isnan(out)], src[~np.isnan(src)])
    assert np.isnan(out[0, 2])  # invalid pixel preserved as NaN
    # get_depth returns a copy, not the cached buffer.
    assert out is not cam.get_depth("head")


@zenoh_required
def test_zenoh_decode_16uc1_millimetres():
    cam = _make_zenoh_camera()
    src = np.array([[100, 250], [1000, 65535]], dtype=np.uint16)
    cam._on_depth("head", _FakeMsg(2, 2, "16UC1", src.tobytes()))

    out = cam.get_depth("head")
    assert out is not None
    assert out.dtype == np.uint16 and out.shape == (2, 2)
    np.testing.assert_array_equal(out, src)


@zenoh_required
def test_zenoh_unsupported_depth_encoding_ignored():
    cam = _make_zenoh_camera()
    cam._on_depth("head", _FakeMsg(2, 2, "rgb8", b"\x00" * 12))
    assert cam.get_depth("head") is None


@zenoh_required
def test_zenoh_get_depth_none_before_first_frame():
    assert _make_zenoh_camera().get_depth("head") is None
