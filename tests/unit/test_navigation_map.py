"""Unit tests for the occupancy-grid map helpers on Navigation."""

from unittest.mock import MagicMock
import numpy as np
import pytest

from walkie_sdk.modules.navigation import Navigation


def _make_nav(namespace: str = "") -> tuple[Navigation, MagicMock]:
    transport = MagicMock()
    transport.is_connected = True
    return Navigation(transport=transport, namespace=namespace), transport


def _sample_grid(w=4, h=3, data=None):
    return {
        "info": {
            "resolution": 0.05,
            "width": w,
            "height": h,
            "origin": {"position": {"x": -1.0, "y": -1.0, "z": 0.0}},
        },
        "data": data if data is not None else [0] * (w * h),
    }


# ---------------------------------------------------------------------------
# _occupancy_grid_to_image
# ---------------------------------------------------------------------------


class TestOccupancyGridToImage:
    def test_shape(self):
        img = Navigation._occupancy_grid_to_image(_sample_grid(w=4, h=3))
        assert img.shape == (3, 4)
        assert img.dtype == np.uint8

    def test_value_mapping(self):
        # one of each: unknown(-1), free(0), occupied(100), occupied(50)
        grid = _sample_grid(w=4, h=1, data=[-1, 0, 100, 50])
        img = Navigation._occupancy_grid_to_image(grid)
        # single row → flipud is a no-op
        assert list(img[0]) == [127, 255, 0, 0]

    def test_row_flip(self):
        # bottom row all free (0→255), top row all occupied (100→0)
        grid = _sample_grid(w=2, h=2, data=[0, 0, 100, 100])
        img = Navigation._occupancy_grid_to_image(grid)
        # after flipud, image row 0 (top) is the ROS top row (occupied)
        assert list(img[0]) == [0, 0]
        assert list(img[1]) == [255, 255]


# ---------------------------------------------------------------------------
# map_to_pixel
# ---------------------------------------------------------------------------


class TestMapToPixel:
    def test_raises_before_fetch(self):
        nav, _ = _make_nav()
        with pytest.raises(RuntimeError):
            nav.map_to_pixel(0.0, 0.0)

    def test_origin_maps_to_bottom_left(self):
        nav, _ = _make_nav()
        nav._map_metadata = {
            "resolution": 0.05,
            "width": 4,
            "height": 3,
            "origin_x": -1.0,
            "origin_y": -1.0,
        }
        # the origin corner maps to pixel (0, height-1)
        assert nav.map_to_pixel(-1.0, -1.0) == (0, 2)

    def test_known_coords(self):
        nav, _ = _make_nav()
        nav._map_metadata = {
            "resolution": 0.05,
            "width": 400,
            "height": 400,
            "origin_x": -10.0,
            "origin_y": -10.0,
        }
        # plan's sanity check: robot at (0,0), origin (-10,-10), res 0.05
        assert nav.map_to_pixel(0.0, 0.0) == (200, 400 - 201)


# ---------------------------------------------------------------------------
# get_map_image — service path
# ---------------------------------------------------------------------------


class TestGetMapImageService:
    def test_service_success_shape_and_metadata(self):
        nav, transport = _make_nav()
        transport.call_service.return_value = {"map": _sample_grid(w=4, h=3)}
        img = nav.get_map_image()
        assert img.shape == (3, 4)
        assert nav.map_metadata == {
            "resolution": 0.05,
            "width": 4,
            "height": 3,
            "origin_x": -1.0,
            "origin_y": -1.0,
        }
        transport.subscribe.assert_not_called()

    def test_falls_back_to_topic(self):
        nav, transport = _make_nav()
        # service fails → returns no map
        transport.call_service.side_effect = Exception("no service")

        grid = _sample_grid(w=4, h=3)

        def _subscribe(topic, message_type, callback, queue_size=1):
            callback(grid)  # fire immediately
            return "handle"

        transport.subscribe.side_effect = _subscribe

        img = nav.get_map_image(timeout=1.0)
        assert img.shape == (3, 4)
        transport.unsubscribe.assert_called_once_with("handle")

    def test_returns_none_when_both_fail(self):
        nav, transport = _make_nav()
        transport.call_service.side_effect = Exception("no service")
        transport.subscribe.return_value = "handle"  # never fires callback
        assert nav.get_map_image(timeout=0.1) is None

    def test_raises_when_not_connected(self):
        nav, transport = _make_nav()
        transport.is_connected = False
        with pytest.raises(ConnectionError):
            nav.get_map_image()
