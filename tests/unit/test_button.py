"""Unit tests for walkie_sdk.modules.button.Button (evdev backend)."""

import threading
from unittest.mock import MagicMock, patch

import pytest

from walkie_sdk.modules.button import Button, _resolve_keycode

try:
    from evdev import ecodes

    _HAVE_EVDEV = True
except ImportError:  # pragma: no cover - evdev is a Linux dep
    _HAVE_EVDEV = False


# ---------------------------------------------------------------------------
# _resolve_keycode helper
# ---------------------------------------------------------------------------


class TestResolveKeycode:
    def test_none_is_any(self):
        assert _resolve_keycode(None) is None

    def test_empty_is_any(self):
        assert _resolve_keycode("") is None

    def test_any_string_is_any(self):
        assert _resolve_keycode("any") is None
        assert _resolve_keycode("ANY") is None

    def test_int_passes_through(self):
        assert _resolve_keycode(186) == 186

    def test_decimal_string(self):
        assert _resolve_keycode("186") == 186

    def test_leftover_x11_keysym_falls_back_to_any(self):
        # Old config value; not a valid evdev name → match any key.
        assert _resolve_keycode("0x1008ff47") is None

    @pytest.mark.skipif(not _HAVE_EVDEV, reason="evdev not installed")
    def test_evdev_name(self):
        assert _resolve_keycode("KEY_F16") == ecodes.KEY_F16

    @pytest.mark.skipif(not _HAVE_EVDEV, reason="evdev not installed")
    def test_bare_name_gets_key_prefix(self):
        assert _resolve_keycode("f16") == ecodes.KEY_F16
        assert _resolve_keycode("F16") == ecodes.KEY_F16

    @pytest.mark.skipif(not _HAVE_EVDEV, reason="evdev not installed")
    def test_unknown_name_falls_back_to_any(self):
        assert _resolve_keycode("not_a_real_key") is None


# ---------------------------------------------------------------------------
# is_pressed state transitions via _handle_event
# ---------------------------------------------------------------------------


class TestButtonState:
    def test_initially_not_pressed(self):
        assert Button().is_pressed is False

    def test_key_down_sets_true(self):
        btn = Button(key="any")
        btn._handle_event(code=100, value=1)
        assert btn.is_pressed is True

    def test_key_up_sets_false(self):
        btn = Button(key="any")
        btn._handle_event(code=100, value=1)
        btn._handle_event(code=100, value=0)
        assert btn.is_pressed is False

    def test_autorepeat_keeps_pressed(self):
        btn = Button(key="any")
        btn._handle_event(code=100, value=1)
        btn._handle_event(code=100, value=2)  # autorepeat
        assert btn.is_pressed is True

    def test_double_down_stays_true(self):
        btn = Button(key="any")
        btn._handle_event(code=100, value=1)
        btn._handle_event(code=100, value=1)
        assert btn.is_pressed is True

    def test_double_up_stays_false(self):
        btn = Button(key="any")
        btn._handle_event(code=100, value=0)
        btn._handle_event(code=100, value=0)
        assert btn.is_pressed is False

    def test_specific_key_matches(self):
        btn = Button(key=186)
        btn._handle_event(code=186, value=1)
        assert btn.is_pressed is True

    def test_specific_key_ignores_other_keys(self):
        btn = Button(key=186)
        btn._handle_event(code=100, value=1)  # different key
        assert btn.is_pressed is False

    def test_specific_key_release_of_other_key_ignored(self):
        btn = Button(key=186)
        btn._handle_event(code=186, value=1)
        btn._handle_event(code=100, value=0)  # release of a different key
        assert btn.is_pressed is True

    def test_key_property(self):
        assert Button(key="f3").key == "f3"


# ---------------------------------------------------------------------------
# Device discovery
# ---------------------------------------------------------------------------


class TestFindDevice:
    def test_explicit_existing_device(self, tmp_path):
        dev = tmp_path / "event0"
        dev.write_text("")
        btn = Button(device=str(dev))
        assert btn._find_device() == str(dev)

    def test_explicit_missing_device_returns_none(self):
        btn = Button(device="/dev/input/does-not-exist")
        assert btn._find_device() is None

    def test_by_id_symlink_match(self):
        link = (
            "/dev/input/by-id/"
            "usb-Raspberry_Pi_Pico_5303284720A83B1C-if03-event-kbd"
        )
        with patch(
            "walkie_sdk.modules.button.glob.glob", return_value=[link]
        ), patch(
            "walkie_sdk.modules.button.os.path.realpath",
            return_value="/dev/input/event11",
        ):
            assert Button()._find_device() == "/dev/input/event11"

    def test_no_matching_by_id_returns_none_when_no_evdev_devices(self):
        with patch("walkie_sdk.modules.button.glob.glob", return_value=[]), patch(
            "evdev.list_devices", return_value=[]
        ):
            assert Button()._find_device() is None


# ---------------------------------------------------------------------------
# Thread safety
# ---------------------------------------------------------------------------


class TestButtonThreadSafety:
    def test_concurrent_handle_event(self):
        btn = Button(key="any")
        errors = []

        def _hammer():
            try:
                for _ in range(200):
                    btn._handle_event(code=100, value=1)
                    btn._handle_event(code=100, value=0)
            except Exception as e:  # pragma: no cover
                errors.append(e)

        threads = [threading.Thread(target=_hammer) for _ in range(4)]
        for t in threads:
            t.start()
        for t in threads:
            t.join()

        assert errors == []
        assert isinstance(btn.is_pressed, bool)


# ---------------------------------------------------------------------------
# _start / _stop lifecycle
# ---------------------------------------------------------------------------


class TestButtonLifecycle:
    def test_start_spawns_thread(self):
        btn = Button()
        # Keep the reader loop from touching real hardware.
        with patch.object(btn, "_run", return_value=None):
            btn._start()
            assert btn._thread is not None
            btn._stop()
        assert btn._thread is None

    def test_start_is_idempotent(self):
        btn = Button()
        alive_thread = MagicMock()
        alive_thread.is_alive.return_value = True
        btn._thread = alive_thread

        with patch("walkie_sdk.modules.button.threading.Thread") as mk_thread:
            btn._start()
            mk_thread.assert_not_called()

    def test_start_without_evdev_is_safe(self):
        btn = Button()
        # Simulate evdev missing: the import inside _start raises ImportError.
        import builtins

        real_import = builtins.__import__

        def fake_import(name, *args, **kwargs):
            if name == "evdev":
                raise ImportError("no evdev")
            return real_import(name, *args, **kwargs)

        with patch("builtins.__import__", side_effect=fake_import):
            btn._start()  # should not raise
        assert btn._thread is None

    def test_stop_resets_state_and_thread(self):
        btn = Button()
        btn._pressed = True
        btn._stop()  # no thread running
        assert btn.is_pressed is False
        assert btn._thread is None

    def test_stop_with_no_thread_is_safe(self):
        btn = Button()
        btn._stop()  # should not raise
        assert btn.is_pressed is False
