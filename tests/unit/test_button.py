"""Unit tests for walkie_sdk.modules.button.Button."""

import threading
from unittest.mock import MagicMock, patch

import pytest

from walkie_sdk.modules.button import Button, _key_matches


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------


def _make_f1_key():
    """Return a mock pynput Key object that looks like F1."""
    try:
        from pynput import keyboard as _kb
        return _kb.Key.f1
    except Exception:
        key = MagicMock()
        key.char = None
        return key


def _make_other_key():
    """Return a mock pynput Key object that is NOT F1."""
    try:
        from pynput import keyboard as _kb
        return _kb.Key.f2
    except Exception:
        key = MagicMock()
        key.char = "x"
        return key


# ---------------------------------------------------------------------------
# _key_matches helper
# ---------------------------------------------------------------------------


class TestKeyMatches:
    def test_f1_matches_f1(self):
        try:
            from pynput import keyboard as _kb
            assert _key_matches(_kb.Key.f1, "f1") is True
        except ImportError:
            pytest.skip("pynput not installed")

    def test_f2_does_not_match_f1(self):
        try:
            from pynput import keyboard as _kb
            assert _key_matches(_kb.Key.f2, "f1") is False
        except ImportError:
            pytest.skip("pynput not installed")

    def test_uppercase_key_name_works(self):
        try:
            from pynput import keyboard as _kb
            assert _key_matches(_kb.Key.f1, "F1") is True
        except ImportError:
            pytest.skip("pynput not installed")

    def test_char_key_matches_by_char(self):
        key = MagicMock()
        key.char = "b"
        assert _key_matches(key, "b") is True

    def test_char_key_no_match_wrong_char(self):
        key = MagicMock()
        key.char = "b"
        assert _key_matches(key, "a") is False


# ---------------------------------------------------------------------------
# is_pressed state transitions via _on_press / _on_release
# ---------------------------------------------------------------------------


class TestButtonState:
    def test_initially_not_pressed(self):
        btn = Button(key="f1")
        assert btn.is_pressed is False

    def test_press_sets_true(self):
        btn = Button(key="f1")
        try:
            from pynput import keyboard as _kb
            btn._on_press(_kb.Key.f1)
        except ImportError:
            pytest.skip("pynput not installed")
        assert btn.is_pressed is True

    def test_release_sets_false(self):
        btn = Button(key="f1")
        try:
            from pynput import keyboard as _kb
            btn._on_press(_kb.Key.f1)
            btn._on_release(_kb.Key.f1)
        except ImportError:
            pytest.skip("pynput not installed")
        assert btn.is_pressed is False

    def test_wrong_key_press_ignored(self):
        btn = Button(key="f1")
        try:
            from pynput import keyboard as _kb
            btn._on_press(_kb.Key.f2)
        except ImportError:
            pytest.skip("pynput not installed")
        assert btn.is_pressed is False

    def test_wrong_key_release_ignored(self):
        btn = Button(key="f1")
        try:
            from pynput import keyboard as _kb
            btn._on_press(_kb.Key.f1)
            btn._on_release(_kb.Key.f2)  # wrong key
        except ImportError:
            pytest.skip("pynput not installed")
        assert btn.is_pressed is True

    def test_double_press_stays_true(self):
        btn = Button(key="f1")
        try:
            from pynput import keyboard as _kb
            btn._on_press(_kb.Key.f1)
            btn._on_press(_kb.Key.f1)
        except ImportError:
            pytest.skip("pynput not installed")
        assert btn.is_pressed is True

    def test_double_release_stays_false(self):
        btn = Button(key="f1")
        try:
            from pynput import keyboard as _kb
            btn._on_release(_kb.Key.f1)
            btn._on_release(_kb.Key.f1)
        except ImportError:
            pytest.skip("pynput not installed")
        assert btn.is_pressed is False

    def test_key_property(self):
        btn = Button(key="f3")
        assert btn.key == "f3"


# ---------------------------------------------------------------------------
# Thread safety
# ---------------------------------------------------------------------------


class TestButtonThreadSafety:
    def test_concurrent_press_release(self):
        try:
            from pynput import keyboard as _kb
        except ImportError:
            pytest.skip("pynput not installed")

        btn = Button(key="f1")
        errors = []

        def _press_release():
            try:
                for _ in range(100):
                    btn._on_press(_kb.Key.f1)
                    btn._on_release(_kb.Key.f1)
            except Exception as e:
                errors.append(e)

        threads = [threading.Thread(target=_press_release) for _ in range(4)]
        for t in threads:
            t.start()
        for t in threads:
            t.join()

        assert errors == []
        # Final state is deterministic only in single-thread; here just verify no crash
        assert isinstance(btn.is_pressed, bool)


# ---------------------------------------------------------------------------
# _start / _stop lifecycle
# ---------------------------------------------------------------------------


class TestButtonLifecycle:
    def test_start_sets_listener(self):
        btn = Button(key="f1")
        mock_listener = MagicMock()
        mock_listener.is_alive.return_value = True
        mock_kb = MagicMock()
        mock_kb.Listener.return_value = mock_listener

        with patch.dict("sys.modules", {"pynput": MagicMock(), "pynput.keyboard": mock_kb}):
            with patch("walkie_sdk.modules.button._key_matches", return_value=False):
                btn._start()

        assert btn._listener is not None

    def test_start_is_idempotent(self):
        btn = Button(key="f1")
        mock_listener = MagicMock()
        mock_listener.is_alive.return_value = True
        btn._listener = mock_listener

        call_count_before = mock_listener.start.call_count
        btn._start()
        # A second _start() with an alive listener should not start another
        assert mock_listener.start.call_count == call_count_before

    def test_stop_clears_listener_and_resets_state(self):
        btn = Button(key="f1")
        mock_listener = MagicMock()
        btn._listener = mock_listener
        btn._pressed = True

        btn._stop()

        mock_listener.stop.assert_called_once()
        assert btn._listener is None
        assert btn.is_pressed is False

    def test_stop_with_no_listener_is_safe(self):
        btn = Button(key="f1")
        btn._stop()  # should not raise
        assert btn.is_pressed is False
