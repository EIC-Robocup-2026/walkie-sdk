"""
Button - Physical button state module via Pi Pico USB HID keyboard.

The Pi Pico acts as a USB HID keyboard device. When the physical button is
pressed the Pico sends a function-key keypress (e.g. F16); on release it
sends the corresponding key-up. This module listens globally for those
events and exposes the current state as a bool.

No ROS transport is involved — this is a purely local HID listener.
"""

import re
import threading
from typing import Optional

# pynput's Key enum only names F1–F12 (sometimes up to F20).
# F13+ arrive as raw KeyCode(vk=N) where vk = 65469 + key_number (X11/Linux).
_FKEY_VK_BASE = 65469  # XK_F1 = 65470 = 65469 + 1


def _key_matches(event_key, key_name: str) -> bool:
    """Return True if event_key corresponds to the named key.

    key_name can be:
    - A pynput Key name: ``"f1"`` … ``"f12"``, ``"space"``, etc.
    - A hex keysym from xev: ``"0x1008ff47"`` (paste directly from xev output)
    - A high F-key name: ``"f13"`` … ``"f24"`` (X11 vk formula fallback)
    """
    key_name_lower = key_name.lower()
    try:
        from pynput import keyboard as _kb
        # Hex keysym (e.g. "0x1008ff47" from xev) — most reliable for remapped keys
        if key_name_lower.startswith('0x'):
            vk = int(key_name_lower, 16)
            return getattr(event_key, 'vk', None) == vk
        # Named key (covers F1–F12 and all other Key enum members)
        target = getattr(_kb.Key, key_name_lower, None)
        if target is not None:
            return event_key == target
        # High F-keys (F13+): pynput emits KeyCode(vk=65469+n) on Linux/X11
        m = re.match(r'^f(\d+)$', key_name_lower)
        if m:
            vk = _FKEY_VK_BASE + int(m.group(1))
            return getattr(event_key, 'vk', None) == vk
    except Exception:
        pass
    return getattr(event_key, 'char', None) == key_name


class Button:
    """
    Physical button state via Pi Pico USB HID keyboard input.

    Listens for a configurable function key (sent by the Pi Pico when the
    button is pressed/released) and tracks the current pressed state.

    Args:
        key: Name of the key the Pi Pico sends (e.g. ``"f16"``). Any F-key
             F1–F24 is supported. Defaults to ``"f16"``.

    Example:
        ```python
        # Accessed via WalkieRobot:
        while True:
            if bot.button.is_pressed:
                print("Button held!")
        ```
    """

    def __init__(self, key: str = "0x1008ff47"):
        self._key = key
        self._pressed = False
        self._lock = threading.Lock()
        self._listener: Optional[object] = None

    @property
    def key(self) -> str:
        """The keyboard key name this button listens for."""
        return self._key

    @property
    def is_pressed(self) -> bool:
        """True while the physical button is held down, False otherwise."""
        with self._lock:
            return self._pressed

    def _on_press(self, key) -> None:
        if _key_matches(key, self._key):
            with self._lock:
                self._pressed = True

    def _on_release(self, key) -> None:
        if _key_matches(key, self._key):
            with self._lock:
                self._pressed = False

    def _start(self) -> None:
        """Start the keyboard listener. Idempotent."""
        if self._listener is not None:
            try:
                if self._listener.is_alive():
                    return
            except Exception:
                pass

        try:
            from pynput import keyboard as _kb
            listener = _kb.Listener(
                on_press=self._on_press,
                on_release=self._on_release,
            )
            listener.daemon = True
            listener.start()
            self._listener = listener
        except ImportError:
            print("  ⚠ pynput not installed — button module will not receive events.")
        except Exception as e:
            print(f"  ⚠ Button listener failed to start: {e}")

    def _stop(self) -> None:
        """Stop the keyboard listener."""
        if self._listener is not None:
            try:
                self._listener.stop()
            except Exception:
                pass
            self._listener = None
        with self._lock:
            self._pressed = False
