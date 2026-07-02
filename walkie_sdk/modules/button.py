"""
Button - Physical button state via Pi Pico USB HID keyboard (evdev).

The Pi Pico acts as a USB HID keyboard device. When the physical button is
pressed the Pico sends a keypress; on release it sends the corresponding
key-up. This module reads those events directly from the Linux input
subsystem via ``evdev``.

Why evdev and not pynput: the button listener has to run on the machine the
Pico is plugged into — the robot — which is headless (no X server / no
``DISPLAY``, driven over SSH). pynput's Linux keyboard backend is X11-only and
fails to even import in that environment, so it can never receive events
there. evdev reads ``/dev/input/event*`` directly and works headless.

Permissions: reading ``/dev/input/event*`` requires membership in the
``input`` group. If the button reports "permission denied", run::

    sudo usermod -aG input $USER   # then log out and back in

No ROS transport is involved — this is a purely local HID listener.
"""

import glob
import os
import select
import threading
from typing import Optional

# Substring used to auto-discover the Pico keyboard among the input devices.
# Matches the /dev/input/by-id/usb-Raspberry_Pi_Pico_*-event-kbd symlink and
# the "Raspberry Pi Pico Keyboard" device name.
_DEFAULT_DEVICE_HINT = "Raspberry_Pi_Pico"


def _resolve_keycode(key) -> Optional[int]:
    """Resolve a user-supplied key spec to an evdev key code, or None for "any".

    Accepts:
    - ``None`` / ``""`` / ``"any"`` → ``None`` (match ANY key on the device)
    - an ``int``                    → used directly as an evdev key code
    - a decimal string ``"186"``    → that key code
    - an evdev name ``"KEY_F16"``   → resolved via ``evdev.ecodes``
    - a bare name ``"f16"``         → treated as ``"KEY_F16"``

    Anything unrecognised (e.g. a leftover X11 keysym like ``"0x1008ff47"``)
    resolves to ``None`` → match any key. That is the correct behaviour for a
    dedicated single-button device: whatever key the Pico emits counts as the
    button.
    """
    if key is None:
        return None
    if isinstance(key, int):
        return key
    s = str(key).strip()
    if s == "" or s.lower() == "any":
        return None
    if s.isdigit():
        return int(s)
    try:
        from evdev import ecodes
    except ImportError:
        return None
    name = s.upper()
    if not name.startswith("KEY_"):
        name = "KEY_" + name
    code = getattr(ecodes, name, None)
    return code if isinstance(code, int) else None


class Button:
    """
    Physical button state via Pi Pico USB HID keyboard input (evdev).

    Reads key events straight from the Linux input device the Pico exposes and
    tracks the current pressed state. Runs a background reader thread; query
    the state with :attr:`is_pressed`.

    Args:
        key: Which key on the device counts as the button. ``"any"`` (default)
            treats *any* key from the Pico keyboard as the button — correct for
            a dedicated single-button device. May also be an evdev key name
            (``"f16"``, ``"KEY_F16"``) or numeric key code to match a specific
            key.
        device: Explicit input device path (e.g. ``"/dev/input/event11"``).
            When ``None`` (default) the Pico keyboard is auto-discovered.
        device_hint: Substring used to auto-discover the device by its
            ``/dev/input/by-id`` link or device name. Defaults to the Pico.

    Example:
        ```python
        # Accessed via WalkieRobot:
        while True:
            if bot.button.is_pressed:
                print("Button held!")
        ```
    """

    def __init__(
        self,
        key: str = "any",
        device: Optional[str] = None,
        device_hint: str = _DEFAULT_DEVICE_HINT,
    ):
        self._key = key
        self._keycode = _resolve_keycode(key)
        self._device = device
        self._device_hint = device_hint
        self._pressed = False
        self._lock = threading.Lock()
        self._thread: Optional[threading.Thread] = None
        self._running = False
        self._dev = None

    @property
    def key(self) -> str:
        """The key spec this button listens for."""
        return self._key

    @property
    def is_pressed(self) -> bool:
        """True while the physical button is held down, False otherwise."""
        with self._lock:
            return self._pressed

    def _find_device(self) -> Optional[str]:
        """Locate the Pico keyboard input device path, or None if not found."""
        if self._device:
            return self._device if os.path.exists(self._device) else None

        hint = self._device_hint.lower()

        # 1. Prefer the stable by-id keyboard symlink.
        for link in sorted(glob.glob("/dev/input/by-id/*event-kbd")):
            if hint in os.path.basename(link).lower():
                return os.path.realpath(link)

        # 2. Fall back to scanning device names via evdev.
        try:
            import evdev
        except ImportError:
            return None
        hint_name = hint.replace("_", " ")
        for path in evdev.list_devices():
            try:
                dev = evdev.InputDevice(path)
            except Exception:
                continue
            try:
                name = dev.name.lower()
            finally:
                dev.close()
            if hint_name in name and "keyboard" in name:
                return path
        return None

    def _handle_event(self, code: int, value: int) -> None:
        """Update pressed state from a raw EV_KEY event.

        ``value``: 0 = key up, 1 = key down, 2 = autorepeat (still held).
        """
        if self._keycode is not None and code != self._keycode:
            return
        with self._lock:
            self._pressed = value != 0

    def _run(self) -> None:
        """Reader loop: open the device and translate key events into state."""
        from evdev import ecodes

        path = self._find_device()
        if path is None:
            print(
                f"  ⚠ Button: no Pico keyboard input device found "
                f"(hint '{self._device_hint}'). Button will not receive events."
            )
            return

        try:
            import evdev

            dev = evdev.InputDevice(path)
        except PermissionError:
            print(
                f"  ⚠ Button: permission denied reading {path}.\n"
                f"    Add your user to the 'input' group, then log out/in:\n"
                f"      sudo usermod -aG input $USER"
            )
            return
        except Exception as e:
            print(f"  ⚠ Button: failed to open {path}: {e}")
            return

        self._dev = dev
        print(f"  ✓ Button listening on {path} ({dev.name})")

        try:
            while self._running:
                # Poll with a timeout so _stop() is honoured within 0.5 s.
                try:
                    ready, _, _ = select.select([dev.fd], [], [], 0.5)
                except OSError:
                    break
                if not ready:
                    continue
                try:
                    events = list(dev.read())
                except OSError:
                    break  # device unplugged or fd closed
                for event in events:
                    if event.type == ecodes.EV_KEY:
                        self._handle_event(event.code, event.value)
        finally:
            try:
                dev.close()
            except Exception:
                pass
            self._dev = None

    def _start(self) -> None:
        """Start the evdev reader thread. Idempotent."""
        if self._thread is not None and self._thread.is_alive():
            return
        try:
            import evdev  # noqa: F401
        except ImportError:
            print("  ⚠ evdev not installed — button module will not receive events.")
            return
        self._running = True
        thread = threading.Thread(target=self._run, name="walkie-button", daemon=True)
        thread.start()
        self._thread = thread

    def _stop(self) -> None:
        """Stop the reader thread and reset state."""
        self._running = False
        thread = self._thread
        if thread is not None:
            # The reader loop exits within its select() timeout (0.5 s).
            thread.join(timeout=1.5)
        self._thread = None
        with self._lock:
            self._pressed = False
