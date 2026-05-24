"""
Interactive head tilt test with live camera preview.

Displays the robot's camera feed in an OpenCV window while letting you
drive the head servo angle with keyboard shortcuts.

Controls (focus the camera window first):
  W  or  Up arrow    tilt up   (angle decreases toward negative)
  S  or  Down arrow  tilt down (angle increases toward positive)
  R                  reset to 0.0 rad (look forward)
  [                  halve the step size
  ]                  double the step size
  T                  type a specific angle in the terminal
  Q  or  Esc         quit

Safe range: ±π/4 rad (±45°).  Positive = camera faces floor.

Usage:
    python tests/test_head_tilt_interactive.py
    python tests/test_head_tilt_interactive.py --ip 192.168.1.100
    python tests/test_head_tilt_interactive.py --ip 192.168.1.100 --port 9090
    python tests/test_head_tilt_interactive.py --ip 192.168.1.100 --step 0.1
    python tests/test_head_tilt_interactive.py --ip 192.168.1.100 --namespace robot1
    python tests/test_head_tilt_interactive.py --no-camera   # terminal-only mode
"""

import argparse
import math
import queue
import sys
import threading
import time

import cv2
import numpy as np

from walkie_sdk import WalkieRobot
from walkie_sdk.modules.head import HEAD_TILT_MIN, HEAD_TILT_MAX

# ── Constants ──────────────────────────────────────────────────────────────

_STEP_DEFAULT = 0.05      # radians per keypress (~2.9°)
_STEP_MIN     = 0.005
_STEP_MAX     = 0.2

_WIN_TITLE    = "Head Tilt Control  |  Q / Esc = quit"

_FONT         = cv2.FONT_HERSHEY_SIMPLEX
_COLOR_WHITE  = (255, 255, 255)
_COLOR_GREEN  = (0, 220, 80)
_COLOR_YELLOW = (0, 200, 220)
_COLOR_RED    = (60, 60, 230)
_COLOR_BG     = (30, 30, 30)

# OpenCV key codes (platform-specific on Linux)
_KEY_UP    = 82
_KEY_DOWN  = 84
_KEY_ESC   = 27


# ── Overlay helpers ────────────────────────────────────────────────────────


def _make_placeholder(w: int = 640, h: int = 480) -> np.ndarray:
    """Dark grey frame shown when the camera has no data yet."""
    frame = np.full((h, w, 3), _COLOR_BG, dtype=np.uint8)
    cv2.putText(frame, "Waiting for camera...", (w // 2 - 130, h // 2),
                _FONT, 0.7, _COLOR_WHITE, 1, cv2.LINE_AA)
    return frame


def _draw_overlay(frame: np.ndarray, angle: float, step: float) -> np.ndarray:
    """Stamp angle info and key hints onto the frame (modifies a copy)."""
    out = frame.copy()
    h, w = out.shape[:2]

    angle_deg = math.degrees(angle)
    min_deg   = math.degrees(HEAD_TILT_MIN)
    max_deg   = math.degrees(HEAD_TILT_MAX)
    step_deg  = math.degrees(step)

    # ── angle bar background ──────────────────────────────────────────────
    bar_x1, bar_y1 = 10, 10
    bar_x2, bar_y2 = 310, 90
    cv2.rectangle(out, (bar_x1, bar_y1), (bar_x2, bar_y2), (0, 0, 0), -1)
    cv2.rectangle(out, (bar_x1, bar_y1), (bar_x2, bar_y2), (80, 80, 80), 1)

    # progress bar (maps angle linearly across the range)
    frac = (angle - HEAD_TILT_MIN) / (HEAD_TILT_MAX - HEAD_TILT_MIN)
    bar_inner_x1 = bar_x1 + 8
    bar_inner_x2 = bar_x1 + 8 + int((bar_x2 - bar_x1 - 16) * frac)
    bar_inner_y  = bar_y1 + 62
    cv2.rectangle(out, (bar_x1 + 8, bar_inner_y), (bar_x2 - 8, bar_inner_y + 12), (60, 60, 60), -1)
    if bar_inner_x2 > bar_inner_x1:
        cv2.rectangle(out, (bar_inner_x1, bar_inner_y), (bar_inner_x2, bar_inner_y + 12), _COLOR_GREEN, -1)

    # text
    cv2.putText(out, f"Tilt: {angle:+.3f} rad  ({angle_deg:+.1f} deg)",
                (bar_x1 + 8, bar_y1 + 24), _FONT, 0.55, _COLOR_GREEN, 1, cv2.LINE_AA)
    cv2.putText(out, f"Range [{min_deg:.0f} .. {max_deg:.0f} deg]   Step {step_deg:.1f} deg",
                (bar_x1 + 8, bar_y1 + 46), _FONT, 0.42, _COLOR_YELLOW, 1, cv2.LINE_AA)

    # ── key hint strip at bottom ──────────────────────────────────────────
    strip_h = 26
    cv2.rectangle(out, (0, h - strip_h), (w, h), (0, 0, 0), -1)
    hints = "W/↑ up    S/↓ down    R reset    [ ] step    T type    Q quit"
    cv2.putText(out, hints, (8, h - 8), _FONT, 0.38, _COLOR_WHITE, 1, cv2.LINE_AA)

    return out


# ── Terminal input thread ──────────────────────────────────────────────────


def _input_thread(q: "queue.Queue[str]") -> None:
    """Reads lines from stdin and pushes them onto the queue."""
    while True:
        try:
            line = input()
        except (EOFError, KeyboardInterrupt):
            q.put("__quit__")
            return
        q.put(line.strip())


# ── Core interactive session ───────────────────────────────────────────────


def run(bot: WalkieRobot, step: float, camera_enabled: bool) -> None:
    angle = 0.0
    bot.head.tilt(angle)

    cv2.namedWindow(_WIN_TITLE, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(_WIN_TITLE, 640, 520)

    stdin_q: "queue.Queue[str]" = queue.Queue()
    t = threading.Thread(target=_input_thread, args=(stdin_q,), daemon=True)
    t.start()

    waiting_for_type = False

    print(f"\nHead tilt ready.  Current angle: {angle:.3f} rad")
    print("Focus the camera window and use W/S/R/[/]/Q — or press T to type an angle here.\n")

    try:
        while True:
            # ── camera frame ──────────────────────────────────────────────
            frame = None
            if camera_enabled and bot.camera is not None:
                frame = bot.camera.get_frame()

            if frame is None:
                frame = _make_placeholder()

            frame = _draw_overlay(frame, angle, step)
            cv2.imshow(_WIN_TITLE, frame)

            # ── keyboard (OpenCV window) ──────────────────────────────────
            key = cv2.waitKey(30) & 0xFF

            if key in (ord("q"), ord("Q"), _KEY_ESC):
                break

            elif key in (ord("w"), ord("W"), _KEY_UP):
                new = angle - step
                new = max(HEAD_TILT_MIN, new)
                try:
                    bot.head.tilt(new)
                    angle = new
                    print(f"  tilt {angle:+.3f} rad ({math.degrees(angle):+.1f}°)")
                except ValueError as e:
                    print(f"  {e}")

            elif key in (ord("s"), ord("S"), _KEY_DOWN):
                new = angle + step
                new = min(HEAD_TILT_MAX, new)
                try:
                    bot.head.tilt(new)
                    angle = new
                    print(f"  tilt {angle:+.3f} rad ({math.degrees(angle):+.1f}°)")
                except ValueError as e:
                    print(f"  {e}")

            elif key in (ord("r"), ord("R")):
                bot.head.tilt(0.0)
                angle = 0.0
                print("  reset → 0.000 rad (0.0°)")

            elif key == ord("["):
                step = max(_STEP_MIN, step / 2)
                print(f"  step → {step:.3f} rad ({math.degrees(step):.1f}°)")

            elif key == ord("]"):
                step = min(_STEP_MAX, step * 2)
                print(f"  step → {step:.3f} rad ({math.degrees(step):.1f}°)")

            elif key in (ord("t"), ord("T")):
                waiting_for_type = True
                print(f"  Type angle in radians (range [{HEAD_TILT_MIN:.3f}, {HEAD_TILT_MAX:.3f}]): ", end="", flush=True)

            # ── terminal input ────────────────────────────────────────────
            try:
                line = stdin_q.get_nowait()
            except queue.Empty:
                line = None

            if line == "__quit__":
                break

            if line is not None and waiting_for_type:
                waiting_for_type = False
                if line:
                    try:
                        new = float(line)
                        bot.head.tilt(new)
                        angle = new
                        print(f"  tilt {angle:+.3f} rad ({math.degrees(angle):+.1f}°)")
                    except ValueError as e:
                        print(f"  {e}")
                else:
                    print("  (cancelled)")

            # guard against window close button
            if cv2.getWindowProperty(_WIN_TITLE, cv2.WND_PROP_VISIBLE) < 1:
                break

    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        print(f"\nFinal angle: {angle:+.3f} rad ({math.degrees(angle):+.1f}°)")


# ── Main ───────────────────────────────────────────────────────────────────


def main():
    parser = argparse.ArgumentParser(
        description="Interactive head tilt test with camera preview"
    )
    parser.add_argument("--ip",        default="127.0.0.1", help="Robot IP (default: 127.0.0.1)")
    parser.add_argument("--port",      type=int,   default=9090,        help="Rosbridge port (default: 9090)")
    parser.add_argument("--cam-port",  type=int,   default=7447,        help="Zenoh camera port (default: 7447)")
    parser.add_argument("--protocol",  default="rosbridge", choices=["rosbridge", "zenoh"], help="ROS protocol")
    parser.add_argument("--namespace", default="",          help="ROS namespace (default: none)")
    parser.add_argument("--step",      type=float, default=_STEP_DEFAULT, help=f"Tilt step in radians (default: {_STEP_DEFAULT})")
    parser.add_argument("--no-camera", action="store_true",  help="Disable camera — terminal-only HUD")
    args = parser.parse_args()

    camera_protocol = "none" if args.no_camera else "zenoh"

    print(f"\nConnecting to {args.ip}:{args.port}  (ROS={args.protocol}, camera={camera_protocol}) ...")
    bot = WalkieRobot(
        ip=args.ip,
        ros_protocol=args.protocol,
        ros_port=args.port,
        camera_protocol=camera_protocol,
        camera_port=args.cam_port,
        namespace=args.namespace,
    )
    print("Connected.")
    print(f"  tilt range : [{HEAD_TILT_MIN:.3f}, {HEAD_TILT_MAX:.3f}] rad  (±45°)")
    print(f"  step       : {args.step:.3f} rad  ({math.degrees(args.step):.1f}°)")

    try:
        run(bot, step=args.step, camera_enabled=not args.no_camera)
    finally:
        bot.disconnect()
        print("Disconnected.")


if __name__ == "__main__":
    main()
