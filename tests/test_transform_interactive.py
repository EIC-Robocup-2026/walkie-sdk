"""
Interactive TF transform tester — real-time frame lookups against a live robot.

Lets you query, watch, and visualize transforms between any two TF frames.

Controls (main menu):
  1  lookup     — one-shot lookup (prompts for source / target frame)
  2  watch      — continuous polling of a frame pair (Ctrl+C to stop)
  3  presets    — pick from common frame pairs with a single keypress
  v  viz        — toggle RViz2 marker publishing for the next lookup
  q  quit

Usage:
    python tests/test_transform_interactive.py
    python tests/test_transform_interactive.py --ip 192.168.1.100
    python tests/test_transform_interactive.py --ip 192.168.1.100 --port 9090
    python tests/test_transform_interactive.py --ip 192.168.1.100 --namespace robot1
    python tests/test_transform_interactive.py --ip 192.168.1.100 --timeout 3.0
"""

import argparse
import math
import time

from walkie_sdk import WalkieRobot, SPHERE, ARROW, TEXT_VIEW_FACING


# ── Common frame-pair presets ──────────────────────────────────────────────

_PRESETS = [
    ("map",         "base_link"),
    ("map",         "base_footprint"),
    ("odom",        "base_link"),
    ("base_link",   "camera_link"),
    ("base_link",   "camera_color_optical_frame"),
    ("map",         "odom"),
]


# ── Helpers ────────────────────────────────────────────────────────────────

def _section(title: str) -> None:
    print(f"\n{'─' * 55}")
    print(f"  {title}")
    print("─" * 55)


def _prompt(prompt: str, default: str = "") -> str:
    suffix = f" [{default}]" if default else ""
    raw = input(f"  {prompt}{suffix}: ").strip()
    return raw if raw else default


def _prompt_float(prompt: str, default: float) -> float:
    raw = input(f"  {prompt} [{default}]: ").strip()
    try:
        return float(raw) if raw else default
    except ValueError:
        return default


def _euler_from_quat(x: float, y: float, z: float, w: float):
    """Return (roll, pitch, yaw) in degrees from a quaternion."""
    # roll
    sinr = 2.0 * (w * x + y * z)
    cosr = 1.0 - 2.0 * (x * x + y * y)
    roll = math.degrees(math.atan2(sinr, cosr))
    # pitch
    sinp = 2.0 * (w * y - z * x)
    pitch = math.degrees(math.asin(max(-1.0, min(1.0, sinp))))
    # yaw
    siny = 2.0 * (w * z + x * y)
    cosy = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.degrees(math.atan2(siny, cosy))
    return roll, pitch, yaw


def _print_transform(result: dict, source: str, target: str) -> None:
    pos = result["position"]
    quat = result["quaternion"]
    roll, pitch, yaw = _euler_from_quat(quat["x"], quat["y"], quat["z"], quat["w"])
    print(f"  {source}  →  {target}")
    print(f"    pos  : x={pos['x']:+.4f}  y={pos['y']:+.4f}  z={pos['z']:+.4f}  (m)")
    print(f"    quat : x={quat['x']:+.4f}  y={quat['y']:+.4f}  z={quat['z']:+.4f}  w={quat['w']:+.4f}")
    print(f"    euler: roll={roll:+.2f}°  pitch={pitch:+.2f}°  yaw={yaw:+.2f}°")


def _do_viz(robot: WalkieRobot, result: dict, source: str, target: str,
            marker_ids: list) -> None:
    """Publish / refresh a sphere + arrow + label in RViz2."""
    for mid in marker_ids:
        try:
            robot.viz.delete_marker(mid)
        except Exception:
            pass
    marker_ids.clear()

    pos = result["position"]
    quat = result["quaternion"]
    position = [pos["x"], pos["y"], pos["z"]]
    quaternion = [quat["x"], quat["y"], quat["z"], quat["w"]]

    sphere_id = robot.draw_marker(
        position=position,
        quaternion=quaternion,
        marker_type=SPHERE,
        color=[0.0, 1.0, 0.5, 1.0],
        scale=[0.10, 0.10, 0.10],
        frame_id=source,
        ns="transform_viz",
    )
    arrow_id = robot.draw_marker(
        position=position,
        quaternion=quaternion,
        marker_type=ARROW,
        color=[1.0, 0.5, 0.0, 1.0],
        scale=[0.25, 0.04, 0.04],
        frame_id=source,
        ns="transform_viz_arrow",
    )
    label = f"{target} ({pos['x']:.2f}, {pos['y']:.2f}, {pos['z']:.2f})"
    label_id = robot.draw_marker(
        position=[position[0], position[1], position[2] + 0.18],
        quaternion=[0.0, 0.0, 0.0, 1.0],
        marker_type=TEXT_VIEW_FACING,
        text=label,
        color=[1.0, 1.0, 1.0, 1.0],
        scale=[0.08, 0.08, 0.08],
        frame_id=source,
        ns="transform_viz_label",
    )
    marker_ids.extend([sphere_id, arrow_id, label_id])
    print(f"  [viz] markers published in frame '{source}'")


# ── Menu actions ───────────────────────────────────────────────────────────

def action_lookup(robot: WalkieRobot, timeout: float,
                  viz_on: bool, marker_ids: list,
                  last: dict) -> None:
    _section("Lookup transform")
    source = _prompt("source frame", last.get("source", "map"))
    target = _prompt("target frame", last.get("target", "base_link"))
    t = _prompt_float("timeout (s)", timeout)

    result = robot.transform.lookup(source, target, timeout=t)
    if result is None:
        print("  [FAIL] returned None — check frames and service")
        return

    _print_transform(result, source, target)
    last["source"] = source
    last["target"] = target

    if viz_on:
        _do_viz(robot, result, source, target, marker_ids)


def action_watch(robot: WalkieRobot, timeout: float,
                 viz_on: bool, marker_ids: list,
                 last: dict) -> None:
    _section("Watch mode  (Ctrl+C to stop)")
    source = _prompt("source frame", last.get("source", "map"))
    target = _prompt("target frame", last.get("target", "base_link"))
    interval = _prompt_float("poll interval (s)", 0.5)
    t = _prompt_float("timeout per call (s)", timeout)
    last["source"] = source
    last["target"] = target

    print(f"\n  Watching {source!r} → {target!r} every {interval:.2f}s  (Ctrl+C to stop)\n")
    try:
        while True:
            result = robot.transform.lookup(source, target, timeout=t)
            ts = time.strftime("%H:%M:%S")
            if result is None:
                print(f"  [{ts}] None")
            else:
                pos = result["position"]
                quat = result["quaternion"]
                _, _, yaw = _euler_from_quat(quat["x"], quat["y"], quat["z"], quat["w"])
                print(
                    f"  [{ts}]  x={pos['x']:+.4f}  y={pos['y']:+.4f}  z={pos['z']:+.4f}"
                    f"  yaw={yaw:+.2f}°"
                )
                if viz_on:
                    _do_viz(robot, result, source, target, marker_ids)
            time.sleep(interval)
    except KeyboardInterrupt:
        print("\n  Watch stopped.")


def action_presets(robot: WalkieRobot, timeout: float,
                   viz_on: bool, marker_ids: list,
                   last: dict) -> None:
    _section("Preset frame pairs")
    for i, (src, tgt) in enumerate(_PRESETS, 1):
        print(f"  {i}.  {src}  →  {tgt}")
    raw = input(f"\n  Choose [1-{len(_PRESETS)}]: ").strip()
    if not raw.isdigit() or not (1 <= int(raw) <= len(_PRESETS)):
        print("  Cancelled.")
        return
    source, target = _PRESETS[int(raw) - 1]
    last["source"] = source
    last["target"] = target

    result = robot.transform.lookup(source, target, timeout=timeout)
    if result is None:
        print("  [FAIL] returned None — check frames and service")
        return

    _print_transform(result, source, target)
    if viz_on:
        _do_viz(robot, result, source, target, marker_ids)


# ── Main loop ──────────────────────────────────────────────────────────────

_MENU = """
  ┌─────────────────────────────────────┐
  │  1  lookup    one-shot frame query  │
  │  2  watch     continuous polling    │
  │  3  presets   common frame pairs    │
  │  v  viz       toggle RViz2 markers  │
  │  q  quit                            │
  └─────────────────────────────────────┘"""


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Interactive TF transform tester"
    )
    parser.add_argument("--ip",        default="127.0.0.1", help="Robot IP (default: 127.0.0.1)")
    parser.add_argument("--port",      type=int, default=9090, help="Rosbridge port (default: 9090)")
    parser.add_argument("--timeout",   type=float, default=5.0, help="Default service timeout in s (default: 5.0)")
    parser.add_argument("--namespace", default="", help="ROS namespace (default: none)")
    parser.add_argument("--viz",       action="store_true", help="Start with RViz2 visualization on")
    args = parser.parse_args()

    print(f"\nConnecting to rosbridge at {args.ip}:{args.port} ...")
    robot = WalkieRobot(
        ros_protocol="rosbridge",
        ip=args.ip,
        ros_port=args.port,
        camera_protocol="none",
        namespace=args.namespace,
    )
    print("Connected.\n")
    print("  Requires: ros2 run walkie_tf tf_server on the robot.")

    viz_on = args.viz
    marker_ids: list = []
    last: dict = {}

    if viz_on:
        try:
            robot.viz.clear_markers()
        except Exception:
            pass
        print("  Visualization ON — open RViz2 and add a Marker display on 'walkie/viz_markers'.")

    try:
        while True:
            viz_label = "ON" if viz_on else "off"
            print(_MENU)
            print(f"  [viz={viz_label}]", end="  ")
            key = input("choice: ").strip().lower()

            if key in ("q", "quit"):
                break
            elif key in ("1", "l", "lookup"):
                action_lookup(robot, args.timeout, viz_on, marker_ids, last)
            elif key in ("2", "w", "watch"):
                action_watch(robot, args.timeout, viz_on, marker_ids, last)
            elif key in ("3", "p", "presets"):
                action_presets(robot, args.timeout, viz_on, marker_ids, last)
            elif key == "v":
                viz_on = not viz_on
                state = "ON" if viz_on else "off"
                print(f"  Visualization {state}.")
                if viz_on:
                    print("  Open RViz2 and add a Marker display on 'walkie/viz_markers'.")
            else:
                print("  Unknown command.")
    except KeyboardInterrupt:
        print("\nInterrupted.")
    finally:
        if marker_ids:
            try:
                for mid in marker_ids:
                    robot.viz.delete_marker(mid)
            except Exception:
                pass
        robot.disconnect()
        print("Disconnected.")


if __name__ == "__main__":
    main()
