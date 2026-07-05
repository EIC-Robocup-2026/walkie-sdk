"""
Interactive hardware test for Navigation (navigate_to_object and navigate_to_pose).

Lets you type goals at runtime — useful for real-life testing where you want
to navigate to arbitrary object positions and observe the approach behaviour.

Requires: rosbridge + Nav2 + nav_commander /navigate_to_object action.

Usage:
    python tests/test_navigation_interactive.py --ip 192.168.1.100
    python tests/test_navigation_interactive.py --ip 192.168.1.100 --namespace robot1

At the prompt type:
    go <x> <y>                  → navigate_to_object (no heading, edge-fit approach)
    go <x> <y> <standoff>       → navigate_to_object with standoff override (m)
    go <x> <y> face_target      → navigate_to_object, skip edge fit
    pose <x> <y> <heading>      → navigate_to_pose (Nav2, heading in radians)
    precise on|off              → select Nav2 goal checker for next goals
                                  (on = precise_goal_checker, off = general)
    autotilt on|off             → enable/disable head_tilt_near_goal auto-tilt node
    cancel                      → cancel current navigation goal
    stop                        → emergency stop (zero cmd_vel + cancel)
    status                      → print current navigation status
    where                       → print latest robot pose (nav.current_pose)
    q / quit                    → exit
"""

import argparse
import sys

from walkie_sdk import WalkieRobot


# ── Helpers ────────────────────────────────────────────────────────────────


def _section(title: str) -> None:
    print(f"\n{'=' * 60}")
    print(f"  {title}")
    print("=" * 60)


def _pass(msg: str) -> None:
    print(f"  [PASS] {msg}")


def _fail(msg: str) -> None:
    print(f"  [FAIL] {msg}")


def _print_status(bot: WalkieRobot) -> None:
    print(f"  status={bot.nav.status}  navigating={bot.nav.is_navigating}")
    if bot.nav.distance_remaining is not None:
        print(f"  distance_remaining={bot.nav.distance_remaining:.3f} m")
    if bot.nav.nav_error_msg:
        print(f"  last_message={bot.nav.nav_error_msg!r}")


def _print_pose(bot: WalkieRobot) -> None:
    pose = bot.nav.current_pose
    if not pose:
        print("  current_pose=None  (no Nav2 feedback received yet)")
        return
    p = pose.get("pose", pose)
    pos = p.get("position", {})
    ori = p.get("orientation", {})
    frame = pose.get("header", {}).get("frame_id", "?")
    print(f"  frame={frame}")
    print(f"  position=(x={pos.get('x')}, y={pos.get('y')}, z={pos.get('z')})")
    print(f"  orientation=(x={ori.get('x')}, y={ori.get('y')}, "
          f"z={ori.get('z')}, w={ori.get('w')})")


def _parse_command(raw: str):
    """
    Parse a user input string. Returns one of:
      ("go_obj",  x, y, standoff, align_method)
      ("go_pose", x, y, heading)
      ("precise", enabled)
      ("cancel",)
      ("stop",)
      ("status",)
      ("where",)
      ("quit",)
      ("unknown", original_text)
    """
    parts = raw.split()
    if not parts:
        return None

    cmd = parts[0].lower()

    if cmd in ("q", "quit", "exit"):
        return ("quit",)

    if cmd in ("cancel", "c"):
        return ("cancel",)

    if cmd in ("stop", "s"):
        return ("stop",)

    if cmd in ("status", "st"):
        return ("status",)

    if cmd in ("where", "pose?", "w"):
        return ("where",)

    if cmd == "go":
        if len(parts) < 3:
            print("  Usage: go <x> <y> [<standoff> | face_target]")
            return None
        try:
            x = float(parts[1])
            y = float(parts[2])
        except ValueError:
            print(f"  Invalid coordinates: {parts[1]!r} {parts[2]!r}")
            return None

        standoff = 0.0
        align_method = ""
        if len(parts) >= 4:
            if parts[3].lower() == "face_target":
                align_method = "face_target"
            else:
                try:
                    standoff = float(parts[3])
                except ValueError:
                    print(f"  Invalid standoff {parts[3]!r} — use a number or 'face_target'")
                    return None
        return ("go_obj", x, y, standoff, align_method)

    if cmd == "autotilt":
        if len(parts) < 2 or parts[1].lower() not in ("on", "off"):
            print("  Usage: autotilt on|off")
            return None
        return ("autotilt", parts[1].lower() == "on")

    if cmd == "precise":
        if len(parts) < 2 or parts[1].lower() not in ("on", "off"):
            print("  Usage: precise on|off")
            return None
        return ("precise", parts[1].lower() == "on")

    if cmd == "pose":
        if len(parts) < 4:
            print("  Usage: pose <x> <y> <heading_rad>")
            return None
        try:
            x = float(parts[1])
            y = float(parts[2])
            h = float(parts[3])
        except ValueError:
            print(f"  Invalid pose args: {parts[1:]}")
            return None
        return ("go_pose", x, y, h)

    return ("unknown", raw)


# ── Interactive session ────────────────────────────────────────────────────


def run_interactive_session(bot: WalkieRobot, timeout: float) -> tuple[int, int]:
    """Prompt the user for navigation goals until they quit. Returns (passed, attempted)."""
    _section("Interactive Navigation Control")
    print()
    print("  Commands:")
    print("    go <x> <y>                 navigate_to_object  (no heading, edge-fit)")
    print("    go <x> <y> <standoff>      navigate_to_object  with standoff override (m)")
    print("    go <x> <y> face_target     navigate_to_object  skip edge alignment")
    print("    pose <x> <y> <heading>     navigate_to_pose    (Nav2, heading in rad)")
    print("    precise on|off             goal checker for next goals (on = precise_goal_checker)")
    print("    autotilt on|off            enable/disable head_tilt_near_goal node")
    print("    cancel                     cancel current goal")
    print("    stop                       emergency stop (zero cmd_vel + cancel)")
    print("    status                     print current nav status")
    print("    where                      print latest robot pose (nav.current_pose)")
    print("    q / quit                   exit")
    print()

    passed = 0
    attempted = 0
    precise = False

    while True:
        try:
            raw = input("  nav> ").strip()
        except (KeyboardInterrupt, EOFError):
            print()
            break

        if not raw:
            continue

        parsed = _parse_command(raw)
        if parsed is None:
            continue

        kind = parsed[0]

        if kind == "quit":
            break

        if kind == "cancel":
            ok = bot.nav.cancel()
            print(f"  cancel() -> {ok}  |  status: {bot.nav.status}")
            continue

        if kind == "stop":
            ok = bot.nav.stop()
            print(f"  stop() -> {ok}  |  status: {bot.nav.status}")
            continue

        if kind == "status":
            _print_status(bot)
            print(f"  precise={'on' if precise else 'off'}")
            continue

        if kind == "where":
            _print_pose(bot)
            continue

        if kind == "autotilt":
            enable = parsed[1]
            ok = bot.head.set_auto_tilt(enable)
            state = "enabled" if enable else "disabled"
            print(f"  set_auto_tilt({enable}) -> {ok}  ({state})")
            continue

        if kind == "precise":
            precise = parsed[1]
            checker = "precise_goal_checker" if precise else "general_goal_checker"
            print(f"  precise={'on' if precise else 'off'}  (next goals use {checker})")
            continue

        if kind == "unknown":
            print(f"  Unknown command: {parsed[1]!r}")
            print("  Commands: go | pose | precise | autotilt | cancel | stop | status | where | q")
            continue

        if kind == "go_obj":
            _, x, y, standoff, align_method = parsed
            label = f"navigate_to_object  x={x}  y={y}  standoff={standoff}  align={align_method or 'nearest_edge'}"
            print(f"  → {label}  precise={'on' if precise else 'off'}  (blocking, timeout={timeout}s) ...")
            attempted += 1

            result = bot.nav.go_to(x=x, y=y, standoff=standoff, align_method=align_method,
                                   precise=precise, blocking=True, timeout=timeout)
            print(f"  Result: {result}  |  method: {bot.nav.nav_error_msg}")

        elif kind == "go_pose":
            _, x, y, h = parsed
            print(f"  → navigate_to_pose  x={x}  y={y}  heading={h:.3f} rad  "
                  f"precise={'on' if precise else 'off'}  (blocking, timeout={timeout}s) ...")
            attempted += 1

            result = bot.nav.go_to(x=x, y=y, heading=h, precise=precise,
                                   blocking=True, timeout=timeout)
            print(f"  Result: {result}  |  status: {bot.nav.status}")

        if result in ("SUCCEEDED", "CLOSE_ENOUGH"):
            _pass(result)
            passed += 1
        else:
            _fail(f"Expected SUCCEEDED/CLOSE_ENOUGH, got {result}")

    return passed, attempted


# ── Main ───────────────────────────────────────────────────────────────────


def main():
    parser = argparse.ArgumentParser(
        description="Interactive hardware test for Navigation (navigate_to_object / navigate_to_pose)"
    )
    parser.add_argument("--ip",        default="127.0.0.1", help="Robot IP (default: 127.0.0.1)")
    parser.add_argument("--port",      type=int,   default=9090, help="Rosbridge port (default: 9090)")
    parser.add_argument("--timeout",   type=float, default=60.0, help="Max seconds per blocking move (default: 60.0)")
    parser.add_argument("--namespace", default="", help="ROS namespace (default: none)")
    args = parser.parse_args()

    print(f"\nConnecting to rosbridge at {args.ip}:{args.port} ...")
    bot = WalkieRobot(
        ip=args.ip,
        ros_protocol="rosbridge",
        camera_protocol="none",
        namespace=args.namespace,
    )
    print("Connected.")
    print(f"  timeout = {args.timeout}s")

    passed = attempted = 0
    try:
        passed, attempted = run_interactive_session(bot, args.timeout)
    except KeyboardInterrupt:
        print("\nInterrupted by user — sending stop().")
        try:
            bot.nav.stop()
        except Exception:
            pass
    finally:
        bot.disconnect()
        print("\nDisconnected.")

    print(f"\n{'=' * 60}")
    if attempted == 0:
        print("  No goals attempted.")
    else:
        print(f"  Goals: {passed}/{attempted} SUCCEEDED")
    print("=" * 60)
    sys.exit(0 if (attempted == 0 or passed == attempted) else 1)


if __name__ == "__main__":
    main()
