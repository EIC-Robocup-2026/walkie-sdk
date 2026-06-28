"""
Interactive hardware test for Navigation.set_velocity() — direct cmd_vel publish
(geometry_msgs/msg/TwistStamped, open-loop teleop-style drive).

WARNING: these commands MOVE THE ROBOT. Keep the area clear, start with small
speeds, and be ready to Ctrl-C — interrupting (or quitting) sends stop().

set_velocity() publishes a SINGLE TwistStamped per call. Most base controllers
run a cmd_vel watchdog (~0.5 s), so one publish only produces a brief twitch; to
keep moving you must publish continuously. This script offers both:

    vel   — one raw set_velocity() publish (tests the call itself)
    drive — publish at <rate> Hz for <secs>, then stop() (sustained motion)

Requires: rosbridge + a base controller subscribing to cmd_vel (TwistStamped).

Usage:
    python tests/test_velocity_interactive.py --ip 192.168.1.100
    python tests/test_velocity_interactive.py --ip 192.168.1.100 --namespace robot1
    python tests/test_velocity_interactive.py --ip 192.168.1.100 --speed 0.15 --rate 20

At the prompt type (linear m/s, angular rad/s; +x forward, +y left, +z yaw CCW):
    vel <vx> <vy> <wz>            → single set_velocity() publish (one TwistStamped)
    drive <vx> <vy> <wz> [secs]   → publish at <rate> Hz for <secs>, then stop
    f [secs] / b [secs]           → drive forward / backward at --speed
    l [secs] / r [secs]           → strafe left / right at --speed
    ccw [secs] / cw [secs]        → rotate left / right at --turn
    stop                          → emergency stop (zero cmd_vel + cancel)
    speed <mps>                   → set default linear speed for shortcuts
    turn <rps>                    → set default angular speed for shortcuts
    rate <hz>                     → set continuous publish rate for drive
    status                        → print nav.status
    q / quit                      → stop and exit
"""

import argparse
import sys
import time

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


def _drive_for(bot: WalkieRobot, vx: float, vy: float, wz: float,
               secs: float, rate: float) -> tuple[bool, int]:
    """
    Publish set_velocity() at ``rate`` Hz for ``secs`` seconds, then stop().

    Returns (ok, n_publishes). ``ok`` is False if any publish returned False.
    stop() is always sent at the end, including on Ctrl-C / exceptions.
    """
    period = 1.0 / rate if rate > 0 else 0.05
    deadline = time.monotonic() + secs
    n = 0
    ok = True
    try:
        while time.monotonic() < deadline:
            if not bot.nav.set_velocity(vx, vy, wz):
                ok = False
                break
            n += 1
            time.sleep(period)
    finally:
        bot.nav.stop()
    return ok, n


def _parse_command(raw: str):
    """
    Parse a user input string. Returns one of:
      ("vel",   vx, vy, wz)
      ("drive", vx, vy, wz, secs)
      ("stop",)
      ("speed", mps)
      ("turn",  rps)
      ("rate",  hz)
      ("status",)
      ("quit",)
      ("unknown", original_text)
    or None on a usage/parse error (message already printed).
    """
    parts = raw.split()
    if not parts:
        return None

    cmd = parts[0].lower()

    if cmd in ("q", "quit", "exit"):
        return ("quit",)

    if cmd in ("stop", "s"):
        return ("stop",)

    if cmd in ("status", "st"):
        return ("status",)

    def _floats(tokens, label):
        try:
            return [float(t) for t in tokens]
        except ValueError:
            print(f"  Invalid {label}: {' '.join(tokens)!r}")
            return None

    if cmd == "vel":
        if len(parts) != 4:
            print("  Usage: vel <vx> <vy> <wz>")
            return None
        vals = _floats(parts[1:4], "velocity")
        return ("vel", *vals) if vals else None

    if cmd == "drive":
        if len(parts) not in (4, 5):
            print("  Usage: drive <vx> <vy> <wz> [secs]")
            return None
        vals = _floats(parts[1:], "drive args")
        if not vals:
            return None
        vx, vy, wz = vals[0], vals[1], vals[2]
        secs = vals[3] if len(vals) == 4 else None  # None → caller's default
        return ("drive", vx, vy, wz, secs)

    # Direction shortcuts: <dir> [secs]
    shortcuts = {"f": "fwd", "b": "back", "l": "left", "r": "right",
                 "ccw": "ccw", "cw": "cw"}
    if cmd in shortcuts:
        secs = None
        if len(parts) >= 2:
            got = _floats([parts[1]], "duration")
            if not got:
                return None
            secs = got[0]
        return ("shortcut", shortcuts[cmd], secs)

    if cmd in ("speed", "turn", "rate"):
        if len(parts) != 2:
            print(f"  Usage: {cmd} <value>")
            return None
        got = _floats([parts[1]], cmd)
        if not got:
            return None
        return (cmd, got[0])

    return ("unknown", raw)


# ── Interactive session ────────────────────────────────────────────────────


def run_interactive_session(bot: WalkieRobot, speed: float, turn: float,
                            rate: float, duration: float) -> tuple[int, int]:
    """Drive the robot with velocity commands until the user quits. Returns (passed, attempted)."""
    _section("Interactive Velocity Control (set_velocity / cmd_vel)")
    print()
    print("  WARNING: these commands move the robot. Keep clear; Ctrl-C stops.")
    print()
    print("  Commands (linear m/s, angular rad/s; +x fwd, +y left, +z yaw CCW):")
    print("    vel <vx> <vy> <wz>          single set_velocity() publish")
    print("    drive <vx> <vy> <wz> [secs] publish at rate Hz for secs, then stop")
    print("    f|b [secs]                  forward / backward at speed")
    print("    l|r [secs]                  strafe left / right at speed")
    print("    ccw|cw [secs]               rotate left / right at turn")
    print("    stop                        emergency stop")
    print("    speed|turn|rate <v>         change defaults")
    print("    status                      print nav.status")
    print("    q / quit                    stop and exit")
    print()
    print(f"  defaults: speed={speed} m/s  turn={turn} rad/s  "
          f"rate={rate} Hz  duration={duration} s")
    print()

    passed = 0
    attempted = 0

    while True:
        try:
            raw = input("  vel> ").strip()
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

        if kind == "stop":
            ok = bot.nav.stop()
            print(f"  stop() -> {ok}  |  status: {bot.nav.status}")
            continue

        if kind == "status":
            print(f"  status={bot.nav.status}")
            continue

        if kind == "speed":
            speed = parsed[1]
            print(f"  speed = {speed} m/s")
            continue

        if kind == "turn":
            turn = parsed[1]
            print(f"  turn = {turn} rad/s")
            continue

        if kind == "rate":
            rate = parsed[1]
            print(f"  rate = {rate} Hz")
            continue

        if kind == "unknown":
            print(f"  Unknown command: {parsed[1]!r}")
            print("  Commands: vel | drive | f/b/l/r/ccw/cw | stop | "
                  "speed/turn/rate | status | q")
            continue

        if kind == "vel":
            _, vx, vy, wz = parsed
            attempted += 1
            ok = bot.nav.set_velocity(vx, vy, wz)
            print(f"  set_velocity(vx={vx}, vy={vy}, wz={wz}) -> {ok}")
            print("  (one-shot publish — base watchdog may halt soon; send 'stop' to be safe)")
            if ok:
                _pass("set_velocity returned True")
                passed += 1
            else:
                _fail("set_velocity returned False (connected?)")
            continue

        # Resolve drive / shortcut into (vx, vy, wz, secs)
        if kind == "drive":
            _, vx, vy, wz, secs = parsed
        else:  # shortcut
            _, direction, secs = parsed
            vx = vy = wz = 0.0
            if direction == "fwd":
                vx = speed
            elif direction == "back":
                vx = -speed
            elif direction == "left":
                vy = speed
            elif direction == "right":
                vy = -speed
            elif direction == "ccw":
                wz = turn
            elif direction == "cw":
                wz = -turn

        if secs is None:
            secs = duration

        attempted += 1
        print(f"  → drive vx={vx} vy={vy} wz={wz} for {secs}s @ {rate}Hz, then stop ...")
        ok, n = _drive_for(bot, vx, vy, wz, secs, rate)
        print(f"  done: {n} publishes, stopped  |  ok={ok}  status={bot.nav.status}")
        if ok and n > 0:
            _pass(f"{n} set_velocity publishes succeeded")
            passed += 1
        else:
            _fail("a set_velocity publish failed during drive")

    return passed, attempted


# ── Main ───────────────────────────────────────────────────────────────────


def main():
    parser = argparse.ArgumentParser(
        description="Interactive hardware test for Navigation.set_velocity() (direct cmd_vel)"
    )
    parser.add_argument("--ip",        default="127.0.0.1", help="Robot IP (default: 127.0.0.1)")
    parser.add_argument("--port",      type=int,   default=9090, help="Rosbridge port (default: 9090)")
    parser.add_argument("--namespace", default="", help="ROS namespace (default: none)")
    parser.add_argument("--speed",     type=float, default=0.15, help="Default linear speed for shortcuts, m/s (default: 0.15)")
    parser.add_argument("--turn",      type=float, default=0.5,  help="Default angular speed for shortcuts, rad/s (default: 0.5)")
    parser.add_argument("--rate",      type=float, default=20.0, help="Continuous publish rate for drive, Hz (default: 20)")
    parser.add_argument("--duration",  type=float, default=2.0,  help="Default drive duration when secs omitted (default: 2.0)")
    args = parser.parse_args()

    print(f"\nConnecting to rosbridge at {args.ip}:{args.port} ...")
    bot = WalkieRobot(
        ip=args.ip,
        ros_protocol="rosbridge",
        camera_protocol="none",
        namespace=args.namespace,
    )
    print("Connected.")

    passed = attempted = 0
    try:
        passed, attempted = run_interactive_session(
            bot, args.speed, args.turn, args.rate, args.duration
        )
    except KeyboardInterrupt:
        print("\nInterrupted by user — sending stop().")
    finally:
        try:
            bot.nav.stop()  # always leave the robot stopped
        except Exception:
            pass
        bot.disconnect()
        print("\nDisconnected.")

    print(f"\n{'=' * 60}")
    if attempted == 0:
        print("  No velocity commands attempted.")
    else:
        print(f"  Commands: {passed}/{attempted} OK")
    print("=" * 60)
    sys.exit(0 if (attempted == 0 or passed == attempted) else 1)


if __name__ == "__main__":
    main()
