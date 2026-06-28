"""
Interactive hardware test for Navigation.set_velocity() — direct cmd_vel publish
(geometry_msgs/msg/TwistStamped, open-loop teleop-style drive).

WARNING: these commands MOVE THE ROBOT. Keep the area clear, start with small
speeds, and be ready to Ctrl-C — interrupting (or quitting) sends stop().

Two control modes:

  1. Keyboard "game" mode (default on launch) — real-time WASD + Q/E:
         W / S   forward / backward        A / D   strafe left / right
         Q / E   turn left / right         SPACE   stop
         X / ESC exit to the typed prompt  Ctrl-C  quit
     Hold a key to keep driving; release and the robot auto-stops after a
     short timeout (terminals send no key-up event, so this relies on the OS
     key-repeat keeping the key "down"). Fixed speed: 0.1 m/s, 0.1 rad/s.

  2. Typed-command prompt (X/ESC drops into it; 'wasd' re-enters game mode) —
     precise one-shot / timed velocities, see the prompt help.

set_velocity() publishes a SINGLE TwistStamped per call. Most base controllers
run a cmd_vel watchdog (~0.5 s), so one publish only produces a brief twitch;
sustained motion requires continuous publishing, which both modes do.

Requires: rosbridge + a base controller subscribing to cmd_vel (TwistStamped).

Usage:
    python tests/test_velocity_interactive.py --ip 192.168.1.100
    python tests/test_velocity_interactive.py --ip 192.168.1.100 --namespace robot1
    python tests/test_velocity_interactive.py --ip 192.168.1.100 --speed 0.1 --turn 0.1
"""

import argparse
import select
import sys
import termios
import time
import tty

from walkie_sdk import WalkieRobot


# Real-time keyboard teleop tuning.
KEY_TIMEOUT = 0.4   # zero velocity if no key seen within this window (release-to-stop)
PUB_PERIOD = 0.05   # publish / poll period in seconds (20 Hz)


# ── Helpers ────────────────────────────────────────────────────────────────


def _section(title: str) -> None:
    print(f"\n{'=' * 60}")
    print(f"  {title}")
    print("=" * 60)


def _pass(msg: str) -> None:
    print(f"  [PASS] {msg}")


def _fail(msg: str) -> None:
    print(f"  [FAIL] {msg}")


# ── Keyboard "game" mode ────────────────────────────────────────────────────


def run_keyboard_teleop(bot: WalkieRobot, speed: float, turn: float) -> int:
    """
    Real-time WASD + Q/E teleop. Returns the number of set_velocity publishes.

    Drives the robot at a fixed ``speed`` (m/s) / ``turn`` (rad/s) for whichever
    key is currently held. Publishes at ``1 / PUB_PERIOD`` Hz; if no key arrives
    within ``KEY_TIMEOUT`` the velocity is zeroed (release-to-stop). Restores the
    terminal and sends stop() on every exit path (X/ESC, Ctrl-C, error).
    """
    if not sys.stdin.isatty():
        print("  stdin is not a TTY — keyboard mode needs an interactive terminal.")
        return 0

    key_vel = {
        "w": (speed, 0.0, 0.0),
        "s": (-speed, 0.0, 0.0),
        "a": (0.0, speed, 0.0),    # strafe left  (+y)
        "d": (0.0, -speed, 0.0),   # strafe right (-y)
        "q": (0.0, 0.0, turn),     # turn left    (CCW, +z)
        "e": (0.0, 0.0, -turn),    # turn right   (CW,  -z)
    }

    _section("Keyboard Teleop  (hold to drive)")
    print()
    print("    W / S   forward / backward       A / D   strafe left / right")
    print("    Q / E   turn left / right        SPACE   stop")
    print("    X / ESC exit to typed prompt     Ctrl-C  quit")
    print()
    print(f"  fixed speed = {speed} m/s   turn = {turn} rad/s   "
          f"(release ~{KEY_TIMEOUT}s -> auto-stop)")
    print()

    fd = sys.stdin.fileno()
    old_attrs = termios.tcgetattr(fd)
    vx = vy = wz = 0.0
    last_key = time.monotonic()
    n = 0
    try:
        tty.setcbreak(fd)  # cbreak keeps ISIG so Ctrl-C still interrupts
        while True:
            ready, _, _ = select.select([sys.stdin], [], [], PUB_PERIOD)
            if ready:
                ch = sys.stdin.read(1)
                if ch == "\x03":               # Ctrl-C
                    raise KeyboardInterrupt
                if ch == "\x1b":               # ESC, or start of an arrow-key sequence
                    more, _, _ = select.select([sys.stdin], [], [], 0.0)
                    if more:
                        sys.stdin.read(2)       # drain & ignore arrow keys
                        continue
                    break                       # bare ESC -> exit
                if ch in ("x", "X"):
                    break
                if ch == " ":
                    vx = vy = wz = 0.0
                    last_key = time.monotonic()
                else:
                    mapped = key_vel.get(ch.lower())
                    if mapped is not None:
                        vx, vy, wz = mapped
                        last_key = time.monotonic()

            if time.monotonic() - last_key > KEY_TIMEOUT:
                vx = vy = wz = 0.0

            bot.nav.set_velocity(vx, vy, wz)
            n += 1
            print(f"\r  vx={vx:+.2f}  vy={vy:+.2f}  wz={wz:+.2f}    "
                  f"[WASD move | Q/E turn | SPACE stop | X/ESC exit]   ",
                  end="", flush=True)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_attrs)
        bot.nav.stop()
        print("\n  (keyboard mode stopped)")
    return n


# ── Typed-command mode ──────────────────────────────────────────────────────


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
    Parse a typed-mode input string. Returns one of:
      ("game",)
      ("vel",   vx, vy, wz)
      ("drive", vx, vy, wz, secs)
      ("stop",)
      ("speed", mps) / ("turn", rps) / ("rate", hz)
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

    if cmd in ("wasd", "game", "teleop", "key", "keys"):
        return ("game",)

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

    if cmd in ("speed", "turn", "rate"):
        if len(parts) != 2:
            print(f"  Usage: {cmd} <value>")
            return None
        got = _floats([parts[1]], cmd)
        if not got:
            return None
        return (cmd, got[0])

    return ("unknown", raw)


def run_interactive_session(bot: WalkieRobot, speed: float, turn: float,
                            rate: float, duration: float) -> tuple[int, int]:
    """Typed-command velocity prompt. Returns (passed, attempted)."""
    _section("Typed Velocity Commands")
    print()
    print("  Commands (linear m/s, angular rad/s; +x fwd, +y left, +z yaw CCW):")
    print("    wasd                        enter real-time keyboard game mode")
    print("    vel <vx> <vy> <wz>          single set_velocity() publish")
    print("    drive <vx> <vy> <wz> [secs] publish at rate Hz for secs, then stop")
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

        if kind == "game":
            run_keyboard_teleop(bot, speed, turn)
            continue

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
            print("  Commands: wasd | vel | drive | stop | speed/turn/rate | status | q")
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

        if kind == "drive":
            _, vx, vy, wz, secs = parsed
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
        description="Interactive hardware test for Navigation.set_velocity() (WASD keyboard teleop + typed commands)"
    )
    parser.add_argument("--ip",        default="127.0.0.1", help="Robot IP (default: 127.0.0.1)")
    parser.add_argument("--port",      type=int,   default=9090, help="Rosbridge port (default: 9090)")
    parser.add_argument("--namespace", default="", help="ROS namespace (default: none)")
    parser.add_argument("--speed",     type=float, default=0.2, help="Linear speed, m/s (default: 0.1)")
    parser.add_argument("--turn",      type=float, default=0.3, help="Angular speed, rad/s (default: 0.1)")
    parser.add_argument("--rate",      type=float, default=20.0, help="Continuous publish rate, Hz (default: 20)")
    parser.add_argument("--duration",  type=float, default=2.0,  help="Default 'drive' duration when secs omitted (default: 2.0)")
    parser.add_argument("--no-game",   action="store_true", help="Skip keyboard mode; start at the typed prompt")
    args = parser.parse_args()

    print(f"\nConnecting to rosbridge at {args.ip}:{args.port} ...")
    bot = WalkieRobot(
        ip=args.ip,
        ros_protocol="rosbridge",
        camera_protocol="none",
        namespace=args.namespace,
    )
    print("Connected.")

    kb_pubs = 0
    passed = attempted = 0
    try:
        # Start in the keyboard "game" by default; X/ESC drops into the typed prompt.
        if not args.no_game:
            kb_pubs = run_keyboard_teleop(bot, args.speed, args.turn)
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
    print(f"  keyboard publishes: {kb_pubs}")
    if attempted == 0:
        print("  typed commands: none attempted.")
    else:
        print(f"  typed commands: {passed}/{attempted} OK")
    print("=" * 60)
    sys.exit(0 if (attempted == 0 or passed == attempted) else 1)


if __name__ == "__main__":
    main()
