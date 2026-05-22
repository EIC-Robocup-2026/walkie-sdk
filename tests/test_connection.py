"""
Hardware integration test for connection lifecycle, namespace handling, and config.

Exercises the WalkieRobot facade itself rather than a single robot capability:
  - connect / disconnect / is_connected (and idempotent disconnect)
  - context manager auto-disconnect
  - protocol + identity properties
  - namespace setter fan-out to nav/status/arm/lift/viz (+ re-subscription)
  - invalid protocol raises ValueError (offline, no socket opened)
  - legacy params (enable_camera=False, ws_port -> ros_port)
  - config_path YAML override of topic dictionaries (offline check via load_config)

Connection-dependent tests need a running rosbridge server; the offline checks
(invalid protocol, config YAML) run regardless and are reported separately.

Usage:
    python tests/test_connection.py
    python tests/test_connection.py --ip 192.168.1.100
    python tests/test_connection.py --ip 192.168.1.100 --port 9090
"""

import argparse
import os
import sys
import tempfile

from walkie_sdk import WalkieRobot


# ── Helpers ────────────────────────────────────────────────────────────────


def _section(title: str) -> None:
    print(f"\n{'=' * 60}")
    print(f"  {title}")
    print("=" * 60)


def _pass(msg: str) -> str:
    print(f"  [PASS] {msg}")
    return "PASS"


def _fail(msg: str) -> str:
    print(f"  [FAIL] {msg}")
    return "FAIL"


def _skip(msg: str) -> str:
    print(f"  [SKIP] {msg}")
    return "SKIP"


# ── Offline tests (no robot required) ────────────────────────────────────────


def test_invalid_protocol() -> str:
    _section("TEST 1: invalid ros_protocol raises ValueError (offline)")
    try:
        WalkieRobot(ip="127.0.0.1", ros_protocol="not_a_protocol")
    except ValueError as e:
        return _pass(f"ValueError raised as expected: {e}")
    except Exception as e:
        return _fail(f"expected ValueError, got {type(e).__name__}: {e}")
    return _fail("no exception raised for invalid protocol")


def test_config_yaml_override() -> str:
    _section("TEST 2: config_path YAML overrides topic dicts (offline)")
    from walkie_sdk.config.ros_topics import NAV_TOPICS, load_config

    original = NAV_TOPICS["cmd_vel"]
    sentinel = "test_cmd_vel_override"
    tmp_path = None
    try:
        with tempfile.NamedTemporaryFile("w", suffix=".yaml", delete=False) as f:
            f.write(f"NAV_TOPICS:\n  cmd_vel: \"{sentinel}\"\n")
            tmp_path = f.name

        load_config(tmp_path)
        if NAV_TOPICS["cmd_vel"] != sentinel:
            return _fail(f"load_config did not update NAV_TOPICS in place "
                         f"(got '{NAV_TOPICS['cmd_vel']}')")
        return _pass(f"NAV_TOPICS['cmd_vel'] updated in place: '{original}' -> '{sentinel}'")
    finally:
        # Restore the dict so the rest of the run uses defaults
        NAV_TOPICS["cmd_vel"] = original
        if tmp_path and os.path.exists(tmp_path):
            os.unlink(tmp_path)


# ── Online tests (require a reachable robot) ─────────────────────────────────


def test_connect_disconnect(ip: str, port: int) -> str:
    _section("TEST 3: connect, is_connected, idempotent disconnect")
    bot = WalkieRobot(ip=ip, ros_protocol="rosbridge", ros_port=port, camera_protocol="none")
    if not bot.is_connected:
        bot.disconnect()
        return _fail("is_connected False right after construction")
    bot.disconnect()
    if bot.is_connected:
        return _fail("is_connected still True after disconnect()")
    bot.disconnect()  # second call must be a safe no-op
    return _pass("connected, disconnected, and second disconnect was a no-op")


def test_properties(ip: str, port: int) -> str:
    _section("TEST 4: identity / protocol properties")
    bot = WalkieRobot(ip=ip, ros_protocol="rosbridge", ros_port=port, camera_protocol="none")
    try:
        checks = {
            "ip": bot.ip == ip,
            "ros_protocol": bot.ros_protocol == "rosbridge",
            "camera_protocol": bot.camera_protocol == "none",
        }
        bad = [k for k, ok in checks.items() if not ok]
        if bad:
            return _fail(f"unexpected property values: {bad} "
                         f"(ip={bot.ip}, ros={bot.ros_protocol}, cam={bot.camera_protocol})")
        return _pass(f"ip={bot.ip}  ros_protocol={bot.ros_protocol}  camera_protocol={bot.camera_protocol}")
    finally:
        bot.disconnect()


def test_namespace_fanout(ip: str, port: int) -> str:
    _section("TEST 5: namespace setter fans out to all modules")
    bot = WalkieRobot(ip=ip, ros_protocol="rosbridge", ros_port=port, camera_protocol="none")
    try:
        bot.namespace = "testns"
        mods = {
            "nav": bot.nav.namespace,
            "status": bot.status.namespace,
            "arm": bot.arm.namespace,
            "lift": bot.lift.namespace,
            "viz": bot.viz.namespace,
        }
        wrong = {k: v for k, v in mods.items() if v != "testns"}
        if wrong:
            return _fail(f"modules not updated to 'testns': {wrong}")
        # Topic name properties should reflect the namespace prefix
        if not bot.nav.cmd_vel_topic.startswith("testns/"):
            return _fail(f"nav.cmd_vel_topic not namespaced: '{bot.nav.cmd_vel_topic}'")
        return _pass(f"all modules -> 'testns'; nav.cmd_vel_topic='{bot.nav.cmd_vel_topic}'")
    finally:
        bot.disconnect()


def test_context_manager(ip: str, port: int) -> str:
    _section("TEST 6: context manager auto-disconnects")
    with WalkieRobot(ip=ip, ros_protocol="rosbridge", ros_port=port, camera_protocol="none") as bot:
        if not bot.is_connected:
            return _fail("not connected inside 'with' block")
        ref = bot
    if ref.is_connected:
        return _fail("still connected after 'with' block exited")
    return _pass("connected inside block, auto-disconnected on exit")


def test_legacy_params(ip: str, port: int) -> str:
    _section("TEST 7: legacy params (enable_camera=False, ws_port -> ros_port)")
    # enable_camera=False must disable the camera entirely
    bot = WalkieRobot(ip=ip, ros_protocol="rosbridge", ws_port=port, enable_camera=False)
    try:
        if bot.camera is not None:
            return _fail("enable_camera=False but bot.camera is not None")
        if bot.camera_protocol != "none":
            return _fail(f"enable_camera=False but camera_protocol='{bot.camera_protocol}'")
        if not bot.is_connected:
            return _fail("ws_port did not map to ros_port — failed to connect")
        return _pass("enable_camera=False disabled camera; ws_port mapped to ros_port")
    finally:
        bot.disconnect()


# ── Main ───────────────────────────────────────────────────────────────────


def main() -> None:
    parser = argparse.ArgumentParser(description="Connection / lifecycle / namespace / config test")
    parser.add_argument("--ip", default="127.0.0.1", help="Robot IP (default: 127.0.0.1)")
    parser.add_argument("--port", type=int, default=9090, help="Rosbridge port (default: 9090)")
    parser.add_argument("--offline-only", action="store_true",
                        help="Run only the offline checks (no robot needed)")
    args = parser.parse_args()

    results = []

    # Offline checks always run.
    results.append(test_invalid_protocol())
    results.append(test_config_yaml_override())

    if args.offline_only:
        print("\n  --offline-only: skipping connection-dependent tests.")
    else:
        print(f"\n  Online tests will connect to {args.ip}:{args.port} ...")
        online = [
            test_connect_disconnect,
            test_properties,
            test_namespace_fanout,
            test_context_manager,
            test_legacy_params,
        ]
        for fn in online:
            try:
                results.append(fn(args.ip, args.port))
            except (ConnectionError, TimeoutError, OSError) as e:
                results.append(_skip(f"{fn.__name__}: robot unreachable ({e})"))
            except KeyboardInterrupt:
                print("\nInterrupted by user.")
                break

    passed = results.count("PASS")
    failed = results.count("FAIL")
    skipped = results.count("SKIP")
    total = len(results)
    print(f"\n{'=' * 60}")
    print(f"  Results: {passed} passed, {failed} failed, {skipped} skipped  (of {total})")
    print("=" * 60)
    sys.exit(0 if failed == 0 else 1)


if __name__ == "__main__":
    main()
