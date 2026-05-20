"""
Integration test for Tools.bboxes_to_positions() against a live robot.

Requires a running rosbridge server and the perception/srv/GetObPose service.

Usage:
    python tests/test_tools_bbox.py
    python tests/test_tools_bbox.py --ip 192.168.1.100
    python tests/test_tools_bbox.py --ip 192.168.1.100 --port 9090 --timeout 5.0
    python tests/test_tools_bbox.py --namespace robot1
"""

import argparse
import sys

from walkie_sdk import WalkieRobot


# ── Helpers ────────────────────────────────────────────────────────────────


def _section(title: str):
    print(f"\n{'=' * 60}")
    print(f"  {title}")
    print("=" * 60)


def _pass(msg: str):
    print(f"  [PASS] {msg}")


def _fail(msg: str):
    print(f"  [FAIL] {msg}")


# ── Test cases ─────────────────────────────────────────────────────────────


def test_single_bbox(robot: WalkieRobot, timeout: float) -> bool:
    _section("TEST 1: single bounding box")
    bbox = [[320.0, 240.0, 50.0, 50.0]]
    print(f"  Input:  {bbox}")

    result = robot.tools.bboxes_to_positions(bbox, timeout=timeout)
    print(f"  Output: {result}")

    if result is None:
        _fail("returned None — service unavailable or failed")
        return False
    if not isinstance(result, list) or len(result) != 1:
        _fail(f"expected list of 1, got: {result!r}")
        return False
    pos = result[0]
    if len(pos) != 3:
        _fail(f"expected [x, y, z], got: {pos!r}")
        return False
    _pass(f"x={pos[0]:.3f}  y={pos[1]:.3f}  z={pos[2]:.3f}")
    return True


def test_multiple_bboxes(robot: WalkieRobot, timeout: float) -> bool:
    _section("TEST 2: multiple bounding boxes")
    bboxes = [
        [320.0, 240.0, 50.0, 50.0],
        [220.0, 140.0, 50.0, 50.0],
        [600.0, 300.0, 50.0, 50.0],
    ]
    print(f"  Input:  {bboxes}")

    result = robot.tools.bboxes_to_positions(bboxes, timeout=timeout)
    print(f"  Output: {result}")

    if result is None:
        _fail("returned None")
        return False
    if not isinstance(result, list) or len(result) != len(bboxes):
        _fail(f"expected {len(bboxes)} positions, got: {result!r}")
        return False
    for i, pos in enumerate(result):
        print(f"    bbox[{i}] → x={pos[0]:.3f}  y={pos[1]:.3f}  z={pos[2]:.3f}")
    _pass(f"received {len(result)} 3D positions")
    return True


def test_empty_list(robot: WalkieRobot, timeout: float) -> bool:
    _section("TEST 3: empty bbox list")
    print("  Input:  []")

    result = robot.tools.bboxes_to_positions([], timeout=timeout)
    print(f"  Output: {result}")

    # Acceptable: empty list or None (service may reject empty input)
    if result == [] or result is None:
        _pass(f"returned {result!r} (acceptable for empty input)")
        return True
    _fail(f"unexpected response: {result!r}")
    return False


def test_short_timeout(robot: WalkieRobot) -> bool:
    _section("TEST 4: very short timeout (expect graceful None)")
    bbox = [[320.0, 240.0, 50.0, 50.0]]
    print(f"  Input:  {bbox}  timeout=0.001s")

    result = robot.tools.bboxes_to_positions(bbox, timeout=0.001)
    print(f"  Output: {result}")

    if result is None:
        _pass("timed out cleanly and returned None")
        return True
    # If the service is extremely fast it might still succeed — that's fine too
    _pass(f"service responded before timeout: {result!r}")
    return True


# ── Main ───────────────────────────────────────────────────────────────────


def main():
    parser = argparse.ArgumentParser(
        description="Integration test for Tools.bboxes_to_positions()"
    )
    parser.add_argument("--ip", default="127.0.0.1", help="Robot IP (default: 127.0.0.1)")
    parser.add_argument("--port", type=int, default=9090, help="Rosbridge port (default: 9090)")
    parser.add_argument("--timeout", type=float, default=5.0, help="Service timeout in seconds (default: 5.0)")
    parser.add_argument("--namespace", default="", help="ROS namespace (default: none)")
    args = parser.parse_args()

    print(f"\nConnecting to rosbridge at {args.ip}:{args.port} ...")
    robot = WalkieRobot(
        ros_protocol="rosbridge",
        ip=args.ip,
        ros_port=args.port,
        namespace=args.namespace,
    )
    print("Connected.")

    results = []
    try:
        results.append(test_single_bbox(robot, args.timeout))
        results.append(test_multiple_bboxes(robot, args.timeout))
        results.append(test_empty_list(robot, args.timeout))
        results.append(test_short_timeout(robot))
    except KeyboardInterrupt:
        print("\nInterrupted by user.")
    finally:
        robot.disconnect()
        print("\nDisconnected.")

    passed = sum(results)
    total = len(results)
    print(f"\n{'=' * 60}")
    print(f"  Results: {passed}/{total} passed")
    print("=" * 60)
    sys.exit(0 if passed == total else 1)


if __name__ == "__main__":
    main()
