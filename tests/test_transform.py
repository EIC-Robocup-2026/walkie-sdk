"""
Integration test for Transform.lookup() against a live robot.

Requires a running rosbridge server and the walkie_tf/get_transform service
(ros2 run walkie_tf tf_server).

With --viz, the returned transform is visualized as an axis-aligned marker in
RViz2 at the position of the target frame relative to the source frame. Open
RViz2, set the Fixed Frame to the source frame (e.g. "map"), and add a "Marker"
display on topic `walkie/viz_markers`.

Usage:
    python tests/test_transform.py --ip 192.168.1.100
    python tests/test_transform.py --ip 192.168.1.100 --source map --target base_link
    python tests/test_transform.py --ip 192.168.1.100 --source odom --target camera_link
    python tests/test_transform.py --ip 192.168.1.100 --namespace robot1
    python tests/test_transform.py --ip 192.168.1.100 --viz
    python tests/test_transform.py --ip 192.168.1.100 --timeout 3.0
"""

import argparse
import sys

from walkie_sdk import WalkieRobot, SPHERE, ARROW


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


def _print_result(result: dict) -> None:
    pos = result["position"]
    quat = result["quaternion"]
    print(f"    position  : x={pos['x']:.4f}  y={pos['y']:.4f}  z={pos['z']:.4f}")
    print(f"    quaternion: x={quat['x']:.4f}  y={quat['y']:.4f}  z={quat['z']:.4f}  w={quat['w']:.4f}")


def _visualize(robot: WalkieRobot, result: dict, source_frame: str, target_frame: str) -> None:
    pos = result["position"]
    quat = result["quaternion"]
    position = [pos["x"], pos["y"], pos["z"]]
    quaternion = [quat["x"], quat["y"], quat["z"], quat["w"]]

    sphere_id = robot.draw_marker(
        position=position,
        quaternion=quaternion,
        marker_type=SPHERE,
        color=[0.0, 1.0, 0.5, 1.0],
        scale=[0.12, 0.12, 0.12],
        frame_id=source_frame,
        ns="transform_result",
    )
    arrow_id = robot.draw_marker(
        position=position,
        quaternion=quaternion,
        marker_type=ARROW,
        color=[1.0, 0.5, 0.0, 1.0],
        scale=[0.2, 0.03, 0.03],
        frame_id=source_frame,
        ns="transform_result_arrow",
    )
    print(f"  [viz] marker at ({position[0]:.3f}, {position[1]:.3f}, {position[2]:.3f})"
          f" in frame '{source_frame}' (ids {sphere_id}, {arrow_id})")


# ── Test cases ─────────────────────────────────────────────────────────────


def test_known_frames(robot: WalkieRobot, source: str, target: str,
                      timeout: float, viz: bool) -> str:
    _section(f"TEST 1: lookup '{source}' -> '{target}'")
    result = robot.transform.lookup(source, target, timeout=timeout)
    print(f"  Result: {result}")

    if result is None:
        return _fail("returned None — service unavailable or frames not in TF tree")
    if "position" not in result or "quaternion" not in result:
        return _fail(f"unexpected response structure: {result!r}")
    pos = result["position"]
    quat = result["quaternion"]
    for key in ("x", "y", "z"):
        if key not in pos:
            return _fail(f"position missing key '{key}'")
    for key in ("x", "y", "z", "w"):
        if key not in quat:
            return _fail(f"quaternion missing key '{key}'")
    _print_result(result)
    if viz:
        _visualize(robot, result, source, target)
    return _pass(f"received valid transform {source!r} -> {target!r}")


def test_same_frame(robot: WalkieRobot, source: str, timeout: float) -> str:
    _section(f"TEST 2: identity lookup '{source}' -> '{source}'")
    result = robot.transform.lookup(source, source, timeout=timeout)
    print(f"  Result: {result}")

    if result is None:
        return _fail("returned None — service should handle same-frame lookup")
    pos = result["position"]
    quat = result["quaternion"]
    tol = 1e-4
    pos_zero = all(abs(pos[k]) < tol for k in ("x", "y", "z"))
    quat_identity = (
        abs(quat["x"]) < tol
        and abs(quat["y"]) < tol
        and abs(quat["z"]) < tol
        and abs(abs(quat["w"]) - 1.0) < tol
    )
    if not pos_zero:
        return _fail(f"expected zero translation for same frame, got {pos}")
    if not quat_identity:
        return _fail(f"expected identity quaternion for same frame, got {quat}")
    _print_result(result)
    return _pass("identity transform (zero translation, identity quaternion)")


def test_unknown_frame(robot: WalkieRobot, timeout: float) -> str:
    _section("TEST 3: unknown frame (expect graceful None)")
    frame = "walkie_nonexistent_frame_xyz_9999"
    print(f"  source='map'  target='{frame}'")
    result = robot.transform.lookup("map", frame, timeout=min(timeout, 2.0))
    print(f"  Result: {result}")

    if result is None:
        return _pass("returned None cleanly for unknown frame")
    return _fail(f"expected None for non-existent frame, got {result!r}")


def test_short_timeout(robot: WalkieRobot, source: str, target: str) -> str:
    _section("TEST 4: very short timeout (expect graceful None)")
    print(f"  source={source!r}  target={target!r}  timeout=0.0001s")
    result = robot.transform.lookup(source, target, timeout=0.0001)
    print(f"  Result: {result}")

    if result is None:
        return _pass("timed out cleanly and returned None")
    # Service may respond before the tiny timeout — also acceptable
    return _pass(f"service responded before timeout: {result!r}")


# ── Main ───────────────────────────────────────────────────────────────────


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Integration test for Transform.lookup() — requires walkie_tf service"
    )
    parser.add_argument("--ip", default="127.0.0.1", help="Robot IP (default: 127.0.0.1)")
    parser.add_argument("--port", type=int, default=9090, help="Rosbridge port (default: 9090)")
    parser.add_argument("--timeout", type=float, default=5.0, help="Service timeout in seconds (default: 5.0)")
    parser.add_argument("--namespace", default="", help="ROS namespace (default: none)")
    parser.add_argument("--source", default="map", help="Source TF frame (default: map)")
    parser.add_argument("--target", default="base_link", help="Target TF frame (default: base_link)")
    parser.add_argument("--viz", action="store_true",
                        help="Publish the transform result as an RViz2 marker")
    args = parser.parse_args()

    print(f"\nConnecting to rosbridge at {args.ip}:{args.port} ...")
    robot = WalkieRobot(
        ip=args.ip,
        camera_protocol="none",
        namespace=args.namespace,
    )
    print("Connected.")

    if args.viz:
        try:
            robot.viz.clear_markers()
        except Exception:
            pass
        print(f"  Visualization ON — open RViz2, Fixed Frame='{args.source}', "
              f"add Marker display on 'walkie/viz_markers'"
              f"{' (prefix ' + args.namespace + '/)' if args.namespace else ''}.")

    results = []
    try:
        results.append(test_known_frames(robot, args.source, args.target, args.timeout, args.viz))
        results.append(test_same_frame(robot, args.source, args.timeout))
        results.append(test_unknown_frame(robot, args.timeout))
        results.append(test_short_timeout(robot, args.source, args.target))
    except KeyboardInterrupt:
        print("\nInterrupted by user.")
    finally:
        robot.disconnect()
        print("\nDisconnected.")

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
