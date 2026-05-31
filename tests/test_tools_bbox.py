"""
Integration test for Tools.bboxes_to_positions() against a live robot.

Requires a running rosbridge server and the walkie_perception/srv/GetObPose service.

With --viz, the returned 3D positions are published as RViz2 markers (a colored
sphere + text label per detection) so you can SEE the result. Open RViz2, set
the Fixed Frame to match --viz-frame (default "map"), and add a "Marker" display
on topic `walkie/viz_markers` (prefixed with your namespace if set).

For an interactive version (draw boxes on the live camera feed with the mouse),
see tests/test_tools_bbox_interactive.py.

Usage:
    python tests/test_tools_bbox.py
    python tests/test_tools_bbox.py --ip 192.168.1.100
    python tests/test_tools_bbox.py --ip 192.168.1.100 --port 9090 --timeout 5.0
    python tests/test_tools_bbox.py --namespace robot1
    python tests/test_tools_bbox.py --ip 192.168.1.100 --viz                 # visualize in RViz2
    python tests/test_tools_bbox.py --ip 192.168.1.100 --viz --viz-frame camera_link --viz-hold 5
"""

import argparse
import sys
import time

from walkie_sdk import WalkieRobot, SPHERE, TEXT_VIEW_FACING


# ── Helpers ────────────────────────────────────────────────────────────────


def _section(title: str):
    print(f"\n{'=' * 60}")
    print(f"  {title}")
    print("=" * 60)


def _pass(msg: str):
    print(f"  [PASS] {msg}")


def _fail(msg: str):
    print(f"  [FAIL] {msg}")


# Distinct colors per detection so multiple positions are easy to tell apart.
_MARKER_COLORS = [
    [1.0, 0.0, 0.0, 1.0],  # red
    [0.0, 1.0, 0.0, 1.0],  # green
    [0.0, 0.4, 1.0, 1.0],  # blue
    [1.0, 1.0, 0.0, 1.0],  # yellow
    [1.0, 0.0, 1.0, 1.0],  # magenta
]


def _visualize(robot: WalkieRobot, positions, frame_id: str, viz_ids: list) -> None:
    """Publish a sphere + text label per 3D position to RViz2.

    Deletes markers from the previous query (tracked in viz_ids) first so the
    display always shows the most recent result.
    """
    for mid in viz_ids:
        try:
            robot.viz.delete_marker(mid)
        except Exception:
            pass
    viz_ids.clear()

    for i, pos in enumerate(positions):
        color = _MARKER_COLORS[i % len(_MARKER_COLORS)]
        sphere_id = robot.draw_marker(
            position=pos,
            quaternion=[0.0, 0.0, 0.0, 1.0],
            marker_type=SPHERE,
            color=color,
            scale=[0.1, 0.1, 0.1],
            frame_id=frame_id,
            ns="bbox_detections",
        )
        viz_ids.append(sphere_id)

        label = f"det_{i} ({pos[0]:.2f}, {pos[1]:.2f}, {pos[2]:.2f})"
        text_id = robot.draw_marker(
            position=[pos[0], pos[1], pos[2] + 0.15],
            quaternion=[0.0, 0.0, 0.0, 1.0],
            marker_type=TEXT_VIEW_FACING,
            text=label,
            color=[1.0, 1.0, 1.0, 1.0],
            scale=[0.08, 0.08, 0.08],
            frame_id=frame_id,
            ns="bbox_detections_label",
        )
        viz_ids.append(text_id)

    print(f"  [viz] {len(positions)} marker(s) on 'walkie/viz_markers' (frame_id='{frame_id}')")


# ── Test cases ─────────────────────────────────────────────────────────────


def test_single_bbox(robot, timeout, viz=False, frame_id="map", viz_ids=None) -> bool:
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
    if viz:
        _visualize(robot, result, frame_id, viz_ids)
    _pass(f"x={pos[0]:.3f}  y={pos[1]:.3f}  z={pos[2]:.3f}")
    return True


def test_multiple_bboxes(robot, timeout, viz=False, frame_id="map", viz_ids=None) -> bool:
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
    if viz:
        _visualize(robot, result, frame_id, viz_ids)
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
    parser.add_argument("--viz", action="store_true", help="Publish results as RViz2 markers")
    parser.add_argument("--viz-frame", default="map", dest="viz_frame",
                        help="TF frame for the markers (default: map). Set to the frame the "
                             "service returns poses in if markers appear in the wrong place.")
    parser.add_argument("--viz-hold", type=float, default=3.0, dest="viz_hold",
                        help="Seconds to keep each visualized result on screen (default: 3.0)")
    args = parser.parse_args()

    print(f"\nConnecting to rosbridge at {args.ip}:{args.port} ...")
    robot = WalkieRobot(
        ros_protocol="rosbridge",
        ip=args.ip,
        ros_port=args.port,
        camera_protocol="none",  # service + marker test; no camera needed
        namespace=args.namespace,
    )
    print("Connected.")

    viz_ids: list = []
    if args.viz:
        # Start from a clean slate so stale markers from a previous run go away.
        try:
            robot.viz.clear_markers()
        except Exception:
            pass
        print(f"  Visualization ON — open RViz2, Fixed Frame='{args.viz_frame}', "
              f"add Marker display on 'walkie/viz_markers'"
              f"{' (prefix ' + args.namespace + '/)' if args.namespace else ''}.")

    results = []
    try:
        results.append(test_single_bbox(robot, args.timeout, args.viz, args.viz_frame, viz_ids))
        if args.viz:
            time.sleep(args.viz_hold)
        results.append(test_multiple_bboxes(robot, args.timeout, args.viz, args.viz_frame, viz_ids))
        if args.viz:
            time.sleep(args.viz_hold)
        results.append(test_empty_list(robot, args.timeout))
        results.append(test_short_timeout(robot))
    except KeyboardInterrupt:
        print("\nInterrupted by user.")
    finally:
        if args.viz and viz_ids:
            print("\n  Markers left in RViz2 (re-run to refresh, or call viz.clear_markers()).")
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
