"""
Hardware integration test for the Visualization module (bot.viz) against a live robot.

Safe — publishes only RViz2 markers/poses, never commands motion.

Most checks are programmatic (publish succeeds, correct return types, expected
errors). To *visually* confirm, open RViz2 and add:
  - "Marker"      on  walkie/viz_markers
  - "MarkerArray" on  walkie/viz_markers_array
  - "Pose"        on  walkie/target_pose
(prefix with your namespace if --namespace is set). Use --delay to slow the
sequence down enough to watch.

Requires only a running rosbridge server.

Usage:
    python tests/test_visualization.py
    python tests/test_visualization.py --ip 192.168.1.100
    python tests/test_visualization.py --namespace robot1 --delay 1.5 --frame map
"""

import argparse
import sys
import time

from walkie_sdk import (
    WalkieRobot,
    ARROW, CUBE, SPHERE, CYLINDER, LINE_STRIP, TEXT_VIEW_FACING,
)


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


# ── Test cases ─────────────────────────────────────────────────────────────


def test_draw_marker(bot: WalkieRobot, frame: str, delay: float) -> str:
    _section("TEST 1: draw_marker() — default red arrow")
    mid = bot.viz.draw_marker([1.0, 0.0, 0.0], frame_id=frame)
    time.sleep(delay)
    if not isinstance(mid, int):
        return _fail(f"expected int marker id, got {type(mid)}")
    return _pass(f"published arrow, marker_id={mid}")


def test_marker_types(bot: WalkieRobot, frame: str, delay: float) -> str:
    _section("TEST 2: draw each marker type (spread along +X)")
    specs = [
        (CUBE, [1.0, 0.0, 0.0, 1.0], "cube"),
        (SPHERE, [0.0, 1.0, 0.0, 1.0], "sphere"),
        (CYLINDER, [0.0, 0.0, 1.0, 1.0], "cylinder"),
        (LINE_STRIP, [1.0, 1.0, 0.0, 1.0], "line_strip"),
        (TEXT_VIEW_FACING, [1.0, 1.0, 1.0, 1.0], "text"),
    ]
    ids = []
    try:
        for i, (mtype, color, label) in enumerate(specs):
            kwargs = dict(frame_id=frame, marker_type=mtype, color=color)
            if mtype == TEXT_VIEW_FACING:
                kwargs["text"] = "walkie"
            if mtype == LINE_STRIP:
                # LINE_STRIP needs points; draw a tiny segment via two markers fallback
                kwargs["scale"] = [0.02, 0.0, 0.0]
            mid = bot.viz.draw_marker([0.0, float(i) * 0.4, 0.0], **kwargs)
            ids.append(mid)
            print(f"    {label:10s} -> id={mid}")
            time.sleep(delay)
    except Exception as e:
        return _fail(f"draw_marker raised for a type: {e}")
    if len(ids) != len(specs):
        return _fail(f"expected {len(specs)} ids, got {len(ids)}")
    return _pass(f"published {len(ids)} marker types: ids={ids}")


def test_update_marker(bot: WalkieRobot, frame: str, delay: float) -> str:
    _section("TEST 3: update_marker() — move + recolor an existing marker")
    mid = bot.viz.draw_marker([0.5, 0.0, 0.0], frame_id=frame, marker_type=SPHERE,
                              color=[1.0, 0.0, 0.0, 1.0], scale=[0.2, 0.2, 0.2])
    time.sleep(delay)
    try:
        bot.viz.update_marker(mid, position=[0.5, 0.5, 0.5], color=[0.0, 1.0, 0.0, 1.0])
    except Exception as e:
        return _fail(f"update_marker raised: {e}")
    time.sleep(delay)
    return _pass(f"updated marker id={mid} (red->green, moved up)")


def test_draw_markers_array(bot: WalkieRobot, frame: str, delay: float) -> str:
    _section("TEST 4: draw_markers() — MarkerArray")
    markers = [
        {"position": [2.0, 0.0, 0.0], "marker_type": ARROW, "color": [1.0, 0.0, 0.0, 1.0]},
        {"position": [2.0, 0.5, 0.0], "marker_type": SPHERE, "color": [0.0, 1.0, 0.0, 1.0]},
        {"position": [2.0, 1.0, 0.0], "marker_type": CUBE, "color": [0.0, 0.0, 1.0, 1.0]},
    ]
    for m in markers:
        m["frame_id"] = frame
    ids = bot.viz.draw_markers(markers)
    time.sleep(delay)
    if not isinstance(ids, list) or len(ids) != len(markers):
        return _fail(f"expected {len(markers)} ids, got {ids}")
    return _pass(f"published MarkerArray, ids={ids}")


def test_delete_and_clear(bot: WalkieRobot, frame: str, delay: float) -> str:
    _section("TEST 5: delete_marker() then clear_markers()")
    mid = bot.viz.draw_marker([1.5, 0.0, 0.0], frame_id=frame, marker_type=CUBE)
    time.sleep(delay)
    try:
        bot.viz.delete_marker(mid)
        time.sleep(delay)
        bot.viz.clear_markers()
        time.sleep(delay)
    except Exception as e:
        return _fail(f"delete/clear raised: {e}")
    return _pass(f"deleted marker id={mid} and cleared all markers")


def test_draw_pose(bot: WalkieRobot, frame: str, delay: float) -> str:
    _section("TEST 6: draw_pose() / update_pose()")
    topic = bot.viz.draw_pose([1.0, 0.0, 0.3], frame_id=frame)
    time.sleep(delay)
    if not isinstance(topic, str):
        return _fail(f"draw_pose should return topic str, got {type(topic)}")
    try:
        bot.viz.update_pose(position=[1.0, 0.5, 0.3], topic=topic)
    except Exception as e:
        return _fail(f"update_pose raised: {e}")
    time.sleep(delay)
    return _pass(f"published + updated pose on topic '{topic}'")


def test_robot_convenience(bot: WalkieRobot, frame: str, delay: float) -> str:
    _section("TEST 7: WalkieRobot convenience wrappers")
    try:
        mid = bot.draw_marker([0.0, 0.0, 1.0], frame_id=frame, marker_type=SPHERE)
        bot.update_marker(mid, position=[0.0, 0.0, 1.2])
        topic = bot.draw_pose([0.0, 0.0, 1.0], frame_id=frame)
        bot.update_pose(position=[0.0, 0.0, 1.2], topic=topic)
        time.sleep(delay)
    except Exception as e:
        return _fail(f"convenience wrapper raised: {e}")
    return _pass(f"bot.draw_marker/update_marker/draw_pose/update_pose all OK (id={mid})")


def test_error_cases(bot: WalkieRobot) -> str:
    _section("TEST 8: expected KeyError on update of unknown marker/pose")
    ok = True
    try:
        bot.viz.update_marker(999999, position=[0, 0, 0])
        ok = False
        print("    update_marker(unknown) did NOT raise")
    except KeyError:
        print("    update_marker(unknown) raised KeyError (expected)")
    try:
        bot.viz.update_pose(position=[0, 0, 0], topic="walkie/does_not_exist")
        ok = False
        print("    update_pose(unknown) did NOT raise")
    except KeyError:
        print("    update_pose(unknown) raised KeyError (expected)")
    return _pass("unknown id/topic correctly raise KeyError") if ok \
        else _fail("missing KeyError on unknown id/topic")


# ── Main ───────────────────────────────────────────────────────────────────


def main() -> None:
    parser = argparse.ArgumentParser(description="Hardware test for Visualization (bot.viz)")
    parser.add_argument("--ip", default="127.0.0.1", help="Robot IP (default: 127.0.0.1)")
    parser.add_argument("--port", type=int, default=9090, help="Rosbridge port (default: 9090)")
    parser.add_argument("--frame", default="base_link", help="TF frame_id for markers (default: base_link)")
    parser.add_argument("--delay", type=float, default=0.5, help="Seconds between draws, to watch in RViz2 (default: 0.5)")
    parser.add_argument("--namespace", default="", help="ROS namespace (default: none)")
    args = parser.parse_args()

    print(f"\nConnecting to rosbridge at {args.ip}:{args.port} ...")
    bot = WalkieRobot(
        ip=args.ip,
        camera_protocol="none",
        namespace=args.namespace,
    )
    print("Connected.")
    print(f"  Open RViz2 on topics walkie/viz_markers[_array] and walkie/target_pose"
          f"{' (prefixed with ' + args.namespace + '/)' if args.namespace else ''} to watch.\n")

    results = []
    try:
        results.append(test_draw_marker(bot, args.frame, args.delay))
        results.append(test_marker_types(bot, args.frame, args.delay))
        results.append(test_update_marker(bot, args.frame, args.delay))
        results.append(test_draw_markers_array(bot, args.frame, args.delay))
        results.append(test_delete_and_clear(bot, args.frame, args.delay))
        results.append(test_draw_pose(bot, args.frame, args.delay))
        results.append(test_robot_convenience(bot, args.frame, args.delay))
        results.append(test_error_cases(bot))
    except KeyboardInterrupt:
        print("\nInterrupted by user.")
    finally:
        bot.disconnect()
        print("\nDisconnected.")

    passed = results.count("PASS")
    total = len(results)
    print(f"\n{'=' * 60}")
    print(f"  Results: {passed}/{total} passed")
    print("=" * 60)
    sys.exit(0 if passed == total and total > 0 else 1)


if __name__ == "__main__":
    main()
