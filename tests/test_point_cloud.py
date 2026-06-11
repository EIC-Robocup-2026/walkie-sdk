"""
Hardware integration test for the PointCloud module on a live robot.

Read-only — no motion. Verifies that PointCloud2 messages actually arrive
from the ZED head filtered-map topic and are well-formed.

Tests:

  1. Facade path (default): bot.point_cloud as wired by WalkieRobot.
     - get_once(): blocking subscribe-once (--once equivalent)
     - get_cloud(): non-blocking latest cached message
     - get_all_clouds(): all subscribed sources

  2. Direct path (--direct): constructs PointCloud directly via ROSBridgeTransport.

  3. Visualize (--visualize): opens a matplotlib 3D scatter of the point cloud.

Requires: a robot publishing
  zed_head/zed_node/point_cloud/cloud_registered/filtered_map
  over rosbridge (default port 9090).

Usage:
    python tests/test_point_cloud.py --ip 192.168.1.100
    python tests/test_point_cloud.py --ip 192.168.1.100 --timeout 15.0
    python tests/test_point_cloud.py --ip 192.168.1.100 --direct
    python tests/test_point_cloud.py --ip 192.168.1.100 --visualize
    python tests/test_point_cloud.py --ip 192.168.1.100 --namespace robot1
"""

import argparse
import sys
import time

from walkie_sdk import WalkieRobot
from walkie_sdk.config.ros_topics import POINT_CLOUD_TOPICS
from walkie_sdk.modules.point_cloud import PointCloud


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


def _print_cloud_info(cloud: dict, prefix: str = "    ") -> None:
    """Print a human-readable summary of a PointCloud2 message dict."""
    try:
        header = cloud.get("header", {})
        stamp = header.get("stamp", {})
        print(f"{prefix}frame_id   : {header.get('frame_id', 'N/A')}")
        print(f"{prefix}stamp      : sec={stamp.get('sec', '?')}  nanosec={stamp.get('nanosec', '?')}")
        w = cloud.get('width', 0)
        h = cloud.get('height', 0)
        total = w * h if w and h else w or h
        print(f"{prefix}size       : {total:,} pts  "
              f"(width={w}  height={h}  is_dense={cloud.get('is_dense', '?')})")
        print(f"{prefix}point_step : {cloud.get('point_step', '?')} bytes/pt")
        fields = cloud.get("fields", [])
        field_names = [f.get("name", "?") if isinstance(f, dict) else str(f) for f in fields]
        print(f"{prefix}fields     : {field_names}")
        data = cloud.get("data", b"")
        data_len = len(data) if data is not None else 0
        print(f"{prefix}data bytes : {data_len}")
    except Exception as e:
        print(f"{prefix}(could not print cloud info: {e})")


def _is_valid_cloud(cloud) -> bool:
    """Return True if the cloud dict looks like a valid non-empty PointCloud2."""
    if not isinstance(cloud, dict):
        return False
    if cloud.get("width", 0) == 0:
        return False
    data = cloud.get("data")
    if data is None or len(data) == 0:
        return False
    return True


# ── Facade-path tests (bot.point_cloud) ────────────────────────────────────


def test_get_once(bot: WalkieRobot, timeout: float) -> str:
    _section(f"TEST 1: bot.point_cloud.get_once(timeout={timeout})")
    if not bot.point_cloud.is_subscribed:
        return _fail("bot.point_cloud is not subscribed — _setup_subscription() may have failed")

    print(f"    source_names  : {bot.point_cloud.source_names}")
    print(f"    is_subscribed  : {bot.point_cloud.is_subscribed}")
    print(f"    Waiting up to {timeout}s for first message ...")

    cloud = bot.point_cloud.get_once("head", timeout=timeout)
    if cloud is None:
        return _fail(
            f"no message received within {timeout}s — "
            "is the ZED head publishing the filtered map topic?"
        )

    _print_cloud_info(cloud)

    if not _is_valid_cloud(cloud):
        return _fail(f"message arrived but appears empty (width={cloud.get('width')})")

    total = cloud.get("width", 0) * cloud.get("height", 0)
    return _pass(f"received cloud: {total:,} pts  frame={cloud.get('header', {}).get('frame_id')}")


def test_get_cloud_non_blocking(bot: WalkieRobot) -> str:
    _section("TEST 2: bot.point_cloud.get_cloud() — non-blocking latest")
    cloud = bot.point_cloud.get_cloud("head")
    if cloud is None:
        return _fail(
            "get_cloud() returned None — no message cached yet "
            "(run TEST 1 first to prime the cache)"
        )

    _print_cloud_info(cloud)
    if not _is_valid_cloud(cloud):
        return _fail(f"cached cloud appears empty (width={cloud.get('width')})")

    total = cloud.get("width", 0) * cloud.get("height", 0)
    return _pass(f"cached cloud: {total:,} pts")


def test_get_all_clouds(bot: WalkieRobot) -> str:
    _section("TEST 3: bot.point_cloud.get_all_clouds()")
    all_clouds = bot.point_cloud.get_all_clouds()
    print(f"    sources with data: {list(all_clouds.keys())}")
    if not all_clouds:
        return _fail(
            "get_all_clouds() returned empty dict — no sources have data yet "
            "(run TEST 1 first to prime the cache)"
        )

    for name, cloud in all_clouds.items():
        valid = _is_valid_cloud(cloud)
        total = cloud.get("width", 0) * cloud.get("height", 0)
        print(f"    {name:12s}: {'OK ' + f'{total:,}' + ' pts' if valid else 'INVALID'}")

    valid_sources = [n for n, c in all_clouds.items() if _is_valid_cloud(c)]
    if not valid_sources:
        return _fail("sources received data but all clouds appear empty")

    return _pass(f"valid sources: {valid_sources}")


# ── Visualize (--visualize) ────────────────────────────────────────────────


def test_visualize(bot: WalkieRobot, timeout: float, max_points: int = 50_000) -> str:
    _section(f"TEST 4: bot.point_cloud visualize (timeout={timeout})")
    try:
        import numpy as np
        import matplotlib.pyplot as plt
    except ImportError as e:
        return _skip(f"matplotlib/numpy not available: {e}")

    from walkie_sdk.utils.converters import parse_point_cloud_xyz

    print(f"    Waiting up to {timeout}s for first message ...")
    cloud = bot.point_cloud.get_once("head", timeout=timeout)
    if cloud is None:
        return _fail(f"no message received within {timeout}s")

    pts = parse_point_cloud_xyz(cloud)
    if pts is None or len(pts) == 0:
        return _fail("cloud received but no valid XYZ points parsed")

    total = cloud.get("height", 0) * cloud.get("width", 0)
    if len(pts) > max_points:
        idx = np.random.choice(len(pts), max_points, replace=False)
        pts = pts[idx]

    frame_id = cloud.get("header", {}).get("frame_id", "?")
    print(f"    Plotting {len(pts):,} / {total:,} pts  frame={frame_id} ...")

    fig = plt.figure(figsize=(10, 8))
    ax = fig.add_subplot(111, projection="3d")
    sc = ax.scatter(pts[:, 0], pts[:, 1], pts[:, 2],
                    c=pts[:, 2], cmap="viridis", s=0.3, alpha=0.5)
    plt.colorbar(sc, ax=ax, label="Z (m)", shrink=0.6)
    ax.set_xlabel("X (m)")
    ax.set_ylabel("Y (m)")
    ax.set_zlabel("Z (m)")
    ax.set_title(
        f"PointCloud  source=head  frame={frame_id}\n"
        f"showing {len(pts):,} / {total:,} pts"
    )
    plt.tight_layout()
    plt.show()
    return _pass(f"visualized {len(pts):,} pts")


# ── Direct transport path (--direct) ──────────────────────────────────────


def test_direct_transport(host: str, port: int, timeout: float) -> str:
    _section("TEST 4: direct PointCloud via rosbridge transport (bypasses WalkieRobot facade)")
    try:
        from walkie_sdk.core.transports.rosbridge import ROSBridgeTransport
    except ImportError as e:
        return _skip(f"rosbridge transport unavailable: {e}")

    transport = ROSBridgeTransport(host=host, port=port)
    try:
        transport.connect()
    except Exception as e:
        return _fail(f"ROSBridgeTransport.connect() failed: {e}")

    print(f"    Connected. Setting up PointCloud subscriptions ...")

    try:
        pc = PointCloud(transport)
        pc._setup_subscription()

        print(f"    source_names={pc.source_names}")
        print(f"    Waiting up to {timeout}s for first message ...")

        cloud = pc.get_once("head", timeout=timeout)
        if cloud is None:
            return _fail(f"no message within {timeout}s via direct transport")

        _print_cloud_info(cloud)
        if not _is_valid_cloud(cloud):
            return _fail(f"cloud arrived but appears empty (width={cloud.get('width')})")

        return _pass(
            f"direct path OK: {cloud.get('width')} pts  "
            f"frame={cloud.get('header', {}).get('frame_id')}"
        )
    finally:
        pc.stop()
        transport.disconnect()
        print("    Transport disconnected.")


# ── Main ───────────────────────────────────────────────────────────────────


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Hardware integration test for PointCloud module"
    )
    parser.add_argument("--ip", default="127.0.0.1", help="Robot IP (default: 127.0.0.1)")
    parser.add_argument("--port", type=int, default=9090, help="Rosbridge port (default: 9090)")
    parser.add_argument("--camera-port", type=int, default=7447, dest="camera_port",
                        help="Zenoh router port (default: 7447)")
    parser.add_argument("--timeout", type=float, default=10.0,
                        help="Seconds to wait for first cloud message (default: 10.0)")
    parser.add_argument("--namespace", default="", help="ROS namespace (default: none)")
    parser.add_argument("--direct", action="store_true",
                        help="Also run direct ZenohPointCloud test (bypasses WalkieRobot)")
    parser.add_argument("--visualize", action="store_true",
                        help="Fetch one cloud and display a matplotlib 3D scatter plot")
    args = parser.parse_args()

    print(f"\nConnecting to {args.ip} (rosbridge:{args.port}, zenoh:{args.camera_port}) ...")
    print(f"  Topic: {POINT_CLOUD_TOPICS['head']}")

    bot = WalkieRobot(
        ros_protocol="rosbridge",
        ip=args.ip,
        ros_port=args.port,
        camera_protocol="none",
        namespace=args.namespace,
    )
    print("Connected.\n")
    print(f"  timeout     = {args.timeout}s")
    print(f"  is_subscribed= {bot.point_cloud.is_subscribed if bot.point_cloud else 'N/A (None)'}")

    results = []
    try:
        results.append(test_get_once(bot, args.timeout))
        results.append(test_get_cloud_non_blocking(bot))
        results.append(test_get_all_clouds(bot))
        if args.visualize:
            results.append(test_visualize(bot, args.timeout))
        if args.direct:
            results.append(test_direct_transport(args.ip, args.port, args.timeout))
    except KeyboardInterrupt:
        print("\nInterrupted by user.")
    finally:
        bot.disconnect()
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
