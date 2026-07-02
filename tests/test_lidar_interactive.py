"""
Interactive 2D lidar visualization test against a live robot.

Subscribes to the robot's laser scan via bot.lidar, converts each LaserScan
from polar (angle, range) to Cartesian (x, y) in the sensor frame, and plots
the points in a top-down 2D scatter. By default it animates live, redrawing on
every new scan; pass --once for a single snapshot.

The robot sits at the origin (0, 0). +x is forward (angle 0), +y is left.

Requires a running robot publishing sensor_msgs/msg/LaserScan on the scan topic.
matplotlib is needed for display:

    uv pip install matplotlib

Usage:
    python tests/test_lidar_interactive.py --ip 192.168.1.100
    python tests/test_lidar_interactive.py --ip 192.168.1.100 --once --save /tmp/scan.png
    python tests/test_lidar_interactive.py --ip 192.168.1.100 --namespace robot1
"""

import argparse
import math
import sys

from walkie_sdk import WalkieRobot


def _scan_to_xy(scan):
    """Convert a LaserScan dict to lists of Cartesian (xs, ys) in the sensor frame.

    Skips beams that are NaN/inf or outside [range_min, range_max].
    """
    ranges = scan.get("ranges") or []
    angle = scan.get("angle_min", 0.0)
    inc = scan.get("angle_increment", 0.0)
    rmin = scan.get("range_min", 0.0)
    rmax = scan.get("range_max", float("inf"))

    xs, ys = [], []
    for r in ranges:
        theta = angle
        angle += inc
        if r is None:
            continue
        try:
            rf = float(r)
        except (TypeError, ValueError):
            continue
        if math.isnan(rf) or math.isinf(rf):
            continue
        if rf < rmin or rf > rmax:
            continue
        xs.append(rf * math.cos(theta))
        ys.append(rf * math.sin(theta))
    return xs, ys


def main() -> int:
    parser = argparse.ArgumentParser(description="Visualize the robot's 2D lidar scan.")
    parser.add_argument("--ip", default="127.0.0.1", help="Robot IP (default: 127.0.0.1)")
    parser.add_argument("--port", type=int, default=9090, help="rosbridge port (default: 9090)")
    parser.add_argument("--namespace", default="", help="ROS namespace (default: none)")
    parser.add_argument("--timeout", type=float, default=10.0, help="First-scan wait timeout in seconds")
    parser.add_argument("--once", action="store_true", help="Plot a single snapshot instead of animating")
    parser.add_argument("--save", default=None, help="Save the rendered figure to this path")
    args = parser.parse_args()

    print(f"Connecting to {args.ip}:{args.port} (namespace={args.namespace!r}) ...")
    bot = WalkieRobot(
        ip=args.ip,
        ros_port=args.port,
        camera_protocol="none",
        namespace=args.namespace,
    )

    try:
        print(f"Scan topic: {bot.lidar.scan_topic}")
        print(f"Waiting for first scan (timeout={args.timeout}s) ...")
        scan = bot.lidar.get_once(timeout=args.timeout)
        if scan is None:
            print("[FAIL] No scan received — is the lidar publishing on the scan topic?")
            return 1

        n = len(scan.get("ranges") or [])
        print("[OK] Scan received.")
        print(f"  frame_id   : {scan.get('header', {}).get('frame_id')}")
        print(f"  beams      : {n}")
        print(f"  angle      : [{scan.get('angle_min'):.3f}, {scan.get('angle_max'):.3f}] rad")
        print(f"  range      : [{scan.get('range_min')}, {scan.get('range_max')}] m")

        _display(bot, scan, args)
        return 0
    finally:
        bot.disconnect()


def _display(bot, scan, args):
    try:
        import matplotlib.pyplot as plt
    except ImportError:
        print("[FAIL] matplotlib not installed. Run: uv pip install matplotlib")
        return

    rmax = scan.get("range_max") or 10.0
    try:
        lim = float(rmax)
        if math.isinf(lim) or lim <= 0:
            lim = 10.0
    except (TypeError, ValueError):
        lim = 10.0

    fig, ax = plt.subplots(figsize=(8, 8))
    ax.set_aspect("equal")
    ax.set_xlim(-lim, lim)
    ax.set_ylim(-lim, lim)
    ax.set_xlabel("x (m) — forward")
    ax.set_ylabel("y (m) — left")
    ax.grid(True, linestyle=":", alpha=0.5)

    # Robot marker at the origin.
    ax.plot(0, 0, "b^", markersize=12, label="robot")
    (scatter,) = ax.plot([], [], "r.", markersize=2, label="scan")
    ax.legend(loc="upper right")

    def _draw(s):
        xs, ys = _scan_to_xy(s)
        scatter.set_data(xs, ys)
        frame = s.get("header", {}).get("frame_id", "")
        ax.set_title(f"Walkie 2D lidar — {len(xs)} pts (frame: {frame})")

    _draw(scan)

    if args.save:
        fig.savefig(args.save, dpi=150, bbox_inches="tight")
        print(f"[OK] Saved figure to {args.save}")

    if args.once:
        plt.show()
        return

    # Live mode: poll for new scans and redraw until the window is closed.
    print("Animating live scans. Close the plot window to stop.")
    plt.ion()
    plt.show()
    try:
        while plt.fignum_exists(fig.number):
            s = bot.lidar.get_scan()
            if s is not None:
                _draw(s)
            fig.canvas.draw_idle()
            plt.pause(0.1)
    except KeyboardInterrupt:
        print("\nInterrupted.")


if __name__ == "__main__":
    sys.exit(main())
