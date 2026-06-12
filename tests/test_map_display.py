"""
Interactive map display test for the Navigation map helpers against a live robot.

Fetches the robot's occupancy grid via bot.nav.get_map_image(), prints its
metadata, and renders it with matplotlib. If the robot's position is available
it overlays a red arrow at the robot's pixel location pointing along its heading.

Requires a running robot publishing the map (service nav_msgs/srv/GetMap or topic
nav_msgs/msg/OccupancyGrid). matplotlib is needed for display:

    uv pip install matplotlib

Usage:
    python tests/test_map_display.py --ip 192.168.1.100
    python tests/test_map_display.py --ip 192.168.1.100 --namespace robot1
    python tests/test_map_display.py --ip 192.168.1.100 --timeout 10 --save /tmp/map.png
"""

import argparse
import math
import sys

from walkie_sdk import WalkieRobot


def main() -> int:
    parser = argparse.ArgumentParser(description="Display the robot's occupancy grid map.")
    parser.add_argument("--ip", default="127.0.0.1", help="Robot IP (default: 127.0.0.1)")
    parser.add_argument("--port", type=int, default=9090, help="rosbridge port (default: 9090)")
    parser.add_argument("--namespace", default="", help="ROS namespace (default: none)")
    parser.add_argument("--timeout", type=float, default=10.0, help="Map fetch timeout in seconds")
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
        print(f"Fetching map (timeout={args.timeout}s) ...")
        img = bot.nav.get_map_image(timeout=args.timeout)
        if img is None:
            print("[FAIL] No map received — is the map server / map topic active?")
            return 1

        meta = bot.nav.map_metadata
        print("[OK] Map received.")
        print(f"  resolution : {meta['resolution']} m/px")
        print(f"  size       : {meta['width']} x {meta['height']} px")
        print(f"  origin     : ({meta['origin_x']:.3f}, {meta['origin_y']:.3f}) m")

        pos = bot.status.get_position()
        if pos is None:
            print("[warn] No robot position available — drawing map without pose overlay.")
        else:
            print(f"  robot pose : x={pos['x']:.3f}  y={pos['y']:.3f}  heading={pos['heading']:.3f} rad")

        _display(bot, img, meta, pos, args.save)
        return 0
    finally:
        bot.disconnect()


def _display(bot, img, meta, pos, save_path):
    try:
        import matplotlib.pyplot as plt
        from matplotlib.ticker import FuncFormatter
    except ImportError:
        print("[FAIL] matplotlib not installed. Run: uv pip install matplotlib")
        return

    res = meta["resolution"]
    ox, oy = meta["origin_x"], meta["origin_y"]
    height = meta["height"]

    fig, ax = plt.subplots(figsize=(8, 8))
    ax.imshow(img, cmap="gray", origin="upper")
    ax.set_title("Walkie occupancy grid")

    # Axis ticks in metres (map frame) instead of pixels.
    ax.xaxis.set_major_formatter(FuncFormatter(lambda px, _: f"{ox + px * res:.1f}"))
    ax.yaxis.set_major_formatter(FuncFormatter(lambda py, _: f"{oy + (height - 1 - py) * res:.1f}"))
    ax.set_xlabel("x (m)")
    ax.set_ylabel("y (m)")

    if pos is not None:
        try:
            px, py = bot.nav.map_to_pixel(pos["x"], pos["y"])
        except Exception as e:
            print(f"[warn] map_to_pixel failed: {e}")
            px = py = None
        if px is not None:
            scale = max(meta["width"], height) * 0.08
            dx = math.cos(pos["heading"]) * scale
            dy = -math.sin(pos["heading"]) * scale  # image y is flipped
            ax.annotate(
                "",
                xy=(px + dx, py + dy),
                xytext=(px, py),
                arrowprops=dict(facecolor="red", edgecolor="red", width=2, headwidth=10),
            )
            ax.plot(px, py, "ro", markersize=5)

    if save_path:
        fig.savefig(save_path, dpi=150, bbox_inches="tight")
        print(f"[OK] Saved figure to {save_path}")

    plt.show()


if __name__ == "__main__":
    sys.exit(main())
