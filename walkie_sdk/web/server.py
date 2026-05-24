"""
CLI entrypoint for the Walkie web interface.

``walkie-web`` (or ``python -m walkie_sdk.web``) starts a uvicorn server that
serves the dashboard and the ``/api/*`` routes. Connecting to a robot is
normally done from the browser, but ``--ip`` will auto-connect on startup.
"""

from __future__ import annotations

import argparse
import socket

from walkie_sdk.web.app import create_app
from walkie_sdk.web.state import session


def _local_ips() -> list[str]:
    """Best-effort list of this machine's LAN IPv4 addresses (no loopback)."""
    ips: set[str] = set()

    # Primary outbound IP (the one on the active default route). No packet is
    # actually sent for a UDP socket; this just picks the source address.
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            s.connect(("8.8.8.8", 80))
            ips.add(s.getsockname()[0])
        finally:
            s.close()
    except OSError:
        pass

    # Anything else resolvable from the hostname.
    try:
        for info in socket.getaddrinfo(socket.gethostname(), None, socket.AF_INET):
            ips.add(info[4][0])
    except OSError:
        pass

    return sorted(ip for ip in ips if not ip.startswith("127."))


def _print_urls(host: str, port: int) -> None:
    """Print copy-pasteable URLs for reaching the dashboard."""
    lines = [f"\nWalkie web interface listening on port {port}:",
             f"  Local:   http://127.0.0.1:{port}"]
    if host == "0.0.0.0":
        lines += [
            f"  Network: http://{ip}:{port}   ← share with others on the LAN"
            for ip in _local_ips()
        ]
        lines.append("  (others must be on the same network; the API has no auth)")
    elif host not in ("127.0.0.1", "localhost"):
        lines.append(f"  Network: http://{host}:{port}")
    # flush so the banner shows immediately even when stdout is redirected/piped.
    print("\n".join(lines) + "\n", flush=True)


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        prog="walkie-web",
        description="Serve the Walkie SDK web control panel.",
    )
    parser.add_argument("--host", default="127.0.0.1", help="Bind address (default: 127.0.0.1)")
    parser.add_argument("--port", type=int, default=8080, help="Bind port (default: 8080)")
    parser.add_argument("--reload", action="store_true", help="Enable auto-reload (dev)")

    # Optional auto-connect on startup (otherwise connect from the browser).
    parser.add_argument("--ip", default=None, help="Robot IP to auto-connect on startup")
    parser.add_argument("--ros-protocol", default="rosbridge")
    parser.add_argument("--ros-port", type=int, default=9090)
    parser.add_argument("--camera-protocol", default="zenoh")
    parser.add_argument("--camera-port", type=int, default=7447)
    parser.add_argument("--namespace", default="")
    parser.add_argument("--arm-mode", default="custom_ik")
    return parser


def main(argv: list[str] | None = None) -> None:
    import uvicorn

    args = build_parser().parse_args(argv)

    if args.ip:
        try:
            session.connect(
                ip=args.ip,
                ros_protocol=args.ros_protocol,
                ros_port=args.ros_port,
                camera_protocol=args.camera_protocol,
                camera_port=args.camera_port,
                namespace=args.namespace,
                arm_mode=args.arm_mode,
            )
            print(f"✓ Auto-connected to robot at {args.ip}")
        except Exception as e:
            print(f"⚠ Auto-connect to {args.ip} failed: {e}")
            print("  Start the server anyway; connect from the browser.")

    app = create_app()
    _print_urls(args.host, args.port)
    uvicorn.run(app, host=args.host, port=args.port, reload=args.reload)


if __name__ == "__main__":
    main()
