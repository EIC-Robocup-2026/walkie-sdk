"""
CLI entrypoint for the Walkie web interface.

``walkie-web`` (or ``python -m walkie_sdk.web``) starts a uvicorn server that
serves the dashboard and the ``/api/*`` routes. Connecting to a robot is
normally done from the browser, but ``--ip`` will auto-connect on startup.
"""

from __future__ import annotations

import argparse
import socket
import subprocess

from walkie_sdk.web.app import create_app
from walkie_sdk.web.state import session

# Virtual / container interfaces whose addresses outsiders can't reach.
_SKIP_IFACE_PREFIXES = (
    "lo", "docker", "br-", "veth", "virbr", "vmnet", "tun", "tap", "zt", "wg",
)


def _local_ips() -> list[str]:
    """
    Best-effort list of this machine's *externally reachable* LAN IPv4 addresses.

    Enumerates every network interface (via the Linux ``ip`` command) and keeps
    real ones only — so a machine bridging two networks (e.g. internet on Wi-Fi
    plus a separate robot LAN on Ethernet) lists *both* addresses, not just the
    default-route one. Loopback and virtual/container interfaces (docker, bridges,
    VPNs…) are skipped because peers can't connect to those.

    Falls back to the default-route source address if ``ip`` is unavailable
    (e.g. non-Linux hosts).
    """
    ips: list[str] = []

    try:
        out = subprocess.run(
            ["ip", "-4", "-o", "addr", "show", "scope", "global"],
            capture_output=True, text=True, timeout=2,
        ).stdout
        for line in out.splitlines():
            # e.g. "2: wlp1s0    inet 10.0.0.201/24 brd ... scope global ..."
            parts = line.split()
            if len(parts) < 4 or parts[1].startswith(_SKIP_IFACE_PREFIXES):
                continue
            for i, tok in enumerate(parts):
                if tok == "inet" and i + 1 < len(parts):
                    ips.append(parts[i + 1].split("/")[0])
    except (OSError, subprocess.SubprocessError):
        pass

    if not ips:
        # Fallback: primary outbound IP (default route only). No packet is sent
        # for a UDP socket; this just picks the source address.
        try:
            s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            try:
                s.connect(("8.8.8.8", 80))
                ips.append(s.getsockname()[0])
            finally:
                s.close()
        except OSError:
            pass

    # De-dupe (preserve order), drop loopback.
    return list(dict.fromkeys(ip for ip in ips if not ip.startswith("127.")))


def _print_urls(host: str, port: int) -> None:
    """Print copy-pasteable URLs for reaching the dashboard."""
    lines = [f"\nWalkie web interface listening on port {port}:",
             f"  Local:   http://127.0.0.1:{port}"]
    if host == "0.0.0.0":
        ips = _local_ips()
        lines += [f"  Network: http://{ip}:{port}" for ip in ips]
        if not ips:
            lines.append("  (could not detect a LAN IP — check `ip -4 addr`)")
        elif len(ips) == 1:
            lines.append("  ↳ share the Network URL with others (the API has no auth)")
        else:
            lines.append("  ↳ share the URL on the SAME network as the other machine "
                         "(the API has no auth)")
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
    parser.add_argument("--ros-protocol", default="hybrid")
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
                # arm_mode=args.arm_mode,
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
