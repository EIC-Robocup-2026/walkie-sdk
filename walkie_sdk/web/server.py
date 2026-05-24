"""
CLI entrypoint for the Walkie web interface.

``walkie-web`` (or ``python -m walkie_sdk.web``) starts a uvicorn server that
serves the dashboard and the ``/api/*`` routes. Connecting to a robot is
normally done from the browser, but ``--ip`` will auto-connect on startup.
"""

from __future__ import annotations

import argparse

from walkie_sdk.web.app import create_app
from walkie_sdk.web.state import session


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
    print(f"Walkie web interface → http://{args.host}:{args.port}")
    uvicorn.run(app, host=args.host, port=args.port, reload=args.reload)


if __name__ == "__main__":
    main()
