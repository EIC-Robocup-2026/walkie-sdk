"""
Web interface for the Walkie SDK.

Exposes a small FastAPI application that holds a single server-side
``WalkieRobot`` connection and lets a browser drive it (navigation, lift,
arm/gripper, camera) over plain HTTP. The SDK itself stays pure-Python;
the web layer is an optional extra installed via ``walkie-sdk[web]``.

Run it with::

    walkie-web --host 0.0.0.0 --port 8080
    # or
    python -m walkie_sdk.web --port 8080
"""

from walkie_sdk.web.app import create_app
from walkie_sdk.web.state import RobotSession, RobotNotConnected, session

__all__ = ["create_app", "RobotSession", "RobotNotConnected", "session"]
