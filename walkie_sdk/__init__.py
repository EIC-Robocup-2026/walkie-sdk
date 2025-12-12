"""
Walkie SDK - Python SDK for controlling Walkie robots.

A pure Python library that wraps WebSocket (ROSBridge) and WebRTC protocols
into simple function calls. No ROS 2 installation required on the client.

Example:
    from walkie_sdk import WalkieRobot

    bot = WalkieRobot(ip="192.168.1.100")
    print(f"Pose: {bot.status.get_pose()}")
    bot.nav.go_to(x=2.0, y=1.0, heading=0.0)
"""

from walkie_sdk.robot import WalkieRobot

__version__ = "0.1.0"
__all__ = ["WalkieRobot", "__version__"]
