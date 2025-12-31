"""
Core clients for robot communication.

- BridgeClient: WebSocket connection via ROSBridge
- WebRTCClient: Video streaming via WebRTC
"""

from walkie_sdk.core.bridge_client import BridgeClient

__all__ = ["BridgeClient"]
