"""
Core clients for robot communication.

- BridgeClient: WebSocket connection via ROSBridge
- WebRTCClient: Video streaming via WebRTC
"""

from walkie_sdk.core.bridge_client import BridgeClient
from walkie_sdk.core.webrtc_client import WebRTCClient

__all__ = ["BridgeClient", "WebRTCClient"]
