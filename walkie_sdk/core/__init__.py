"""
Walkie SDK - Core Module

This module provides core infrastructure for SDK:
- Abstract interfaces defining transport contracts
- Transport factory for creating protocol-specific implementations
- Backward compatibility aliases

The core module abstracts away the underlying communication protocol,
allowing SDK to work with different backends (rosbridge, zenoh).
"""

from walkie_sdk.core.factory import (
    CameraProtocol,
    ROSProtocol,
    TransportFactory,
)
from walkie_sdk.core.interfaces import (
    CameraTransportInterface,
    ROSTransportInterface,
)


# Backward compatibility: BridgeClient alias for ROSBridgeTransport
# This allows existing code using BridgeClient to continue working
def _get_bridge_client():
    """Lazy import for backward compatibility."""
    from walkie_sdk.core.transports.rosbridge import ROSBridgeTransport

    return ROSBridgeTransport


# Create a lazy property for BridgeClient
class _BridgeClientAlias:
    """Backward compatibility alias for BridgeClient -> ROSBridgeTransport."""

    _class = None

    def __new__(cls, *args, **kwargs):
        if cls._class is None:
            from walkie_sdk.core.transports.rosbridge import ROSBridgeTransport

            cls._class = ROSBridgeTransport
        return cls._class(*args, **kwargs)


# Alias for backward compatibility
BridgeClient = _BridgeClientAlias

__all__ = [
    # Interfaces
    "ROSTransportInterface",
    "CameraTransportInterface",
    # Factory
    "TransportFactory",
    "ROSProtocol",
    "CameraProtocol",
    # Backward compatibility
    "BridgeClient",
]
