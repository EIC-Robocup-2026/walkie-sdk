"""
Utility functions for the Walkie SDK.
"""

from walkie_sdk.utils.converters import (
    quaternion_to_euler,
    euler_to_quaternion,
)
from walkie_sdk.utils.namespace import apply_namespace

__all__ = ["quaternion_to_euler", "euler_to_quaternion", "apply_namespace"]
