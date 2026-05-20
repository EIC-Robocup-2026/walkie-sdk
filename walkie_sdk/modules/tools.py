"""
Tools - Utility module for Object Detection and Coordinate Transformation.

Provides bboxes_to_positions() to convert 2D bounding boxes into 3D world
coordinates via the perception service.
"""

from typing import Any, Dict, Optional, List

from walkie_sdk.utils import converters
from walkie_sdk.core.interfaces import ROSTransportInterface
from walkie_sdk.utils.namespace import apply_namespace
from walkie_sdk.config.ros_topics import OB_POSE_SERVICE


class Tools:
    """
    Controller for auxiliary robot tools and vision processing.

    This module works with any transport implementation (rosbridge, zenoh).
    """

    def __init__(
        self,
        transport: ROSTransportInterface,
        namespace: str = "",
    ):
        self._transport = transport
        self._namespace = namespace

    @property
    def namespace(self) -> str:
        """Current ROS namespace."""
        return self._namespace

    @namespace.setter
    def namespace(self, value: str) -> None:
        """Set ROS namespace."""
        self._namespace = value

    def bboxes_to_positions(
        self, coords: List[List[float]], timeout: float = 5.0
    ) -> Optional[List]:
        """
        Convert 2D bounding boxes to 3D world positions via the perception service.

        Args:
            coords: List of 2D bounding boxes as [cx, cy, w, h].
                    Example: [[320, 240, 50, 50], ...]
            timeout: Service call timeout in seconds (default: 5.0).

        Returns:
            List of 3D positions, or None if the service call failed or timed out.
        """
        try:
            detections_msg = converters.convert_bboxes_to_detection_array(coords)
            response = self._transport.call_service(
                service_name=apply_namespace(OB_POSE_SERVICE["service_name"], self._namespace),
                service_type=OB_POSE_SERVICE["service_type"],
                request={"detections": detections_msg},
                timeout=timeout,
            )
            if not response.get("success", False):
                print("[Tools] Service returned success=False")
                return None
            return converters.convert_poses_to_array(response["poses"])
        except TimeoutError:
            print(f"[Tools] Service call timed out after {timeout}s")
            return None
        except Exception as e:
            print(f"[Tools] Error in bboxes_to_positions: {e}")
            return None
