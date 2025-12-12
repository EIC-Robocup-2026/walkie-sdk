"""
Camera - Robot camera/video stream module.

Provides get_frame() function to retrieve video frames from
the robot's WebRTC video stream.
"""

from typing import Optional

import numpy as np

from walkie_sdk.core.webrtc_client import WebRTCClient


class Camera:
    """
    Robot camera interface.
    
    Wraps the WebRTC client to provide simple frame access.
    
    Args:
        webrtc: WebRTCClient instance for video streaming
    """
    
    def __init__(self, webrtc: WebRTCClient):
        self._webrtc = webrtc
    
    def get_frame(self) -> Optional[np.ndarray]:
        """
        Get the latest camera frame.
        
        Returns:
            numpy array in BGR format (shape: HxWx3, dtype: uint8),
            compatible with OpenCV. Returns None if no frame is available.
        
        Example:
            >>> frame = bot.camera.get_frame()
            >>> if frame is not None:
            ...     cv2.imshow("Camera", frame)
            ...     cv2.waitKey(1)
        """
        return self._webrtc.get_frame()
    
    @property
    def is_streaming(self) -> bool:
        """Check if camera stream is active."""
        return self._webrtc.is_connected
    
    @property
    def frame_shape(self) -> Optional[tuple]:
        """
        Get the shape of camera frames.
        
        Returns:
            Tuple of (height, width, channels) or None if no frame available.
        """
        frame = self._webrtc.get_frame()
        if frame is not None:
            return frame.shape
        return None
