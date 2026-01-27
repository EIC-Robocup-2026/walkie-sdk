from dataclasses import dataclass
from lerobot.robots.config import RobotConfig

@dataclass
class WalkieConfig(RobotConfig):
    ip: str = "192.168.1.100"
    ros_port: int = 9090
    camera_port: int = 8554
    ws_port: int | None = None
    webrtc_port: int | None = None
    enable_camera: bool = True
    mock: bool = False

    def __post_init__(self):
        super().__post_init__()
        # Handle legacy
        if self.ws_port is not None:
            self.ros_port = self.ws_port
        if self.webrtc_port is not None:
            self.camera_port = self.webrtc_port

    @property
    def type(self) -> str:
        return "walkie"
