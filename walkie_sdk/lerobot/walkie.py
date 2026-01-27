import torch
import numpy as np
import time

from lerobot.robots.robot import Robot
from lerobot.processor import RobotAction, RobotObservation

from walkie_sdk.lerobot.config_walkie import WalkieConfig
from walkie_sdk.robot import Walkie as WalkieSDKRobot

class Walkie(Robot):
    config_class = WalkieConfig
    name = "walkie"

    def __init__(self, config: WalkieConfig):
        super().__init__(config)
        self.config = config
        self.robot: WalkieSDKRobot | None = None
        self._connected = False

    @property
    def observation_features(self) -> dict:
        return {
            "observation.state": (16,),
            "observation.images.head": (480, 640, 3),
            "observation.images.left_wrist": (480, 640, 3),
            "observation.images.right_wrist": (480, 640, 3),
        }

    @property
    def action_features(self) -> dict:
        return {
            "action": (16,)
        }

    @property
    def is_connected(self) -> bool:
        return self._connected

    def connect(self, calibrate: bool = True) -> None:
        if self._connected:
            return

        if self.config.mock:
            print("Connecting to Mock Walkie Robot...")
            self._connected = True
            return

        print(f"Connecting to Walkie Robot at {self.config.ip}...")
        self.robot = WalkieSDKRobot(
            ip=self.config.ip,
            ros_port=self.config.ros_port,
            camera_port=self.config.camera_port,
            enable_camera=self.config.enable_camera
        )
        self._connected = True
        
        if calibrate:
             self.calibrate()

    def disconnect(self) -> None:
        if not self._connected:
            return
            
        if self.robot:
            self.robot.disconnect()
            self.robot = None
            
        self._connected = False

    def is_calibrated(self) -> bool:
        return True

    def calibrate(self) -> None:
        pass

    def configure(self) -> None:
        pass

    def get_observation(self) -> RobotObservation:
        if not self._connected:
            raise ConnectionError("Robot is not connected")

        if self.config.mock:
            # Return mock data
            return {
                "observation.state": torch.zeros(16),
                "observation.images.head": torch.zeros((3, 480, 640)),
                "observation.images.left_wrist": torch.zeros((3, 480, 640)),
                "observation.images.right_wrist": torch.zeros((3, 480, 640)),
            }

        states = self.robot.arm.get_joint_states()
        if states is None:
            timestamp = time.time()
            state_vec = torch.zeros(16)
        else:
            left_pos = states["left_arm"]["positions"]
            left_grip = [states["left_gripper"] if states["left_gripper"] is not None else 0.0]
            right_pos = states["right_arm"]["positions"]
            right_grip = [states["right_gripper"] if states["right_gripper"] is not None else 0.0]
            
            flat_state = left_pos + left_grip + right_pos + right_grip
            state_vec = torch.tensor(flat_state, dtype=torch.float32)

        # Get images
        # LeRobot expects (C, H, W) tensors usually, but observation_features says (H, W, C)
        # Walkie SDK returns numpy (H, W, C) BGR (opencv)
        
        obs = {
            "observation.state": state_vec
        }
        
        if self.robot.cameras:
            frames = self.robot.cameras.get_all_frames()
            # Keys: head, left, right

            for key, cam_name in [
                ("observation.images.head", "head"),
                ("observation.images.left_wrist", "left"),
                ("observation.images.right_wrist", "right")
            ]:
                frame = frames.get(cam_name)
                if frame is not None:
                    # BGR to RGB
                    # H, W, C -> C, H, W
                    frame = frame[..., ::-1].copy()
                    frame_t = torch.from_numpy(frame).permute(2, 0, 1)
                    obs[key] = frame_t
                else:
                    obs[key] = torch.zeros((3, 480, 640)) # Placeholder
        
        return obs

    def send_action(self, action: RobotAction) -> RobotAction:
        if not self._connected:
             raise ConnectionError("Robot is not connected")

        if self.config.mock:
             return action

        if isinstance(action, dict) and "action" in action:
            vec = action["action"]
        else:
            vec = action
            # Safety check
            if not isinstance(vec, (torch.Tensor, np.ndarray)):
                if isinstance(action, dict):
                    vec = next(iter(action.values()))

        if isinstance(vec, torch.Tensor):
            vec = vec.tolist()
        
        # Left(7) + L_Grip(1) + Right(7) + R_Grip(1)
        if len(vec) != 16:
            print(f"Warning: Action vector length {len(vec)} != 16")
            return action

        left_arm = vec[0:7]
        left_grip = vec[7]
        right_arm = vec[8:15]
        right_grip = vec[15]

        # Send to SDK
        self.robot.arm.set_joint_positions(
            left_arm=left_arm,
            right_arm=right_arm,
            left_gripper=left_grip,
            right_gripper=right_grip,
            blocking=False
        )
        
        return action
