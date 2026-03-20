# walkie_sdk/config/zenoh_topics.py
import os

# Default topics, with the ability to override via Environment Variables
CAMERA_TOPICS = {
    "head": os.getenv("WALKIE_CAM_HEAD", "/zed_head/zed_node/rgb/color/rect/image"),
    "left": os.getenv("WALKIE_CAM_LEFT", "/walkie/camera/left"),
    "right": os.getenv("WALKIE_CAM_RIGHT", "/walkie/camera/right"),
}


