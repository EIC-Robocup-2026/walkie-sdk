#!/usr/bin/env python3
import sys
import time
import cv2
from walkie_sdk import WalkieRobot


def main():
    print("--- 2. Camera & Multi-Camera ---")
    bot = WalkieRobot(ip="127.0.0.1", camera_protocol="zenoh")

    # --- Single Camera Interface ---
    if bot.camera:
        print(f"Streaming: {bot.camera.is_streaming}")
        print(f"Shape: {bot.camera.frame_shape}")
        
        t_end = time.time() + 10  # Read telemetry for 10 seconds
        while time.time() < t_end:
            frame = bot.camera.get_frame()
            print(f"Got frame: {frame is not None}")
            if frame is not None:
                cv2.imshow("Camera Feed", frame)
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break

            
        bot.camera.stop()
        bot.camera.start()
    # --- Multi-Camera Interface ---
    # if bot.cameras:
    #     print(f"Available Cameras: {bot.cameras.camera_names}")
    #     print(f"Any Streaming: {bot.cameras.is_streaming}")
        
    #     head_shape = bot.cameras.get_frame_shape("head")
    #     head_frame = bot.cameras.get_frame("head")
        
    #     all_frames = bot.cameras.get_all_frames() # Returns dict of frames
    #     print(f"Got frames for: {list(all_frames.keys())}")

    #     bot.cameras.stop()
    #     bot.cameras.start()

    bot.disconnect()

if __name__ == "__main__":
    main()