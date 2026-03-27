#!/usr/bin/env python3
import time
from walkie_sdk import WalkieRobot

def main():
    print("--- 6. Telemetry ---")
    bot = WalkieRobot(ip="127.0.0.1", camera_protocol="none")
    time.sleep(1) # Wait for data to arrive

    # 1. Properties
    print(f"Has Data: {bot.status.has_data}")
    print(f"Odom Topic: {bot.status.odom_topic}")
    print(f"Point Cloud Topic: {bot.status.zed_point_cloud_topic}")

    if bot.status.has_data:
        # 2. Odometry & Velocity
        print(f"Position (x,y,heading): {bot.status.get_position()}")
        print(f"Velocity (linear,angular): {bot.status.get_velocity()}")
        print(f"Raw Odometry Dict: {bot.status.get_raw_odom() is not None}")

    # 3. Point Cloud
    t_end = time.time() + 5  # Read telemetry for 10 seconds
    while time.time() < t_end:
        pc_info = bot.status.get_point_cloud_info()
        if pc_info:
            print(f"Point Cloud Meta: {pc_info['width']}x{pc_info['height']}")
        
        t0 = time.time()
        pc_data = bot.status.get_full_point_cloud()
        t1 = time.time()
        if pc_data:
            print(f"    Extracted  : {len(pc_data)} points (took {(t1-t0)*1000:.1f} ms)")
            print(f"    First 3 Pts: {pc_data[:3]}")
            print(f"    Last 3 Pts : {pc_data[-3:]}")

    # 4. Lifecycle Control (usually handled automatically by WalkieRobot)
    bot.status.stop()
    bot.status.start()

    bot.disconnect()

if __name__ == "__main__":
    main()