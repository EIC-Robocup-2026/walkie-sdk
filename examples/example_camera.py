#!/usr/bin/env python3
"""
Walkie SDK - Camera Feed Example

Demonstrates how to get and display camera frames from the robot.
Uses OpenCV to display the video feed in a window.

Usage:
    uv run python examples/example_camera.py

Requirements:
    - Robot running WebRTC camera server on port 8554
    - OpenCV installed (opencv-python)

Controls:
    - Press 'q' to quit
    - Press 's' to save a snapshot
"""

import sys
import time

import cv2

# Configuration - Change this to your robot's IP
ROBOT_IP = "127.0.0.1"
NAMESPACE = ""  # Optional: "robot1" for namespaced topics

# Protocol selection
ROS_PROTOCOL = "rosbridge"
CAMERA_PROTOCOL = "webrtc"


def main():
    print("=" * 60)
    print("Walkie SDK - Camera Feed Example")
    print("=" * 60)

    from walkie_sdk import WalkieRobot

    # 1. Connect with camera enabled
    print(f"\n[1] Connecting to {ROBOT_IP}...")
    print(f"    ROS Protocol: {ROS_PROTOCOL}")
    print(f"    Camera Protocol: {CAMERA_PROTOCOL}")

    try:
        bot = WalkieRobot(
            ip=ROBOT_IP,
            ros_protocol=ROS_PROTOCOL,
            ros_port=9090,
            camera_protocol=CAMERA_PROTOCOL,
            camera_port=8554,
            timeout=10.0,
            namespace=NAMESPACE,
        )
    except ConnectionError as e:
        print(f"❌ Connection failed: {e}")
        sys.exit(1)

    print(f"    Using: {bot.ros_protocol} protocol")
    print(f"    Camera: {bot.camera_protocol} protocol")

    # Check if camera is available
    if bot.camera is None:
        print("❌ Camera is not available!")
        bot.disconnect()
        sys.exit(1)

    # 2. Wait for camera stream to start
    print("\n[2] Waiting for camera stream...")
    wait_start = time.time()
    while not bot.camera.is_streaming:
        if time.time() - wait_start > 10.0:
            print("❌ Camera stream timeout!")
            bot.disconnect()
            sys.exit(1)
        time.sleep(0.1)

    print(f"  ✓ Camera streaming!")

    # 3. Display camera feed
    print("\n[3] Displaying camera feed...")
    print("    Press 'q' to quit")
    print("    Press 's' to save snapshot")

    # Create window
    window_name = "Walkie Robot Camera"
    cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)

    frame_count = 0
    fps_start_time = time.time()
    fps = 0.0
    snapshot_count = 0

    try:
        while True:
            # Get latest frame
            frame = bot.camera.get_frame()

            if frame is not None:
                frame_count += 1

                # Calculate FPS every second
                elapsed = time.time() - fps_start_time
                if elapsed >= 1.0:
                    fps = frame_count / elapsed
                    frame_count = 0
                    fps_start_time = time.time()

                # Add overlay information
                display_frame = frame.copy()

                # Add FPS counter
                cv2.putText(
                    display_frame,
                    f"FPS: {fps:.1f}",
                    (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    1.0,
                    (0, 255, 0),
                    2,
                )

                # Add frame shape info
                h, w = frame.shape[:2]
                cv2.putText(
                    display_frame,
                    f"Resolution: {w}x{h}",
                    (10, 70),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    1.0,
                    (0, 255, 0),
                    2,
                )

                # Add robot pose if available
                pose = bot.status.get_pose()
                if pose:
                    cv2.putText(
                        display_frame,
                        f"Pose: x={pose['x']:.2f} y={pose['y']:.2f} θ={pose['heading']:.2f}",
                        (10, 110),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.7,
                        (255, 255, 0),
                        2,
                    )

                # Show frame
                cv2.imshow(window_name, display_frame)

            else:
                # No frame available, show placeholder
                placeholder = cv2.imread("placeholder.png") if False else None
                if placeholder is None:
                    # Create a black placeholder
                    placeholder = cv2.putText(
                        (480, 640, 3),
                        "Waiting for frame...",
                        (50, 240),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        1.0,
                        (255, 255, 255),
                        2,
                    )

            # Handle keyboard input
            key = cv2.waitKey(1) & 0xFF

            if key == ord("q"):
                print("\n  Quitting...")
                break

            elif key == ord("s"):
                # Save snapshot
                if frame is not None:
                    snapshot_count += 1
                    filename = f"snapshot_{snapshot_count:03d}.jpg"
                    cv2.imwrite(filename, frame)
                    print(f"  📸 Saved: {filename}")

    except KeyboardInterrupt:
        print("\n  Interrupted by user")

    finally:
        # Cleanup
        cv2.destroyAllWindows()

    # 4. Disconnect
    print("\n[4] Disconnecting...")
    bot.disconnect()

    print("\n✓ Example completed!")


if __name__ == "__main__":
    main()
