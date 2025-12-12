#!/usr/bin/env python3
"""
Walkie SDK - Example WITH Camera

Demonstrates camera streaming with OpenCV display.
Requires: opencv-python (cv2)

Usage:
    uv run python examples/example_camera.py
"""

import sys
import time

# Configuration - Change this to your robot's IP
ROBOT_IP = "127.0.0.1"
NAMESPACE = ""


def main():
    print("=" * 60)
    print("Walkie SDK - Camera Example")
    print("=" * 60)

    # Check OpenCV is available
    try:
        import cv2
    except ImportError:
        print("❌ OpenCV not installed. Run: uv add opencv-python")
        sys.exit(1)

    from walkie_sdk import WalkieRobot

    # 1. Connect with camera enabled
    print(f"\n[1] Connecting to {ROBOT_IP} with camera...")

    try:
        bot = WalkieRobot(
            ip=ROBOT_IP,
            ws_port=9090,
            webrtc_port=8554,
            timeout=10.0,
            enable_camera=True,  # <-- Camera enabled
            namespace=NAMESPACE,
        )
    except ConnectionError as e:
        print(f"❌ Connection failed: {e}")
        sys.exit(1)

    # 2. Check camera status
    print("\n[2] Camera status:")
    if bot.camera is None:
        print("  ⚠ Camera not available")
        bot.disconnect()
        sys.exit(1)

    print(f"  📷 Streaming: {bot.camera.is_streaming}")

    # Wait for first frame
    print("  Waiting for first frame...")
    for _ in range(20):
        frame = bot.camera.get_frame()
        if frame is not None:
            print(f"  ✓ Frame received: {frame.shape}")
            break
        time.sleep(0.1)
    else:
        print("  ⚠ No frames received")

    # 3. Live camera display
    print("\n[3] Displaying camera feed (press 'q' to quit)...")
    print("  Controls:")
    print("    q - Quit")
    print("    s - Stop robot")
    print("    p - Print pose")

    frame_count = 0
    start_time = time.time()

    while True:
        # Get frame
        frame = bot.camera.get_frame()

        if frame is not None:
            frame_count += 1

            # Add telemetry overlay
            pose = bot.status.get_pose()
            if pose:
                text = f"x:{pose['x']:.2f} y:{pose['y']:.2f} h:{pose['heading']:.2f}"
                cv2.putText(
                    frame, text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2
                )

            # Add FPS
            elapsed = time.time() - start_time
            fps = frame_count / elapsed if elapsed > 0 else 0
            cv2.putText(
                frame,
                f"FPS: {fps:.1f}",
                (10, 60),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (0, 255, 0),
                2,
            )

            # Show frame
            cv2.imshow("Walkie Robot Camera", frame)

        # Handle keyboard
        key = cv2.waitKey(1) & 0xFF

        if key == ord("q"):
            print("\n  Quit requested")
            break
        elif key == ord("s"):
            print("  🛑 Emergency stop!")
            bot.nav.stop()
        elif key == ord("p"):
            pose = bot.status.get_pose()
            if pose:
                print(
                    f"  📍 Pose: x={pose['x']:.3f}, y={pose['y']:.3f}, θ={pose['heading']:.3f}"
                )

    # 4. Cleanup
    cv2.destroyAllWindows()

    print("\n[4] Disconnecting...")
    bot.disconnect()

    elapsed = time.time() - start_time
    fps = frame_count / elapsed if elapsed > 0 else 0
    print(f"\n✓ Captured {frame_count} frames in {elapsed:.1f}s ({fps:.1f} FPS)")


if __name__ == "__main__":
    main()
