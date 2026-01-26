#!/usr/bin/env python3
"""
Walkie SDK - Protocol Selection Example

Demonstrates how to use different communication protocols with the SDK.

Available ROS Protocols:
- "rosbridge": WebSocket via roslibpy (default, no ROS2 required on client)
- "zenoh": Zenoh DDS bridge (no ROS2 required on client)
- "auto": Auto-detect best available protocol

Available Camera Protocols:
- "webrtc": WebRTC stream (default, pairs with rosbridge)
- "zenoh": Zenoh video stream (pairs with zenoh)
- "shm": Shared memory (same-host)
- "none": Disable camera functionality

Usage:
    uv run python examples/example_protocols.py
"""

import sys

# Configuration - Change this to your robot's IP
ROBOT_IP = "127.0.0.1"


def example_rosbridge():
    """
    Example 1: ROSBridge (WebSocket) - Default Protocol

    This is the default protocol that requires no ROS2 installation on the client.
    It connects via WebSocket to a rosbridge_server running on the robot.
    """
    from walkie_sdk import WalkieRobot

    print("\n" + "=" * 60)
    print("Example 1: ROSBridge (WebSocket) Protocol")
    print("=" * 60)
    print("This protocol uses WebSocket to communicate with rosbridge_server.")
    print("No ROS2 installation required on the client machine.")

    try:
        # Default protocol - rosbridge with webrtc camera
        bot = WalkieRobot(
            ip=ROBOT_IP,
            ros_protocol="rosbridge",  # WebSocket via roslibpy
            ros_port=9090,  # ROSBridge WebSocket port
            camera_protocol="webrtc",  # WebRTC video stream
            camera_port=8554,  # WebRTC signaling port
            timeout=10.0,
        )

        print(f"✓ Connected using {bot.ros_protocol} protocol")
        print(f"  Camera protocol: {bot.camera_protocol}")

        # Use the robot...
        pose = bot.status.get_pose()
        if pose:
            print(f"  Robot pose: x={pose['x']:.2f}, y={pose['y']:.2f}")

        bot.disconnect()
        return True

    except ConnectionError as e:
        print(f"✗ Connection failed: {e}")
        return False


def example_rosbridge_no_camera():
    """
    Example 2: ROSBridge without Camera

    Useful when the WebRTC camera server is not available.
    """
    from walkie_sdk import WalkieRobot

    print("\n" + "=" * 60)
    print("Example 2: ROSBridge without Camera")
    print("=" * 60)
    print("Connecting without camera for navigation/telemetry only.")

    try:
        bot = WalkieRobot(
            ip=ROBOT_IP,
            ros_protocol="rosbridge",
            ros_port=9090,
            camera_protocol="none",  # Disable camera
            timeout=10.0,
        )

        print(f"✓ Connected using {bot.ros_protocol} protocol")
        print(f"  Camera: disabled (camera_protocol='none')")

        # Camera is None when disabled
        if bot.camera is None:
            print("  bot.camera is None (as expected)")

        bot.disconnect()
        return True

    except ConnectionError as e:
        print(f"✗ Connection failed: {e}")
        return False


def example_zenoh():
    """
    Example 3: Zenoh DDS Bridge

    This protocol uses Zenoh for communication, providing good performance
    without requiring ROS2 on the client.
    """
    from walkie_sdk import WalkieRobot

    print("\n" + "=" * 60)
    print("Example 3: Zenoh DDS Bridge Protocol")
    print("=" * 60)
    print("This protocol uses Zenoh for low-latency communication.")
    print("No ROS2 installation required on the client machine.")

    try:
        bot = WalkieRobot(
            ip=ROBOT_IP,
            ros_protocol="zenoh",  # Zenoh DDS bridge
            ros_port=7447,  # Zenoh router port
            camera_protocol="zenoh",  # Zenoh video stream
            timeout=10.0,
        )

        print(f"✓ Connected using {bot.ros_protocol} protocol")
        bot.disconnect()
        return True

    except NotImplementedError as e:
        print(f"✗ Protocol not yet implemented: {e}")
        print("  (This is expected - zenoh transport is a stub)")
        return False
    except ConnectionError as e:
        print(f"✗ Connection failed: {e}")
        return False


def example_auto_detect():
    """
    Example 4: Auto-detect Protocol

    Automatically detects and uses the best available protocol.
    Tries: zenoh -> rosbridge
    """
    from walkie_sdk import WalkieRobot

    print("\n" + "=" * 60)
    print("Example 4: Auto-detect Protocol")
    print("=" * 60)
    print("Automatically selecting the best available protocol...")

    try:
        bot = WalkieRobot(
            ip=ROBOT_IP,
            ros_protocol="auto",  # Auto-detect best protocol
            timeout=10.0,
        )

        print(f"✓ Auto-detected protocol: {bot.ros_protocol}")
        bot.disconnect()
        return True

    except ConnectionError as e:
        print(f"✗ No protocol could connect: {e}")
        return False


def example_backward_compatible():
    """
    Example 5: Backward Compatible API

    Shows that the old API parameters still work for backward compatibility.
    """
    from walkie_sdk import WalkieRobot

    print("\n" + "=" * 60)
    print("Example 5: Backward Compatible API")
    print("=" * 60)
    print("Using legacy parameter names (ws_port, enable_camera)...")

    try:
        # Old API still works
        bot = WalkieRobot(
            ip=ROBOT_IP,
            ws_port=9090,  # Legacy: maps to ros_port
            enable_camera=False,  # Legacy: maps to camera_protocol="none"
            timeout=10.0,
        )

        print(f"✓ Connected using legacy API")
        print(f"  Protocol: {bot.ros_protocol}")
        print(f"  Camera: {'enabled' if bot.camera else 'disabled'}")

        bot.disconnect()
        return True

    except ConnectionError as e:
        print(f"✗ Connection failed: {e}")
        return False


def show_available_protocols():
    """Display all available protocol options."""
    from walkie_sdk import CameraProtocol, ROSProtocol

    print("\n" + "=" * 60)
    print("Available Protocols")
    print("=" * 60)

    print("\nROS Protocols (ros_protocol parameter):")
    for protocol in ROSProtocol:
        status = "✓ implemented" if protocol.value == "rosbridge" else "○ stub"
        print(f"  - '{protocol.value}': {status}")

    print("\nCamera Protocols (camera_protocol parameter):")
    for protocol in CameraProtocol:
        if protocol.value in ("webrtc", "none", "shm"):
            status = "✓ implemented"
        else:
            status = "○ stub"
        print(f"  - '{protocol.value}': {status}")


def main():
    print("=" * 60)
    print("Walkie SDK - Protocol Selection Examples")
    print("=" * 60)

    # Show available protocols
    show_available_protocols()

    # Run examples
    # Note: These will fail to connect without a real robot,
    # but demonstrate the API usage

    print("\n" + "-" * 60)
    print("Running Examples (connection errors expected without robot)")
    print("-" * 60)

    # Only run examples that don't require connection for demo purposes
    example_backward_compatible()  # Will fail without robot
    example_rosbridge_no_camera()  # Will fail without robot

    # These will show NotImplementedError (expected)
    example_zenoh()

    print("\n" + "=" * 60)
    print("Examples completed!")
    print("=" * 60)
    print("\nTo test with a real robot, update ROBOT_IP and run:")
    print("  uv run python examples/example_protocols.py")


if __name__ == "__main__":
    main()
