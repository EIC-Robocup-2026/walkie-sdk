# Troubleshooting

## Connection Errors

```
ConnectionError: Connection timeout after 10.0s.
Is ROSBridge running at 192.168.1.100:9090?
```

1. Verify robot IP is reachable: `ping 192.168.1.100`
2. Check ROSBridge is running: `ros2 node list | grep rosbridge`
3. Check firewall allows the port (9090 for rosbridge, 7447 for zenoh)
4. Try increasing timeout: `WalkieRobot(ip="...", timeout=30.0)`

## No Camera Frames

```
Camera connection failed: ...
Camera will not be available.
```

1. Verify the camera server is running on the robot
2. Check the port is accessible (8554 for WebRTC, 7447 for Zenoh)
3. Disable camera if not needed: `camera_protocol="none"`
4. Check `bot.camera` is not `None` before calling `get_frame()`

## No Odometry Data

```python
bot.status.get_pose()  # Returns None
```

1. Wait for the first odometry message (may take 100-200ms after connect)
2. Check the robot is publishing odometry: `ros2 topic echo /odom`
3. Verify the namespace is correct if using one

## USB Camera Not Found

```
Failed to open USB camera device: /dev/v4l/by-id/usb-...
```

1. Check the device exists: `ls /dev/v4l/by-id/`
2. Check permissions: `ls -la /dev/video*`
3. Add your user to the `video` group: `sudo usermod -aG video $USER` (then re-login)
4. Try a different device path or index
5. Test with the diagnostic script: `uv run python examples/example_usb_camera.py`

## USB Camera Permission Denied

```
[USBCamera] Device /dev/video0: reconnection failed
```

On Linux, video devices require membership in the `video` group:

```bash
sudo usermod -aG video $USER
# Log out and back in for the change to take effect
```

## Protocol Not Implemented

```
ValueError: Unknown ROS protocol: ...
```

Valid values:

- `ros_protocol`: `"rosbridge"`, `"zenoh"`, `"auto"`
- `camera_protocol`: `"webrtc"`, `"zenoh"`, `"shm"`, `"usb"`, `"none"`

## Arm Actions Fail

```
[Arm] Action go_to_pose failed: ...
```

1. Check MoveIt is running on the robot
2. Verify the `group_name` matches your robot config (e.g., `"left_arm"`, `"right_arm"`)
3. Check joint state subscription is active: `bot.arm.get_joint_states()` should return data
4. For custom IK mode, verify the IK solver node is subscribed to the target pose topic

## Mixed Camera: One Camera Fails

When using the `cameras` dict, each camera connects independently.
If one camera fails, the others still work:

```
Warning: Camera 'wrist' connection failed: ...
Camera 'wrist' will not be available.
```

The failed camera returns `None` from `get_frame()`. Check which cameras
are active:

```python
for name in bot.cameras.camera_names:
    frame = bot.cameras.get_frame(name)
    print(f"{name}: {'OK' if frame is not None else 'No frame'}")
```
