import zenoh
import time
import json
import sys
from walkie_sdk.robot import WalkieRobot
from walkie_sdk.utils.converters import euler_to_quaternion

# Global robot instance
robot = None
# Marker ID for the end-effector visualization (one per group)
_marker_ids = {}


def listener(sample):
    """
    Callback triggered when a Zenoh message arrives on 'arm_pose'.
    """
    global robot

    try:
        # 1. Decode and Parse
        payload = sample.payload.to_string()
        data = json.loads(payload)

        print(f"\n[Zenoh] Received: {payload}")

        # 2. Extract Parameters with defaults
        group = data.get("group_name", "left_arm")
        x = float(data.get("x", 0.0))
        y = float(data.get("y", 0.0))
        z = float(data.get("z", 0.0))
        roll = float(data.get("roll", 0.0))
        pitch = float(data.get("pitch", 0.0))
        yaw = float(data.get("yaw", 0.0))
        cartesian = bool(data.get("cartesian_path", False))
        blocking = bool(data.get("blocking", True))
        is_relative = bool(data.get("relative", False))

        # 3. Validation: Ensure robot is ready
        if robot is None or not robot.is_connected:
            print("[Error] Robot is not connected, ignoring command.")
            return

        # 4. Execute Command
        # We use blocking=False to prevent freezing the Zenoh listener thread
        if is_relative:
            print(f" -> Executing RELATIVE Move: {group}")
            robot.arm.go_to_pose_relative(
                x=x,
                y=y,
                z=z,
                roll=roll,
                pitch=pitch,
                yaw=yaw,
                group_name=group,
                cartesian_path=cartesian,
                blocking=blocking,
            )
        else:
            print(f" -> Executing ABSOLUTE Move: {group}")
            robot.arm.go_to_pose(
                x=x,
                y=y,
                z=z,
                roll=roll,
                pitch=pitch,
                yaw=yaw,
                group_name=group,
                cartesian_path=cartesian,
                blocking=blocking,
            )

        # 5. Visualize end-effector target as an arrow marker in RViz2
        qx, qy, qz, qw = euler_to_quaternion(roll, pitch, yaw)
        if group not in _marker_ids:
            # First time: create the marker
            _marker_ids[group] = robot.draw_marker(
                position=[x, y, z],
                quaternion=[qx, qy, qz, qw],
                ns=group,
            )
            print(f" -> Created viz marker for '{group}' (id={_marker_ids[group]})")
        else:
            # Update existing marker position/orientation
            robot.update_marker(
                _marker_ids[group],
                position=[x, y, z],
                quaternion=[qx, qy, qz, qw],
            )
            print(f" -> Updated viz marker for '{group}'")

    except json.JSONDecodeError:
        print(f"[Error] Invalid JSON format: {sample.payload.to_string()}")
    except Exception as e:
        print(f"[Error] Failed to process command: {e}")


def main():
    global robot

    # 1. Initialize Robot Connection
    # Change host to your robot's IP
    print("Connecting to WalkieRobot...")
    robot = WalkieRobot(ip="127.0.0.1", ros_port=9090)

    try:
        print("✓ Robot Connected and Arm Initialized")

        # 2. Initialize Zenoh
        print("Opening Zenoh session...")
        conf = zenoh.Config()
        session = zenoh.open(conf)

        # 3. Start Subscriber
        key_expr = "arm_pose"
        print(f"Watching for updates on Zenoh key: '{key_expr}'...")
        sub = session.declare_subscriber(key_expr, listener)

        # 4. Keep Alive
        while True:
            time.sleep(1)

    except KeyboardInterrupt:
        print("\nStopping...")
    except Exception as e:
        print(f"\nCritical Error: {e}")
    finally:
        if "session" in locals() and session:
            session.close()
        if robot:
            robot.disconnect()


if __name__ == "__main__":
    main()
