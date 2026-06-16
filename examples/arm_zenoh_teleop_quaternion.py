import argparse
import zenoh
import time
import json
import threading
import math
from walkie_sdk import WalkieRobot
from walkie_sdk.utils.converters import (
    quaternion_to_euler,
    euler_to_quaternion,
    quaternion_multiply,
    rotate_vector_by_quaternion,
    compose_transforms,
    invert_transform,
    resolve_tf_chain,
)

# Global robot instance
robot = None
# Pose topics for the end-effector visualization (one per group)
_pose_topics = {}
# Initial EE poses keyed by group name: {group: (x, y, z, qx, qy, qz, qw)}
_initial_ee_pose = {}

# EE link names per group
EE_LINKS = {
    "left_arm": "left_link7",
    "right_arm": "right_link7",
}


# ---------------------------------------------------------------------------
# Quaternion / TF helpers (rotate_vector_by_quaternion, compose_transforms,
# invert_transform, resolve_tf_chain) are imported from walkie_sdk.utils.converters
# ---------------------------------------------------------------------------


def lookup_ee_pose(
    robot_instance, target_link, reference_frame="base_footprint", timeout=5.0
):
    """
    Subscribe to /tf, collect transforms, and walk the TF tree from
    *reference_frame* to *target_link* to compute the end-effector pose.

    Returns (x, y, z, qx, qy, qz, qw) in *reference_frame*, or None on
    failure / timeout.
    """
    # Shared state protected by a lock
    tf_data = {}  # (parent, child) -> (tx, ty, tz, qx, qy, qz, qw)
    result = [None]  # mutable container for the result
    done_event = threading.Event()

    def _on_tf(msg):
        transforms = msg.get("transforms", [])
        for t in transforms:
            parent = t["header"]["frame_id"]
            child = t["child_frame_id"]
            tr = t["transform"]["translation"]
            ro = t["transform"]["rotation"]
            tf_data[(parent, child)] = (
                tr["x"],
                tr["y"],
                tr["z"],
                ro["x"],
                ro["y"],
                ro["z"],
                ro["w"],
            )
        # Try to resolve the chain after every batch
        chain = resolve_tf_chain(tf_data, reference_frame, target_link)
        if chain is not None:
            print(chain)
            result[0] = chain
            done_event.set()

    handle = robot_instance._transport.subscribe(
        topic="/tf",
        message_type="tf2_msgs/msg/TFMessage",
        callback=_on_tf,
        throttle_rate=0,
        queue_size=10,
    )

    done_event.wait(timeout=timeout)
    robot_instance._transport.unsubscribe(handle)

    return result[0]


# ---------------------------------------------------------------------------
# Axis remapping:  controller frame  -->  ROS base_footprint frame
# ---------------------------------------------------------------------------
#
# Position mapping (confirmed):
#   controller x  ->  ROS x
#   controller z  ->  ROS -y
#   controller y  ->  ROS z
#
# Rotation mapping (Option C, confirmed):
#   controller roll   ->  ROS yaw
#   controller pitch  ->  ROS roll
#   controller yaw    ->  ROS -pitch
#
# So: ros_roll = c_pitch, ros_pitch = -c_yaw, ros_yaw = c_roll


def remap_controller_to_ros(cx, cy, cz, cqx, cqy, cqz, cqw):
    """
    Remap controller-frame position + quaternion to ROS base_footprint frame.

    Args:
        cx, cy, cz: controller position
        cqx, cqy, cqz, cqw: controller orientation quaternion

    Returns:
        (ros_x, ros_y, ros_z, ros_qx, ros_qy, ros_qz, ros_qw)
    """
    # Position remap
    ros_x = -cx
    ros_y = -cz
    ros_z = cy

    # Swap roll and pitch quaternion components
    ros_qx = -cqx
    ros_qy = -cqz
    ros_qz = cqy
    ros_qw = cqw
    # Apply -90 degree yaw offset
    yaw_offset = math.radians(-90)
    offset_quat = euler_to_quaternion(0, 0, yaw_offset)
    ros_quat = (ros_qx, ros_qy, ros_qz, ros_qw)
    ros_quat = quaternion_multiply(ros_quat, offset_quat)
    ros_qx, ros_qy, ros_qz, ros_qw = ros_quat
    return (ros_x, ros_y, ros_z, ros_qx, ros_qy, ros_qz, ros_qw)


# ---------------------------------------------------------------------------
# Zenoh callback
# ---------------------------------------------------------------------------


def listener(sample):
    """
    Callback triggered when a Zenoh message arrives on 'arm_pose'.

    The controller sends cumulative absolute positions from its own origin.
    We remap them into the ROS frame and add the robot's initial EE pose
    so the arm moves relative to where it started.
    """
    global robot

    try:
        # 1. Decode and Parse
        payload = sample.payload.to_string()
        data = json.loads(payload)

        print(f"\n[Zenoh] Received: {payload}")

        # 2. Extract Parameters
        group = data.get("group_name", "left_arm")
        cx = float(data.get("x", 0.0))
        cy = float(data.get("y", 0.0))
        cz = float(data.get("z", 0.0))

        cqx = float(data.get("qx", 0.0))
        cqy = float(data.get("qy", 0.0))
        cqz = float(data.get("qz", 0.0))
        cqw = float(data.get("qw", 1.0))

        gripperDist = float(data.get("gripperDist", None))

        link_name = data.get("link_name", EE_LINKS.get(group, "left_link7"))
        planning_time = float(data.get("allowed_planning_time", 10.0))
        blocking = bool(data.get("blocking", False))

        global teleop_status
        teleop_status = data.get("teleop_status", "IN_PROGRESS")

        # 3. Validation
        if robot is None or not robot.is_connected:
            print("[Error] Robot is not connected, ignoring command.")
            return

        if group not in _initial_ee_pose:
            print(f"[Error] No initial EE pose for group '{group}', ignoring.")
            return

        # 4. Remap controller values into ROS frame
        ros_x, ros_y, ros_z, ros_qx, ros_qy, ros_qz, ros_qw = remap_controller_to_ros(
            cx, cy, cz, cqx, cqy, cqz, cqw
        )

        # 5. Add initial EE pose offset
        init = _initial_ee_pose[group]
        init_pos = init[:3]
        init_quat = init[3:]

        # target position = initial_pos + remapped delta position
        target_x = init_pos[0] + ros_x
        target_y = init_pos[1] + ros_y
        target_z = init_pos[2] + ros_z

        # target orientation = initial_quat * remapped delta rotation
        target_quat = quaternion_multiply(init_quat, (ros_qx, ros_qy, ros_qz, ros_qw))
        target_qx, target_qy, target_qz, target_qw = target_quat

        print(
            f" -> Remapped target: pos=({target_x:.4f}, {target_y:.4f}, {target_z:.4f}) "
            f"quat=({target_qx:.4f}, {target_qy:.4f}, {target_qz:.4f}, {target_qw:.4f})"
        )

        if teleop_status == "COMPLETED":
            print("Teleop session marked as completed.")
            return

        # 6. Execute quaternion move
        status = robot.arm.go_to_pose_quaternion(
            x=target_x,
            y=target_y,
            z=target_z,
            qx=target_qx,
            qy=target_qy,
            qz=target_qz,
            qw=target_qw,
            group_name=group,
            # frame_id="base_footprint",
            # link_name=link_name,
            # allowed_planning_time=planning_time,
            # mode="moveit",
            mode="custom_ik",
            # blocking=blocking,
        )

        # 6.5 Execue gripper move
        if gripperDist is not None:
            gripper_status = robot.arm.control_gripper(group, gripperDist)
            print(f" -> Gripper result: {gripper_status}")

        print(f" -> Result: {status}")

        # 7. Visualize end-effector target as a PoseStamped in RViz2
        pose_topic = f"walkie/target_pose/{group}"
        if group not in _pose_topics:
            _pose_topics[group] = robot.draw_pose(
                position=[target_x, target_y, target_z],
                quaternion=[target_qx, target_qy, target_qz, target_qw],
                topic=pose_topic,
            )
            print(f" -> Created pose on topic '{_pose_topics[group]}'")
        else:
            robot.update_pose(
                position=[target_x, target_y, target_z],
                quaternion=[target_qx, target_qy, target_qz, target_qw],
                topic=pose_topic,
            )
            print(f" -> Updated pose on topic '{pose_topic}'")

    except json.JSONDecodeError:
        print(f"[Error] Invalid JSON format: {sample.payload.to_string()}")
    except Exception as e:
        print(f"[Error] Failed to process command: {e}")


def main():
    global robot
    global teleop_status
    teleop_status = "IN_PROGRESS"

    parser = argparse.ArgumentParser(description="Zenoh arm teleoperation (quaternion)")
    parser.add_argument("--ip", default="127.0.0.1", help="Robot IP address")
    parser.add_argument("--ros-port", type=int, default=9090, help="ROS bridge port")
    args = parser.parse_args()

    # 1. Initialize Robot Connection
    print("Connecting to WalkieRobot...")
    robot = WalkieRobot(
        ip=args.ip, ros_port=args.ros_port, camera_protocol="none", arm_mode="custom_ik"
    )
    # robot = WalkieRobot(ip="10.206.61.14", ros_port=9090, camera_protocol='none',arm_mode='custom_ik')

    try:
        print("Robot Connected and Arm Initialized")

        # 2. Look up initial EE poses via /tf
        for group, link in EE_LINKS.items():
            print(f"Looking up initial EE pose for '{group}' ({link})...")
            pose = lookup_ee_pose(robot, target_link=link, timeout=0.3)
            if pose is None:
                print(f"[Warning] Could not resolve TF for '{link}' within timeout.")
                print(f"          Using identity pose (0,0,0, 0,0,0,1) for '{group}'.")
                pose = (0.2, -0.3, 0.5, 0.0, 0.0, 0.0, 1.0)
            else:
                print(
                    f"  Initial pose for '{group}': "
                    f"pos=({pose[0]:.4f}, {pose[1]:.4f}, {pose[2]:.4f}) "
                    f"quat=({pose[3]:.4f}, {pose[4]:.4f}, {pose[5]:.4f}, {pose[6]:.4f})"
                )
            _initial_ee_pose[group] = pose

        # 3. Initialize Zenoh
        print("Opening Zenoh session...")
        conf = zenoh.Config()
        session = zenoh.open(conf)

        # 4. Start Subscriber
        key_expr = "arm_pose"
        print(f"Watching for updates on Zenoh key: '{key_expr}'...")
        sub = session.declare_subscriber(key_expr, listener)
        # 5. Keep Alive
        while teleop_status != "COMPLETED":
            print(f"Teleop session status: {teleop_status}.")
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
