import time
import cv2
from walkie_sdk.robot import WalkieRobot

# CONFIGURATION
ROBOT_IP = "127.0.0.1"  # Replace with your actual Robot IP (e.g., 192.168.1.x)
ZENOH_PORT = 7447       # Default Zenoh port

def main():
    print(f"Initializing WalkieRobot with Zenoh on {ROBOT_IP}...")

    # 1. Initialize Robot
    #    - ros_protocol="zenoh": Uses ZenohTransport for commands (cmd_vel, etc.)
    #    - camera_protocol="zenoh": Uses your new ZenohCamera class for video
    try:
        bot = WalkieRobot(
            ip=ROBOT_IP,
            ros_protocol="rosbridge",
            camera_protocol="zenoh",
            ros_port=9090,    # For Zenoh, ros_port and camera_port
            camera_port=ZENOH_PORT  # usually point to the same router port
        )
    except Exception as e:
        print(f"Failed to connect: {e}")
        return

    print("✓ Robot Connected!")
    print("  - Transport: Zenoh")
    print("  - Camera: Zenoh (ZED Topic)")
    print("Sending robot to home position...")
    bot.arm.go_to_home(group_name="left_arm")
    time.sleep(5)
    print("Moving robot arm to initial pose...")
    bot.arm.go_to_pose(group_name="left_arm", x=0.38, y=0.19, z=0.58, roll=-1.57, pitch=0.0, yaw=1.57,cartesian_path=False,blocking=True)

    # 2. Main Viewer Loop
    try:
        while True:
            # 3. Get Frame
            #    WalkieRobot wraps the transport. This calls your ZenohCamera.get_frame()
            #    It returns a BGR numpy array or None.
            frame = bot.camera.get_frame()
            #print(f"Frame shape: {bot.camera.frame_shape}")
            # 4. Display
            if frame is not None:
                cv2.imshow("Stream", frame)
            else:
                # If None, the camera hasn't sent a frame yet, or isn't publishing.
                pass

            # Exit on 'q'
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
            
            # Small sleep to yield CPU
            time.sleep(0.01)

    except KeyboardInterrupt:
        print("\nStopping...")
    finally:
        # 5. Cleanup
        #    This calls bot.disconnect() -> camera.disconnect() -> session.close()
        bot.disconnect()
        cv2.destroyAllWindows()

if __name__ == "__main__":
    main()