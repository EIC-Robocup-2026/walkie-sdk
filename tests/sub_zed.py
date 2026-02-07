import time
import cv2
import numpy as np
from zenoh_ros2_sdk import ZenohSession, ROS2Subscriber

# Configuration
ROUTER_IP = "127.0.0.1"  # Change to your robot's IP if remote
ROUTER_PORT = 7447
DOMAIN_ID = 23
TOPIC = "/zed/zed_node/rgb/color/rect/image/compressed"
MSG_TYPE = "sensor_msgs/msg/CompressedImage"

def on_image_received(msg):
    """Callback function for CompressedImage messages"""
    try:
        # ROS 2 CompressedImage 'data' field contains the raw bytes
        # msg is a dataclass from the SDK, convert data to bytes if needed
        image_data = msg.data
        if hasattr(image_data, 'tobytes'):
            image_bytes = image_data.tobytes()
        elif isinstance(image_data, list):
            image_bytes = bytes(image_data)
        else:
            image_bytes = bytes(image_data)

        # Decode JPEG/PNG buffer to OpenCV image
        np_arr = np.frombuffer(image_bytes, np.uint8)
        frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

        if frame is not None:
            cv2.imshow("ZED Camera Stream", frame)
            cv2.waitKey(1)
        else:
            print("Failed to decode frame")

    except Exception as e:
        print(f"Error processing frame: {e}")

def main():
    print(f"Connecting to Zenoh at {ROUTER_IP}:{ROUTER_PORT}...")
    
    # 1. Initialize Session
    # This singleton manages the connection and type registry
    ZenohSession.get_instance(router_ip=ROUTER_IP, router_port=ROUTER_PORT)
    
    print(f"Subscribing to: {TOPIC} (Domain: {DOMAIN_ID})")

    # 2. Create Subscriber
    # The SDK automatically handles discovery and key expression generation
    # Key generated: 23/zed/zed_node/rgb/color/rect/image/compressed/...
    sub = ROS2Subscriber(
        topic=TOPIC,
        msg_type=MSG_TYPE,
        domain_id=DOMAIN_ID,
        callback=on_image_received,
        router_ip=ROUTER_IP,
        router_port=ROUTER_PORT
    )

    try:
        print("Stream started. Press Ctrl+C to exit.")
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        print("\nStopping...")
    finally:
        sub.close()
        cv2.destroyAllWindows()
        ZenohSession.get_instance().close()

if __name__ == "__main__":
    main()