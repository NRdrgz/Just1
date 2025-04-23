import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from picamera2 import Picamera2
import numpy as np


class CameraNode(Node):
    def __init__(self):
        super().__init__("camera_node")

        # Create publisher for the camera feed
        self.publisher = self.create_publisher(Image, "camera/image_raw", 10)

        # Initialize CV bridge
        self.bridge = CvBridge()

        # Initialize PiCamera2
        self.picam2 = Picamera2()

        # Configure camera
        config = self.picam2.create_preview_configuration(
            main={"size": (640, 480), "format": "RGB888"}
        )
        self.picam2.configure(config)

        # Start camera
        self.picam2.start()

        # Create timer for publishing frames
        self.timer = self.create_timer(0.033, self.timer_callback)  # ~30 FPS

        self.get_logger().info("Camera node initialized")

    def timer_callback(self):
        try:
            # Capture frame from PiCamera
            frame = self.picam2.capture_array()

            # Convert frame to ROS2 message
            msg = self.bridge.cv2_to_imgmsg(frame, "rgb8")
            self.get_logger().info(f"Publishing frame with shape: {frame.shape}")

            self.publisher.publish(msg)
        except Exception as e:
            self.get_logger().error(f"Error capturing or converting frame: {e}")

    def __del__(self):
        if hasattr(self, "picam2"):
            self.picam2.stop()


def main(args=None):
    rclpy.init(args=args)
    node = CameraNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.destroy_node()
        rclpy.shutdown()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
