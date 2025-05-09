import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from picamera2 import Picamera2
import cv2


class CameraNode(Node):
    def __init__(self):
        super().__init__("camera_node")

        self.width = 640
        self.height = 480
        self.fps = 30

        # Create publisher for the camera feed
        self.publisher = self.create_publisher(Image, "camera/image_raw", 10)

        # Initialize CV bridge
        self.bridge = CvBridge()

        # Initialize Picamera2
        self.picam2 = Picamera2()

        # Configure camera for optimized performance
        config = self.picam2.create_preview_configuration(
            main={"size": (self.width, self.height), "format": "BGR888"}
        )
        self.picam2.configure(config)

        # Start camera
        self.picam2.start()

        # Optimize camera controls
        self.picam2.set_controls(
            {
                "AeEnable": True,
                "FrameDurationLimits": (
                    int((1 / self.fps) * 1000000),
                    int((1 / self.fps) * 1000000),
                ),  
                "NoiseReductionMode": 0,  # Disable noise reduction for lower latency
            }
        )

        # Create timer for publishing frames
        self.timer = self.create_timer(
            1 / self.fps, self.timer_callback
        )  
        self.get_logger().info("Camera node initialized")

    def timer_callback(self):
        try:
            # Capture frame directly in BGR format
            frame = self.picam2.capture_array()

            # Flip the frame vertically and horizontally
            frame = cv2.flip(frame, 0)
            frame = cv2.flip(frame, 1)

            # Convert frame to ROS2 message
            msg = self.bridge.cv2_to_imgmsg(frame, "bgr8")

            self.publisher.publish(msg)
        except Exception as e:
            self.get_logger().error(f"Error capturing or converting frame: {e}")

    def cleanup(self):
        """Cleanup resources"""
        if hasattr(self, "picam2"):
            self.picam2.stop()
            self.picam2.close()


def main(args=None):
    rclpy.init(args=args)
    node = CameraNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.cleanup()
        node.destroy_node()


if __name__ == "__main__":
    main()
