import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2


class CameraNode(Node):
    def __init__(self):
        super().__init__("camera_node")

        # Create publisher for the camera feed
        self.publisher = self.create_publisher(Image, "camera/image_raw", 10)

        # Initialize CV bridge
        self.bridge = CvBridge()

        # Initialize camera
        self.cap = cv2.VideoCapture(0)  # Use 0 for the default camera
        if not self.cap.isOpened():
            self.get_logger().error("Could not open camera")
            raise RuntimeError("Could not open camera")

        # Set camera resolution (adjust as needed)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

        # Create timer for publishing frames
        self.timer = self.create_timer(0.033, self.timer_callback)  # ~30 FPS

        self.get_logger().info("Camera node initialized")

    def timer_callback(self):
        ret, frame = self.cap.read()

        if not ret:
            self.get_logger().error("Failed to capture frame")
            return
        self.get_logger().info(f"Raw frame shape: {frame.shape}, dtype: {frame.dtype}")

        # If the frame is grayscale (2D), convert to BGR
        if len(frame.shape) == 2:
            frame = cv2.cvtColor(frame, cv2.COLOR_GRAY2BGR)

        # Convert frame to ROS2 message
        try:
            msg = self.bridge.cv2_to_imgmsg(frame, "bgr8")
            self.get_logger().info(f"Publishing frame with shape: {frame.shape}")

            self.publisher.publish(msg)
        except Exception as e:
            self.get_logger().error(f"Error converting frame: {e}")

    def __del__(self):
        if hasattr(self, "cap"):
            self.cap.release()


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
