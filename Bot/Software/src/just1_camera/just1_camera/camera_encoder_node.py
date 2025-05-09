import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge
import cv2

class CameraEncoderNode(Node):
    def __init__(self):
        super().__init__('camera_encoder_node')
        self.bridge = CvBridge()
        self.jpeg_quality = 85  # JPEG quality (0-100)
        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10
        )
        self.publisher = self.create_publisher(
            CompressedImage,
            '/camera/image_compressed',
            10
        )
        self.get_logger().info("CameraEncoderNode initialized and ready.")

    def image_callback(self, msg):
        try:
            #cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            success, encoded_image = cv2.imencode('.jpg', msg.data, [int(cv2.IMWRITE_JPEG_QUALITY), self.jpeg_quality])
            if success:
                compressed_msg = CompressedImage()
                compressed_msg.header = msg.header
                compressed_msg.format = 'jpeg'
                compressed_msg.data = encoded_image.tobytes()
                self.publisher.publish(compressed_msg)
        except Exception as e:
            self.get_logger().error(f"Failed to convert image: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = CameraEncoderNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()
