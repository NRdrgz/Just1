import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from foxglove_msgs.msg import CompressedVideo

import av
import numpy as np

class CameraEncoderNode(Node):
    def __init__(self):
        super().__init__("camera_encoder_node")

        self.subscription = self.create_subscription(
            Image,
            "camera/image_raw",
            self.image_callback,
            10
        )
        self.publisher = self.create_publisher(
            CompressedVideo,
            "camera/video_compressed",
            10
        )

        self.width = 640
        self.height = 480
        self.fps = 30
        
        # Initialize codec
        self.codec = av.codec.CodecContext.create("libx264", "w")
        self.codec.options = {
            "preset": "ultrafast",
            "tune": "zerolatency",
            "repeat-headers": "1",
        }
        self.codec.width = self.width
        self.codec.height = self.height
        self.codec.time_base = av.Rational(1, self.fps)
        self.codec.pix_fmt = "yuv420p"
        self.codec.open()

    def image_callback(self, msg: Image):
        try:
            # Convert raw image data to numpy array
            img = np.frombuffer(msg.data, dtype=np.uint8).reshape((self.height, self.width, 3))

            # Convert to PyAV VideoFrame
            frame = av.VideoFrame.from_ndarray(img, format="bgr24")
            frame.pts = None

            # Encode
            packets = self.codec.encode(frame)
            for packet in packets:
                out_msg = CompressedVideo()
                out_msg.timestamp = msg.header.stamp
                out_msg.frame_id = msg.header.frame_id
                out_msg.format = "h264"
                out_msg.data = packet.to_bytes()
                self.publisher.publish(out_msg)

        except Exception as e:
            self.get_logger().error(f"Encoding failed: {e}")


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
