import rclpy
from rclpy.node import Node
from foxglove_msgs.msg import CompressedVideo
from sensor_msgs.msg import Image
import subprocess

class CameraEncoderNode(Node):
    def __init__(self):
        super().__init__("camera_encoder_node")

        # Create publisher for the compressed video
        self.publisher = self.create_publisher(CompressedVideo, "camera/video_compressed", 10)

        # Create subscription to raw camera data
        self.subscription = self.create_subscription(
            Image,
            "camera/image_raw",
            self.image_callback,
            10
        )

        # Initialize encoder
        self.encoder = subprocess.Popen(
            [
                "ffmpeg",
                "-f", "rawvideo",
                "-pix_fmt", "bgr24",
                "-s", "640x480",
                "-r", "30",
                "-i", "-",
                "-c:v", "libx264",
                "-preset", "ultrafast",
                "-tune", "zerolatency",
                "-f", "h264",
                "-"
            ],
            stdin=subprocess.PIPE,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,  # Capture stderr for debugging
            bufsize=0
        )

        self.get_logger().info("Camera encoder node initialized")
        self.frame_count = 0

    def image_callback(self, msg):
        try:
            # Verify image format
            if msg.encoding != 'bgr8':
                self.get_logger().warn(f"Unexpected image encoding: {msg.encoding}, expected bgr8")
                return

            # Verify image dimensions
            if msg.width != 640 or msg.height != 480:
                self.get_logger().warn(f"Unexpected image dimensions: {msg.width}x{msg.height}, expected 640x480")
                return

            # Write raw bytes directly to encoder
            self.encoder.stdin.write(msg.data)
            
            # Read available frame from encoder
            data = self._read_frame()
            if data:
                compressed_msg = CompressedVideo()
                # Add metadata to the message
                compressed_msg.timestamp = self.get_clock().now().to_msg()
                compressed_msg.frame_id = "camera"
                compressed_msg.format = "h264"
                compressed_msg.data = data
                self.publisher.publish(compressed_msg)
                
                self.frame_count += 1
                if self.frame_count % 30 == 0:  # Log every 30 frames
                    self.get_logger().info(f"Published {self.frame_count} frames")
            else:
                self.get_logger().warn("No data received from encoder")
                
        except Exception as e:
            self.get_logger().error(f"Encoding error: {e}")
            # Read and log any FFmpeg errors
            stderr_data = self.encoder.stderr.read1(1024)
            if stderr_data:
                self.get_logger().error(f"FFmpeg error: {stderr_data.decode()}")

    def _read_frame(self):
        # Read bytes until we get a full H.264 NAL unit
        try:
            data = self.encoder.stdout.read1(1024 * 1024)
            if not data:
                self.get_logger().warn("No data read from encoder stdout")
            return data
        except Exception as e:
            self.get_logger().error(f"Error reading from encoder: {e}")
            return None

    def cleanup(self):
        """Cleanup resources"""
        if self.encoder:
            self.encoder.stdin.close()
            self.encoder.stdout.close()
            self.encoder.stderr.close()
            self.encoder.terminate()


def main(args=None):
    rclpy.init(args=args)
    node = CameraEncoderNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.cleanup()
        node.destroy_node()


if __name__ == "__main__":
    main()
