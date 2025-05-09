import rclpy
from rclpy.node import Node
from foxglove_msgs.msg import CompressedVideo
from picamera2 import Picamera2
import cv2
import subprocess

class CameraEncoderNode(Node):
    def __init__(self):
        super().__init__("camera_encoder_node")

        # Create publisher for the compressed video
        self.publisher = self.create_publisher(CompressedVideo, "camera/video_compressed", 10)

        # Initialize Picamera2
        self.picam2 = Picamera2()
        config = self.picam2.create_preview_configuration(main={"size": (640, 480), "format": "BGR888"})
        self.picam2.configure(config)
        self.picam2.start()

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
            stderr=subprocess.DEVNULL,
            bufsize=0
        )

        # Create timer for publishing frames at 30 Hz
        self.timer = self.create_timer(0.033, self.timer_callback)

        self.get_logger().info("Camera encoder node initialized")

    def timer_callback(self):
        frame = self.picam2.capture_array()
        frame = cv2.flip(frame, 0)
        frame = cv2.flip(frame, 1)

        try:
            self.encoder.stdin.write(frame.tobytes())
            # Read available frame from encoder
            data = self._read_frame()
            if data:
                msg = CompressedVideo()
                # Add metadata to the message
                msg.timestamp = self.get_clock().now().to_msg()
                msg.frame_id = "camera"
                msg.format = "h264"
                msg.data = data
                self.publisher.publish(msg)
        except Exception as e:
            self.get_logger().error(f"Encoding error: {e}")

    def _read_frame(self):
        # Read bytes until we get a full H.264 NAL unit
        try:
            return self.encoder.stdout.read1(1024 * 1024)
        except Exception:
            return None

    def cleanup(self):
        """Cleanup resources"""
        if hasattr(self, "picam2"):
            self.picam2.stop()
            self.picam2.close()
        if self.encoder:
            self.encoder.stdin.close()
            self.encoder.stdout.close()
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
