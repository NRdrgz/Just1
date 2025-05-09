import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from foxglove_msgs.msg import CompressedVideo
import subprocess
import io

H264_START_CODE = b'\x00\x00\x00\x01'

class CameraEncoderNode(Node):
    def __init__(self):
        super().__init__("camera_encoder_node")

        self.publisher = self.create_publisher(CompressedVideo, "camera/video_compressed", 10)

        self.subscription = self.create_subscription(
            Image,
            "camera/image_raw",
            self.image_callback,
            10
        )

        # Start ffmpeg encoder
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
            stderr=subprocess.DEVNULL,  # Silence or redirect as needed
            bufsize=0
        )

        self.stdin = self.encoder.stdin
        self.stdout = io.BufferedReader(self.encoder.stdout)

        self.buffer = bytearray()
        self.get_logger().info("Camera encoder node initialized")
        self.frame_count = 0

    def image_callback(self, msg):
        try:
            # Only accept expected format
            if msg.encoding != 'bgr8':
                self.get_logger().warn(f"Unexpected image encoding: {msg.encoding}, expected bgr8")
                return

            if msg.width != 640 or msg.height != 480:
                self.get_logger().warn(f"Unexpected image size: {msg.width}x{msg.height}, expected 640x480")
                return

            # Write raw image bytes to encoder
            self.stdin.write(msg.data)
            self.stdin.flush()

            # Read and parse a full H.264 frame
            frame_data = self._read_h264_frame()
            if frame_data:
                compressed_msg = CompressedVideo()
                compressed_msg.timestamp = self.get_clock().now().to_msg()
                compressed_msg.frame_id = "camera"
                compressed_msg.format = "h264"
                compressed_msg.data = frame_data
                self.publisher.publish(compressed_msg)
                self.frame_count += 1

                if self.frame_count % 30 == 0:
                    self.get_logger().info(f"Published {self.frame_count} frames")
        except Exception as e:
            self.get_logger().error(f"Error in image_callback: {e}")

    def _read_h264_frame(self):
        try:
            while True:
                chunk = self.stdout.read1(4096)
                if not chunk:
                    break
                self.buffer.extend(chunk)

                # Search for start codes
                frames = []
                start = 0
                while True:
                    next_start = self.buffer.find(H264_START_CODE, start + 4)
                    if next_start == -1:
                        break
                    frames.append(self.buffer[start:next_start])
                    start = next_start
                # Leave remaining buffer from last start code
                if start > 0:
                    self.buffer = self.buffer[start:]

                # Return the first full frame
                if frames:
                    return b"".join(frames)
        except Exception as e:
            self.get_logger().error(f"Error reading from encoder: {e}")
            return None

    def cleanup(self):
        if self.encoder:
            try:
                if self.stdin:
                    self.stdin.close()
                self.encoder.terminate()
                self.encoder.wait(timeout=2)
            except Exception as e:
                self.get_logger().warn(f"Error during encoder shutdown: {e}")

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
