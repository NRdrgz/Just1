import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from foxglove_msgs.msg import CompressedVideo
import ffmpeg

H264_START_CODE = b'\x00\x00\x00\x01'  # Start code for Annex B format

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

        self.width = 640  # Assuming image width
        self.height = 480  # Assuming image height
        self.fps = 30  # Assuming frame rate

        # Initialize FFmpeg process
        self.process = None
        self.setup_ffmpeg()

        self.get_logger().info("Camera encoder node initialized")

    def setup_ffmpeg(self):
        try:
            # Configure FFmpeg process
            self.process = (
                ffmpeg
                .input('pipe:', format='rawvideo', pix_fmt='bgr24', s=f'{self.width}x{self.height}', r=self.fps)
                .output('pipe:', format='h264', vcodec='libx264', preset='ultrafast', tune='zerolatency',
                       x264opts='repeat-headers=1:keyint=30')
                .overwrite_output()
                .run_async(pipe_stdin=True, pipe_stdout=True, pipe_stderr=True)
            )
        except ffmpeg.Error as e:
            self.get_logger().error(f"Failed to initialize FFmpeg: {e.stderr.decode() if e.stderr else str(e)}")
            raise

    def image_callback(self, msg: Image):
        try:
            # Ensure the image is in bgr8 encoding
            if msg.encoding != 'bgr8':
                self.get_logger().error(f"Unsupported encoding: {msg.encoding}")
                return

            # Directly get the raw byte data from the Image message
            raw_bytes = bytes(msg.data)

            # Write raw bytes to FFmpeg's stdin for encoding
            self.process.stdin.write(raw_bytes)

            # Read encoded frame
            encoded_frame = self.read_full_frame()

            if encoded_frame:
                # Create and publish a CompressedVideo message
                video_msg = CompressedVideo()
                video_msg.timestamp = msg.header.stamp
                video_msg.frame_id = msg.header.frame_id
                video_msg.data = list(encoded_frame)
                video_msg.format = 'h264'

                # Publish the compressed video message
                self.publisher.publish(video_msg)

        except Exception as e:
            self.get_logger().error(f"Encoding error: {e}")

    def read_full_frame(self):
        """Reads a full compressed frame from FFmpeg's stdout."""
        try:
            # Read 64KB of data from stdout
            encoded_frame = self.process.stdout.read(1024 * 64)

            # Ensure that we have a valid frame (contains start code)
            if encoded_frame and encoded_frame.startswith(H264_START_CODE):
                return encoded_frame
            return b""
        except Exception as e:
            self.get_logger().error(f"Error reading frame: {e}")
            return b""

    def cleanup(self):
        if self.process:
            try:
                self.process.stdin.close()
                self.process.stdout.close()
                self.process.stderr.close()
                self.process.wait(timeout=2)
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
