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

        self.width = 640  # Assuming image width
        self.height = 480  # Assuming image height
        self.fps = 30  # Assuming frame rate

        # Start FFmpeg subprocess for H.264 encoding
        self.ffmpeg = self.start_ffmpeg()

        self.get_logger().info("Camera encoder node initialized")
       

    def start_ffmpeg(self):
        # FFmpeg command to encode raw video to H.264 in Annex B format
        cmd = [
            'ffmpeg',
            '-loglevel', 'quiet',  # Disable FFmpeg logs
            '-f', 'rawvideo',
            '-pixel_format', 'bgr24',  # Input format: 24-bit BGR
            '-video_size', f'{self.width}x{self.height}',  # Image resolution
            '-framerate', str(self.fps),  # Frame rate
            '-i', '-',  # Input comes from stdin
            '-c:v', 'libx264',  # Use H.264 codec
            '-preset', 'ultrafast',  # Fastest encoding (low latency)
            '-tune', 'zerolatency',  # Low-latency encoding
            '-x264-params', 'repeat-headers=1:keyint=30',  # Ensure SPS/PPS before each keyframe
            '-f', 'h264',  # Output format: H.264 in Annex B
            '-'  # Output to stdout (FFmpeg will send encoded frames here)
        ]

        return subprocess.Popen(cmd, stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.PIPE)

    def image_callback(self, msg: Image):
        try:
            # Ensure the image is in bgr8 encoding and the raw bytes are accessible
            if msg.encoding != 'bgr8':
                self.get_logger().error(f"Unsupported encoding: {msg.encoding}")
                return

            # Directly get the raw byte data from the Image message
            raw_bytes = bytes(msg.data)

            # Write raw bytes to FFmpeg's stdin for encoding
            self.ffmpeg.stdin.write(raw_bytes)

            # Read the encoded frame from FFmpeg's stdout
            encoded_frame = self.ffmpeg.stdout.read(1024 * 64)  # Read up to 64KB of compressed data

            if encoded_frame:
                # Create and publish a CompressedVideo message
                video_msg = CompressedVideo()
                video_msg.timestamp = msg.header.stamp
                video_msg.frame_id = msg.header.frame_id
                video_msg.data = list(encoded_frame)  # Convert the bytes to list of integers
                video_msg.format = 'h264'

                # Publish the compressed video message
                self.publisher.publish(video_msg)

        except Exception as e:
            self.get_logger().error(f"Encoding error: {e}")

    def cleanup(self):
        if self.ffmpeg:
            try:
                if self.ffmpeg.stdin:
                    self.ffmpeg.stdin.close()
                self.ffmpeg.terminate()
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
