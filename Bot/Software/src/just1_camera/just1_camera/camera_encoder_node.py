import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from foxglove_msgs.msg import CompressedVideo

import ffmpeg
import subprocess


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
        self.read_buffer = bytearray()

        # Start persistent ffmpeg process
        self.ffmpeg_process = self.start_ffmpeg()
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
            if msg.encoding != 'bgr8':
                self.get_logger().error(f"Unsupported image encoding: {msg.encoding}")
                return

            expected_size = self.width * self.height * 3
            if len(msg.data) != expected_size:
                self.get_logger().error(f"Unexpected image size: {len(msg.data)} != {expected_size}")
                return
            
            if self.ffmpeg_process.poll() is not None:
                err = self.ffmpeg_process.stderr.read().decode()
                self.get_logger().error(f"FFmpeg crashed:\n{err}")
                return

            # Feed frame to FFmpeg
            self.ffmpeg_process.stdin.write(msg.data)

            # Read and publish H264 frame
            # This is required by Foxglove: https://docs.foxglove.dev/docs/visualization/message-schemas/compressed-video
            frame = self.ffmpeg_process.stdout.read(1024)
            if frame:
                out_msg = CompressedVideo()
                out_msg.timestamp = msg.header.stamp
                out_msg.frame_id = msg.header.frame_id
                out_msg.format = "h264"
                out_msg.data = frame
                self.publisher.publish(out_msg)

        except Exception as e:
            self.get_logger().error(f"Encoding failed: {e}")


    def cleanup(self):
        if self.ffmpeg_process:
            try:
                self.ffmpeg_process.stdin.close()
                self.ffmpeg_process.stdout.close()
                self.ffmpeg_process.terminate()
            except Exception as e:
                self.get_logger().warn(f"Error shutting down FFmpeg: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = CameraEncoderNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.cleanup()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
