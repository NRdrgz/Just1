import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from foxglove_msgs.msg import CompressedVideo

import subprocess
from cv_bridge import CvBridge

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
        self.bridge = CvBridge()
        self.buffer = b''  # Persistent buffer for partial data

        # Start persistent ffmpeg process
        self.ffmpeg_process = self.start_ffmpeg()

        self.get_logger().info("Camera encoder node started")
        
    def start_ffmpeg(self):
        # FFmpeg command to encode raw video to H.264 in Annex B format
        cmd = [
            'ffmpeg',
            '-loglevel', 'quiet',  # Disable FFmpeg logs
            '-f', 'rawvideo',
            '-pixel_format', 'bgr24', 
            '-video_size', f'{self.width}x{self.height}',  # Image resolution
            '-framerate', str(self.fps),  # Frame rate
            '-i', '-',  # Input comes from stdin
            '-c:v', 'libx264',  # Use H.264 codec
            '-preset', 'ultrafast',  # Fastest encoding (low latency)
            '-tune', 'zerolatency',  # Low-latency encoding
            '-g', '1',           # Force each frame to be a keyframe (IDR)
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

            # Convert the incoming ROS image message to OpenCV format (BGR8)
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

            # Write the image data to ffmpeg's stdin
            self.ffmpeg_process.stdin.write(cv_image.tobytes())

            # Read stdout until we have at least two start codes
            found_frame = False
            max_reads = 10  # Prevent infinite loop in case of broken stream
            read_count = 0

            while not found_frame and read_count < max_reads:
                chunk = self.ffmpeg_process.stdout.read(4096)
                if not chunk:
                    break  # EOF or broken pipe

                self.buffer += chunk

                # Detect NAL start codes (00 00 00 01 or 00 00 01)
                start_codes = []
                i = 0
                while i < len(self.buffer) - 4:
                    if self.buffer[i:i+4] == b'\x00\x00\x00\x01':
                        start_codes.append(i)
                    elif self.buffer[i:i+3] == b'\x00\x00\x01':
                        start_codes.append(i)
                    i += 1

                if len(start_codes) >= 2:
                    frame_start = start_codes[0]
                    frame_end = start_codes[1]
                    frame_data = self.buffer[frame_start:frame_end]
                    self.buffer = self.buffer[frame_end:]  # Keep excess data

                    # Publish the compressed frame
                    out_msg = CompressedVideo()
                    out_msg.timestamp = msg.header.stamp
                    out_msg.frame_id = msg.header.frame_id
                    out_msg.format = "h264"
                    out_msg.data = frame_data
                    self.publisher.publish(out_msg)

                    self.get_logger().info(f"Published compressed frame of size {len(frame_data)} bytes")
                    found_frame = True

                read_count += 1

            if not found_frame:
                self.get_logger().warn("Failed to find two NAL unit start codes after multiple reads")

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
