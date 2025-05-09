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
            
            # Now read the output from ffmpeg (raw H.264)
            nal_unit_data = b""
            while True:
                byte = self.ffmpeg_process.stdout.read(1)
                if not byte:
                    break
                nal_unit_data += byte
                

                # Look for NAL unit start code (0x000001)
                if len(nal_unit_data) >= 4:
                    # Found the NAL unit start code (0x000001), slice the frame
                    frame_start = nal_unit_data.find(b'\x00\x00\x01')  # Find the start code
                    if frame_start != -1:
                        self.get_logger().info('NAL unit start code found')
                        # Frame found; extract it
                        frame = nal_unit_data[frame_start:]
                        nal_unit_data = b""

                        break

            frame_data = frame
            self.get_logger().info(f'frame_data: {frame_data}')

            if frame_data:
                out_msg = CompressedVideo()
                out_msg.timestamp = msg.header.stamp
                out_msg.frame_id = msg.header.frame_id
                out_msg.format = "h264"
                out_msg.data = frame_data
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
