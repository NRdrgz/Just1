import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from foxglove_msgs.msg import CompressedVideo
import ffmpeg
from cv_bridge import CvBridge
import queue
import threading


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
        self.bridge = CvBridge()

        self.frame_queue = queue.Queue()
        self.encoding_thread = threading.Thread(target=self.encoding_loop, daemon=True)
        self.encoding_thread.start()

        self.ffmpeg_process = None
        self.width = None
        self.height = None

    def start_ffmpeg_process(self, width, height):
        self.get_logger().info(f"Starting FFmpeg process for {width}x{height}")
        self.ffmpeg_process = (
            ffmpeg
            .input('pipe:0', format='rawvideo', pix_fmt='bgr24', s=f'{width}x{height}')
            .output('pipe:1', format='h264', vcodec='libx264', pix_fmt='yuv420p',
                    preset='ultrafast', tune='zerolatency',
                    x264opts='keyint=1:no-scenecut')
            .run_async(pipe_stdin=True, pipe_stdout=True, pipe_stderr=True, bufsize=10**8)
        )

    def image_callback(self, msg: Image):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f"CV bridge error: {e}")
            return

        # Dynamic set resolution based on the first frame
        if self.width is None or self.height is None:
            self.height, self.width, _ = cv_image.shape
            self.start_ffmpeg_process(self.width, self.height)

        self.frame_queue.put((msg, cv_image))

    def encoding_loop(self):
        while True:
            msg, frame = self.frame_queue.get()
            try:
                self.ffmpeg_process.stdin.write(frame.tobytes())

                # Read all available bytes from stdout
                # This assumes ffmpeg is configured to flush after every frame
                output = self.ffmpeg_process.stdout.read(65536)

                if output:
                    compressed_msg = CompressedVideo()
                    compressed_msg.timestamp = msg.header.stamp
                    compressed_msg.frame_id = msg.header.frame_id
                    compressed_msg.data = list(output)
                    compressed_msg.format = 'h264'
                    self.publisher.publish(compressed_msg)

            except Exception as e:
                self.get_logger().error(f"FFmpeg encoding error: {e}")

    def cleanup(self):
        if self.ffmpeg_process:
            self.ffmpeg_process.stdin.close()
            self.ffmpeg_process.stdout.close()
            self.ffmpeg_process.stderr.close()
            self.ffmpeg_process.wait()
    

        

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
