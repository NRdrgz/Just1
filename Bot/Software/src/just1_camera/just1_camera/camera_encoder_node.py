import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from foxglove_msgs.msg import CompressedVideo
import ffmpeg
from cv_bridge import CvBridge


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

    def image_callback(self, msg: Image):
        # Convert ROS Image msg to OpenCV image (numpy array)
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        # Encode with ffmpeg-python
        height, width, _ = cv_image.shape
        process = (
            ffmpeg
            .input('pipe:0', format='rawvideo', pix_fmt='bgr24', s='{}x{}'.format(width, height))
            .output('pipe:1', format='h264', vcodec='libx264', pix_fmt='yuv420p',
                    preset='ultrafast', tune='zerolatency', x264opts='keyint=1:no-scenecut')
            .run_async(pipe_stdin=True, pipe_stdout=True, pipe_stderr=True, quiet=True)
        )

        # Write frame
        process.stdin.write(cv_image.tobytes())
        process.stdin.close()
        h264_data = process.stdout.read()
        process.stdout.close()
        process.wait()

        # Create and publish CompressedVideo message
        compressed_msg = CompressedVideo()
        compressed_msg.timestamp = msg.header.stamp
        compressed_msg.frame_id = msg.header.frame_id
        compressed_msg.data = list(h264_data)
        compressed_msg.format = 'h264'
        self.publisher.publish(compressed_msg)

        

def main(args=None):
    rclpy.init(args=args)
    node = CameraEncoderNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()

if __name__ == "__main__":
    main()
