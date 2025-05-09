import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from foxglove_msgs.msg import CompressedVideo
import ffmpeg



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
        
        self.get_logger().info("Camera encoder node initialized")
 
    def image_callback(self, msg: Image):
        try:
            # Run FFmpeg to encode the frame to h264
            process = (
                ffmpeg
                .input('pipe:0', format='rawvideo', pix_fmt='bgr8', s=f'{self.width}x{self.height}')
                .output('pipe:1', format='h264', vcodec='libx264', pix_fmt='yuv420p',
                        preset='ultrafast', tune='zerolatency', x264opts='keyint=1:no-scenecut')
                .run_async(pipe_stdin=True, pipe_stdout=True, pipe_stderr=True)
            )

            # Write the raw frame to stdin and get the encoded bytes
            process.stdin.write(msg.data)
            process.stdin.close()
            encoded_bytes = process.stdout.read()
            process.stdout.close()
            process.wait()

            # Create and publish the CompressedVideo message
            compressed_msg = CompressedVideo()
            compressed_msg.timestamp = msg.header.stamp
            compressed_msg.frame_id = msg.header.frame_id
            compressed_msg.data = encoded_bytes
            compressed_msg.format = "h264"
            self.publisher.publish(compressed_msg)

        except Exception as e:
            self.get_logger().error(f"Encoding failed: {e}")
        

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
