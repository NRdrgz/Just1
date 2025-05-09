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
        self.read_buffer = bytearray()

        # Start persistent ffmpeg process
        self.ffmpeg_process = self.start_ffmpeg()
        self.get_logger().info("Camera encoder node initialized")

    def start_ffmpeg(self):
        args = (
            ffmpeg
            .input('pipe:0', format='rawvideo', pix_fmt='bgr8', s=f'{self.width}x{self.height}', framerate=30)
            .output(
                'pipe:1',
                format='h264',
                vcodec='libx264',
                pix_fmt='yuv420p',
                preset='ultrafast',
                tune='zerolatency',
                **{'x264-params': 'keyint=1:no-scenecut'}
            )
            .compile()
        )

        return subprocess.Popen(args, stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.PIPE)

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

            # Read and publish exactly one encoded H264 frame
            # This is required by Foxglove: https://docs.foxglove.dev/docs/visualization/message-schemas/compressed-video
            frame = self.read_one_annexb_frame()
            if frame:
                out_msg = CompressedVideo()
                out_msg.timestamp = msg.header.stamp
                out_msg.frame_id = msg.header.frame_id
                out_msg.format = "h264"
                out_msg.data = frame
                self.publisher.publish(out_msg)

        except Exception as e:
            self.get_logger().error(f"Encoding failed: {e}")

    def read_one_annexb_frame(self):
        start_code = b'\x00\x00\x00\x01'
        self.read_buffer.extend(self.ffmpeg_process.stdout.read(4096))

        # Find NAL unit boundaries
        start_indices = []
        idx = self.read_buffer.find(start_code)
        while idx != -1:
            start_indices.append(idx)
            idx = self.read_buffer.find(start_code, idx + 4)

        # Need at least two NALs to find frame boundaries
        if len(start_indices) >= 2:
            frame_end = start_indices[1]
            frame = self.read_buffer[:frame_end]
            self.read_buffer = self.read_buffer[frame_end:]
            return bytes(frame)

        return None

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
