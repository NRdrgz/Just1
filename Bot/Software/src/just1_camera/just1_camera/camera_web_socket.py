import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import asyncio
import websockets
import threading


class CameraWebSocketBridge(Node):
    """
    Homemade WebSocket bridge for the camera node.
    This is not used anymore as we are using Foxglove Bridge node instead.
    """

    def __init__(self):
        super().__init__("camera_web_socket")
        self.bridge = CvBridge()
        self.frame = None
        self.jpeg_quality = 85  # JPEG quality (0-100)

        # Subscribe to camera topic
        self.subscription = self.create_subscription(
            Image, "camera/image_raw", self.listener_callback, 10
        )

        # Start the WebSocket server in a separate thread
        self.server_thread = threading.Thread(target=self.start_server)
        self.server_thread.start()
        self.get_logger().info("WebSocket server started")

    def listener_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            # Convert to RGB for web display
            rgb_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
            # Encode to JPEG with specified quality
            _, jpeg = cv2.imencode(
                ".jpg", rgb_image, [cv2.IMWRITE_JPEG_QUALITY, self.jpeg_quality]
            )
            # Store as binary data
            self.frame = jpeg.tobytes()
        except Exception as e:
            self.get_logger().error(f"Failed to convert image: {e}")

    async def send_frames(self, websocket):
        while True:
            if self.frame:
                await websocket.send(self.frame)
            await asyncio.sleep(0.033)  # Match camera FPS

    def start_server(self):
        async def run():
            # Start the WebSocket server with binary mode enabled
            async with websockets.serve(
                self.send_frames, "0.0.0.0", 8765, compression=None
            ):
                self.get_logger().info("WebSocket server running at ws://0.0.0.0:8765")
                await asyncio.Future()  # Keep it running forever

        asyncio.run(run())


def main(args=None):
    rclpy.init(args=args)
    node = CameraWebSocketBridge()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()


if __name__ == "__main__":
    main()
