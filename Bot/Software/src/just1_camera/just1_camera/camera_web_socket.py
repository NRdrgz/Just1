import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import asyncio
import websockets
import threading
import signal
import sys


class CameraWebSocketBridge(Node):
    def __init__(self):
        super().__init__("camera_web_socket")
        self.bridge = CvBridge()
        self.frame = None
        self.jpeg_quality = 85  # JPEG quality (0-100)
        self.server_thread = None
        self.loop = None
        self.stop_event = threading.Event()

        # Subscribe to camera topic
        self.subscription = self.create_subscription(
            Image, "camera/image_raw", self.listener_callback, 10
        )

        # Start the WebSocket server in a separate thread
        self.server_thread = threading.Thread(target=self.start_server)
        self.server_thread.daemon = (
            True  # Make thread daemon so it exits when main thread exits
        )
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
            # Store as binary data instead of base64
            self.frame = jpeg.tobytes()
        except Exception as e:
            self.get_logger().error(f"Failed to convert image: {e}")

    async def send_frames(self, websocket):
        while not self.stop_event.is_set():
            if self.frame:
                try:
                    await websocket.send(self.frame)
                except websockets.exceptions.ConnectionClosed:
                    break
            await asyncio.sleep(0.033)  # Match camera FPS

    def start_server(self):
        async def run():
            # Start the WebSocket server with binary mode enabled
            async with websockets.serve(
                self.send_frames, "0.0.0.0", 8765, compression=None
            ):
                self.get_logger().info("WebSocket server running at ws://0.0.0.0:8765")
                self.loop = asyncio.get_event_loop()
                try:
                    while not self.stop_event.is_set():
                        await asyncio.sleep(0.1)
                except asyncio.CancelledError:
                    pass

        asyncio.run(run())

    def shutdown(self):
        """Clean shutdown of the node and WebSocket server"""
        self.stop_event.set()
        if self.loop and self.loop.is_running():
            self.loop.call_soon_threadsafe(self.loop.stop)
        if self.server_thread and self.server_thread.is_alive():
            self.server_thread.join(timeout=1.0)
        self.destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = CameraWebSocketBridge()

    def signal_handler(sig, frame):
        node.get_logger().info("Received shutdown signal")
        node.shutdown()
        rclpy.shutdown()
        sys.exit(0)

    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()


if __name__ == "__main__":
    main()
