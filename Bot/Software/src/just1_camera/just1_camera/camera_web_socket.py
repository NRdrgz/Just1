import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import asyncio
import websockets
import threading
import base64

class CameraWebSocketBridge(Node):
    def __init__(self):
        super().__init__('camera_web_socket')
        self.bridge = CvBridge()
        self.frame = None

        # Subscribe to camera topic
        self.subscription = self.create_subscription(
            Image,
            'camera/image_raw',
            self.listener_callback,
            10
        )

        # Start the WebSocket server in a separate thread
        self.server_thread = threading.Thread(target=self.start_server)
        self.server_thread.start()
        self.get_logger().info("WebSocket server started")

    def listener_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            _, jpeg = cv2.imencode('.jpg', cv_image)
            self.frame = base64.b64encode(jpeg.tobytes()).decode('utf-8')
        except Exception as e:
            self.get_logger().error(f"Failed to convert image: {e}")

    async def send_frames(self, websocket):
        while True:
            if self.frame:
                await websocket.send(self.frame)
            await asyncio.sleep(0.033)  # ~30 FPS

    def start_server(self):
        async def run():
            # Start the WebSocket server when a client connects
            async with websockets.serve(self.send_frames, '0.0.0.0', 8765):
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
        rclpy.shutdown()

if __name__ == '__main__':
    main()