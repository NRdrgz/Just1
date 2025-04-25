import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import base64
import asyncio
import websockets
import threading


class CameraWebSocketBridge(Node):
    def __init__(self):
        super().__init__("camera_web_socket")
        self.bridge = CvBridge()

        # Subscribe to the image topic
        self.subscription = self.create_subscription(
            Image, "/camera/image_raw", self.image_callback, 10
        )
        self.get_logger().info("WebSocket bridge node started")

        # Start WebSocket server in a separate thread
        self.server_thread = threading.Thread(target=self.run_ws_server)
        self.server_thread.start()

        # Create a list to hold connected clients
        self.connected_clients = []

    async def image_callback(self, msg):
        try:
            # Convert ROS image message to OpenCV (in RGB format)
            cv_image = self.bridge.imgmsg_to_cv2(msg, "rgb8")
            self.get_logger().info(f"Image shape: {cv_image.shape}")

            # Validate image shape
            height, width = cv_image.shape[:2]
            if width <= 0 or height <= 0:
                self.get_logger().warn("Received image with invalid size.")
                return
            # Resize to 500px width while keeping aspect ratio
            target_width = 500
            aspect_ratio = height / width
            resized_height = int(aspect_ratio * target_width)

            cv_image = cv2.resize(cv_image, (target_width, resized_height))

            # Convert RGB image directly to base64
            # Flatten the array and convert to bytes
            img_bytes = cv_image.tobytes()
            # Encode to base64
            b64_data = base64.b64encode(img_bytes).decode("utf-8")

            # Send the base64 image to all connected clients
            await self.broadcast(b64_data)

        except Exception as e:
            self.get_logger().error(f"Error processing image: {e}")

    async def broadcast(self, message):
        # Send the message to all connected clients
        for client in self.connected_clients:
            try:
                await client.send(message)
            except websockets.exceptions.ConnectionClosed:
                self.connected_clients.remove(client)

    async def websocket_handler(self, websocket, path):
        self.get_logger().info("Client connected")
        self.connected_clients.append(websocket)

        try:
            # Keep the connection open
            await websocket.wait_closed()
        except websockets.exceptions.ConnectionClosed:
            self.get_logger().info("Client disconnected")
            self.connected_clients.remove(websocket)

    def run_ws_server(self):
        loop = asyncio.new_event_loop()
        asyncio.set_event_loop(loop)

        start_server = websockets.serve(self.websocket_handler, "0.0.0.0", 8765)
        loop.run_until_complete(start_server)
        loop.run_forever()

    def destroy_node(self):
        super().destroy_node()


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


if __name__ == "__main__":
    main()
