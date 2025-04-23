import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image


class RobotStateMonitor(Node):
    def __init__(self):
        super().__init__("just1_state_monitor")

        # Create subscriber for wheel speeds
        self.subscription = self.create_subscription(
            Image, "/camera/image_raw", self.camera_callback, 10
        )

        self.get_logger().info("Just1 State Monitor initialized")

    def camera_callback(self, msg):
        # Print camera image
        self.get_logger().info(f"Camera image received: {msg}")


def main(args=None):
    rclpy.init(args=args)
    node = RobotStateMonitor()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
