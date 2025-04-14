import rclpy
from rclpy.node import Node
from robot_state_publisher.msg import WheelSpeeds


class RobotStatePublisher(Node):
    def __init__(self):
        super().__init__("robot_state_publisher")

        # Create subscriber for wheel speeds
        self.subscription = self.create_subscription(
            WheelSpeeds, "wheel_speeds", self.wheel_speeds_callback, 10
        )

        self.get_logger().info("Robot State Publisher initialized")

    def wheel_speeds_callback(self, msg):
        # Print wheel speeds
        self.get_logger().info(
            f"Wheel Speeds - FL: {msg.front_left:.2f}, FR: {msg.front_right:.2f}, "
            f"BL: {msg.back_left:.2f}, BR: {msg.back_right:.2f}"
        )


def main(args=None):
    rclpy.init(args=args)
    node = RobotStatePublisher()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
