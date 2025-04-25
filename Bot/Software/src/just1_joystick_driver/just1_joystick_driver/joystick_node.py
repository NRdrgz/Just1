import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
import pygame


class JoystickDriver(Node):
    def __init__(self):
        super().__init__("joystick_driver")

        # Initialize pygame for joystick support
        pygame.display.init()
        pygame.joystick.init()

        # Check if any joysticks are connected
        joystick_count = pygame.joystick.get_count()
        if joystick_count == 0:
            self.get_logger().error(
                "No controllers detected. Please connect a controller and restart the program."
            )
            raise RuntimeError("No joystick detected")

        self.get_logger().info(f"Found {joystick_count} controller(s)")

        # Initialize the first joystick
        self.joystick = pygame.joystick.Joystick(0)
        self.joystick.init()

        self.get_logger().info(f"Controller Information:")
        self.get_logger().info(f"Name: {self.joystick.get_name()}")
        self.get_logger().info(f"ID: {self.joystick.get_id()}")

        # Create publisher for joystick data
        # Uses standard Joy message type
        self.publisher = self.create_publisher(Joy, "joy", 10)

        # Create timer for reading joystick data
        self.timer = self.create_timer(0.01, self.timer_callback)  # 100Hz

        self.get_logger().info("Joystick Driver initialized")

    def timer_callback(self):
        # Process pygame events
        for event in pygame.event.get():
            if event.type == pygame.JOYAXISMOTION:
                # Create and publish Joy message
                joy_msg = Joy()
                joy_msg.header.stamp = self.get_clock().now().to_msg()
                joy_msg.axes = [
                    self.joystick.get_axis(i)
                    for i in range(self.joystick.get_numaxes())
                ]
                joy_msg.buttons = [
                    self.joystick.get_button(i)
                    for i in range(self.joystick.get_numbuttons())
                ]
                self.publisher.publish(joy_msg)

    def __del__(self):
        """Cleanup when node is destroyed"""
        pygame.quit()


def main(args=None):
    rclpy.init(args=args)
    node = JoystickDriver()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()


if __name__ == "__main__":
    main()
