import rclpy
from rclpy.node import Node
import time
from robot_utils.motor_control import setup, stop_all, test_wheel, cleanup


class TestNode(Node):
    def __init__(self):
        super().__init__("test_node")
        self.get_logger().info("Test Node initialized")

        # Initialize GPIO and motors
        setup()

        # Run tests
        self.run_tests()

        # Cleanup
        cleanup()
        self.get_logger().info("Tests completed")
        rclpy.shutdown()

    def run_tests(self):
        """Run a series of tests on the robot's motors"""
        self.get_logger().info("Starting motor tests...")

        # Test each wheel individually
        wheels = ["front_left", "front_right", "back_left", "back_right"]
        for wheel in wheels:
            self.get_logger().info(f"Testing {wheel} wheel...")
            test_wheel(wheel, duration=2)
            time.sleep(1)  # Pause between tests

        # Test all wheels together
        self.get_logger().info("Testing all wheels together...")
        for wheel in wheels:
            test_wheel(wheel, duration=2)

        # Stop all motors
        stop_all()
        self.get_logger().info("All tests completed")


def main(args=None):
    rclpy.init(args=args)
    node = TestNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
