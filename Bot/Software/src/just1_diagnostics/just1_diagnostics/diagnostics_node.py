import rclpy
from rclpy.node import Node
import time
from just1_utils.motor_control import setup, stop_all, test_wheel, cleanup


class DiagnosticsNode(Node):
    def __init__(self):
        super().__init__("diagnostics_node")
        self.get_logger().info("Diagnostics Node initialized")

        # Initialize GPIO and motors
        setup()

        # Run tests
        self.run_diagnostics()

        # Cleanup
        cleanup()
        self.get_logger().info("Diagnostics completed")
        rclpy.shutdown()

    def run_diagnostics(self):
        """Run a series of diagnostics on the robot's motors"""
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
    node = DiagnosticsNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
