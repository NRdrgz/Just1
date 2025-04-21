import rclpy
from rclpy.node import Node
import time
from just1_utils.motor_control import (
    setup,
    stop_all,
    test_wheel,
    cleanup,
    control_wheel,
)
from sensor_msgs.msg import Joy


class DiagnosticsNode(Node):
    def __init__(self):
        super().__init__("diagnostics_node")
        self.get_logger().info("Diagnostics Node initialized")

        # Declare parameters
        self.declare_parameter("test_wheel", "")
        self.declare_parameter("test_movement", "")
        self.declare_parameter("test_joystick", False)
        self.declare_parameter("speed", 50)

        # Initialize GPIO and motors
        setup()

        # Create subscriber for joystick
        self.joystick_subscriber = self.create_subscription(
            Joy, "joy", self.joystick_callback, 10
        )
        self.joystick_data = None

        # Run diagnostics based on parameters
        self.run_diagnostics()

    def joystick_callback(self, msg):
        """Callback for joystick messages"""
        self.joystick_data = msg

    def run_diagnostics(self):
        """Run diagnostics based on parameters"""
        test_wheel = self.get_parameter("test_wheel").get_parameter_value().string_value
        test_movement = (
            self.get_parameter("test_movement").get_parameter_value().string_value
        )
        test_joystick = (
            self.get_parameter("test_joystick").get_parameter_value().bool_value
        )
        speed = self.get_parameter("speed").get_parameter_value().integer_value

        if test_wheel:
            self.test_wheel(test_wheel)
            self.cleanup_and_shutdown()
        elif test_movement:
            self.test_movement(test_movement, speed)
            self.cleanup_and_shutdown()
        elif test_joystick:
            self.test_joystick()
        else:
            self.get_logger().error("No valid test option specified")
            self.cleanup_and_shutdown()

    def cleanup_and_shutdown(self):
        """Cleanup and shutdown the node"""
        cleanup()
        self.get_logger().info("Diagnostics completed")
        rclpy.shutdown()

    def test_joystick(self):
        """Display joystick values"""
        print("\nJoystick Test Mode")
        print("Press Ctrl+C to exit")
        print("Waiting for joystick data...")

        try:
            while True:
                if self.joystick_data is not None:
                    print("\n" + "=" * 50)
                    print("Joystick Data:")
                    print(f"Axes: {[f'{x:.2f}' for x in self.joystick_data.axes]}")
                    print(f"Buttons: {self.joystick_data.buttons}")
                    print("=" * 50)
                time.sleep(0.1)  # Reduce CPU usage
        except KeyboardInterrupt:
            print("\nExiting joystick test mode...")
            self.cleanup_and_shutdown()
            return

    def test_wheel(self, wheel_name):
        """Test a specific wheel"""
        self.get_logger().info(f"Testing {wheel_name} wheel")
        test_wheel(wheel_name)
        time.sleep(2)
        stop_all()

    def test_movement(self, movement_name, speed):
        """Test a specific movement"""
        self.get_logger().info(f"Testing {movement_name} movement at speed {speed}")

        movement_map = {
            "forward": self.move_forward,
            "backward": self.move_backward,
            "left": self.move_left,
            "right": self.move_right,
            "spin_clockwise": self.spin_clockwise,
            "spin_counterclockwise": self.spin_counterclockwise,
            "diagonal_forward_right": self.move_diagonal_forward_right,
            "diagonal_forward_left": self.move_diagonal_forward_left,
            "diagonal_backward_right": self.move_diagonal_backward_right,
            "diagonal_backward_left": self.move_diagonal_backward_left,
            "curve_right": self.move_curve_to_right,
            "curve_left": self.move_curve_to_left,
            "lateral_arc": self.move_lateral_arc,
            "square": self.do_square,
        }

        if movement_name in movement_map:
            movement_map[movement_name](speed)
            time.sleep(2)
            stop_all()
        else:
            self.get_logger().error(f"Invalid movement name: {movement_name}")

    # Movement functions
    def move_forward(self, speed):
        control_wheel("front_left", speed)
        control_wheel("front_right", speed)
        control_wheel("back_left", speed)
        control_wheel("back_right", speed)

    def move_backward(self, speed):
        control_wheel("front_left", -speed)
        control_wheel("front_right", -speed)
        control_wheel("back_left", -speed)
        control_wheel("back_right", -speed)

    def move_left(self, speed):
        control_wheel("front_left", -speed)
        control_wheel("front_right", speed)
        control_wheel("back_left", speed)
        control_wheel("back_right", -speed)

    def move_right(self, speed):
        control_wheel("front_left", speed)
        control_wheel("front_right", -speed)
        control_wheel("back_left", -speed)
        control_wheel("back_right", speed)

    def spin_clockwise(self, speed):
        control_wheel("front_left", speed)
        control_wheel("front_right", -speed)
        control_wheel("back_left", speed)
        control_wheel("back_right", -speed)

    def spin_counterclockwise(self, speed):
        control_wheel("front_left", -speed)
        control_wheel("front_right", speed)
        control_wheel("back_left", -speed)
        control_wheel("back_right", speed)

    def move_diagonal_forward_right(self, speed):
        control_wheel("front_left", speed)
        control_wheel("front_right", 0)
        control_wheel("back_left", 0)
        control_wheel("back_right", speed)

    def move_diagonal_forward_left(self, speed):
        control_wheel("front_left", 0)
        control_wheel("front_right", speed)
        control_wheel("back_left", speed)
        control_wheel("back_right", 0)

    def move_diagonal_backward_right(self, speed):
        control_wheel("front_left", 0)
        control_wheel("front_right", -speed)
        control_wheel("back_left", -speed)
        control_wheel("back_right", 0)

    def move_diagonal_backward_left(self, speed):
        control_wheel("front_left", -speed)
        control_wheel("front_right", 0)
        control_wheel("back_left", 0)
        control_wheel("back_right", -speed)

    def move_curve_to_right(self, speed):
        control_wheel("front_left", speed * 1.2)
        control_wheel("front_right", speed)
        control_wheel("back_left", speed * 1.2)
        control_wheel("back_right", speed)

    def move_curve_to_left(self, speed):
        control_wheel("front_left", speed)
        control_wheel("front_right", speed * 1.2)
        control_wheel("back_left", speed)
        control_wheel("back_right", speed * 1.2)

    def move_lateral_arc(self, speed):
        control_wheel("front_left", speed)
        control_wheel("front_right", -speed)
        control_wheel("back_left", 0)
        control_wheel("back_right", 0)

    def do_square(self, speed):
        movements = [
            (self.move_forward, 2),
            (stop_all, 1),
            (self.move_right, 2),
            (stop_all, 1),
            (self.move_backward, 2),
            (stop_all, 1),
            (self.move_left, 2),
            (stop_all, 1),
        ]

        for movement, duration in movements:
            movement(speed)
            time.sleep(duration)
            stop_all()
            time.sleep(0.5)


def main(args=None):
    rclpy.init(args=args)
    node = DiagnosticsNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Keyboard interrupt received, shutting down...")
        node.cleanup_and_shutdown()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
