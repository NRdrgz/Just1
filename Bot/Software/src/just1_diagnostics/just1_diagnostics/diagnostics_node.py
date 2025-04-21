import rclpy
from rclpy.node import Node
import time
import threading
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

        # Initialize GPIO and motors
        setup()

        # Create subscriber for joystick
        self.joystick_subscriber = self.create_subscription(
            Joy, "joy", self.joystick_callback, 10
        )
        self.joystick_data = None

        # Flag to control menu thread
        self.running = True

        # Start menu in a separate thread
        self.menu_thread = threading.Thread(target=self.run_diagnostics_menu)
        self.menu_thread.start()

    def joystick_callback(self, msg):
        """Callback for joystick messages"""
        self.joystick_data = msg

    def run_diagnostics_menu(self):
        """Run the diagnostics menu"""
        while self.running:
            print("\nDiagnostics Menu:")
            print("1: Test Individual Wheels")
            print("2: Test Movements")
            print("3: Test Joystick")
            print("4: Exit")

            choice = input("Select option (1-4): ")

            if choice == "1":
                self.test_wheels_menu()
            elif choice == "2":
                self.test_movements_menu()
            elif choice == "3":
                self.test_joystick()
            elif choice == "4":
                self.running = False
                break
            else:
                print("Invalid choice! Please select 1-4")

    def test_joystick(self):
        """Display joystick values"""
        print("\nJoystick Test Mode")
        print("Press Enter to exit")
        print("Waiting for joystick data...")

        while self.running:
            if self.joystick_data is not None:
                print("\n" + "=" * 50)
                print("Joystick Data:")
                print(f"Axes: {[f'{x:.2f}' for x in self.joystick_data.axes]}")
                print(f"Buttons: {self.joystick_data.buttons}")
                print("=" * 50)
            time.sleep(0.1)  # Reduce CPU usage

            # Check if Enter was pressed
            if input() == "":
                break

    def test_wheels_menu(self):
        """Menu for testing individual wheels"""
        while True:
            print("\nWheel Test Menu:")
            print("1: Front Left")
            print("2: Front Right")
            print("3: Back Left")
            print("4: Back Right")
            print("5: Stop All")
            print("6: Back to Main Menu")

            choice = input("Select wheel to test (1-6): ")

            if choice == "1":
                self.get_logger().info("Testing Front Left wheel")
                test_wheel("front_left")
            elif choice == "2":
                self.get_logger().info("Testing Front Right wheel")
                test_wheel("front_right")
            elif choice == "3":
                self.get_logger().info("Testing Back Left wheel")
                test_wheel("back_left")
            elif choice == "4":
                self.get_logger().info("Testing Back Right wheel")
                test_wheel("back_right")
            elif choice == "5":
                self.get_logger().info("Stopping all motors")
                stop_all()
            elif choice == "6":
                break
            else:
                print("Invalid choice! Please select 1-6")

    def test_movements_menu(self):
        """Menu for testing different movements"""
        SPEED = 50
        while True:
            print("\nMovement Test Menu:")
            print("1: Forward")
            print("2: Backward")
            print("3: Left")
            print("4: Right")
            print("5: Spin Clockwise")
            print("6: Spin Counterclockwise")
            print("7: Diagonal Forward-Right")
            print("8: Diagonal Forward-Left")
            print("9: Diagonal Backward-Right")
            print("10: Diagonal Backward-Left")
            print("11: Curve to Right")
            print("12: Curve to Left")
            print("13: Lateral Arc")
            print("14: Do Square Pattern")
            print("15: Stop All")
            print("16: Back to Main Menu")

            choice = input("Select movement (1-16): ")

            if choice == "16":
                break
            elif choice == "15":
                stop_all()
            elif choice == "1":
                self.get_logger().info("Moving forward")
                self.move_forward(SPEED)
            elif choice == "2":
                self.get_logger().info("Moving backward")
                self.move_backward(SPEED)
            elif choice == "3":
                self.get_logger().info("Moving left")
                self.move_left(SPEED)
            elif choice == "4":
                self.get_logger().info("Moving right")
                self.move_right(SPEED)
            elif choice == "5":
                self.get_logger().info("Spinning clockwise")
                self.spin_clockwise(SPEED)
            elif choice == "6":
                self.get_logger().info("Spinning counterclockwise")
                self.spin_counterclockwise(SPEED)
            elif choice == "7":
                self.get_logger().info("Moving diagonal forward-right")
                self.move_diagonal_forward_right(SPEED)
            elif choice == "8":
                self.get_logger().info("Moving diagonal forward-left")
                self.move_diagonal_forward_left(SPEED)
            elif choice == "9":
                self.get_logger().info("Moving diagonal backward-right")
                self.move_diagonal_backward_right(SPEED)
            elif choice == "10":
                self.get_logger().info("Moving diagonal backward-left")
                self.move_diagonal_backward_left(SPEED)
            elif choice == "11":
                self.get_logger().info("Moving curve to right")
                self.move_curve_to_right(SPEED)
            elif choice == "12":
                self.get_logger().info("Moving curve to left")
                self.move_curve_to_left(SPEED)
            elif choice == "13":
                self.get_logger().info("Moving lateral arc")
                self.move_lateral_arc(SPEED)
            elif choice == "14":
                self.get_logger().info("Executing square pattern")
                self.do_square(SPEED)
            else:
                print("Invalid choice! Please select 1-16")

            if choice not in ["15", "16"]:
                time.sleep(2)
                stop_all()

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

    def destroy_node(self):
        """Override destroy_node to clean up properly"""
        self.running = False
        if hasattr(self, "menu_thread"):
            self.menu_thread.join()
        cleanup()
        super().destroy_node()


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
