import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from robot_interface.msg import WheelSpeeds
from robot_utils.motor_control import setup, stop_all, control_wheel, cleanup


class ManualMotorController(Node):
    def __init__(self):
        super().__init__("manual_motor_controller")

        # Initialize GPIO and motors
        setup()

        # Create subscriber for joystick input
        self.subscription = self.create_subscription(Joy, "joy", self.joy_callback, 10)

        # Create publisher for wheel speeds. Published at the same speed as the Joystick message
        self.publisher = self.create_publisher(WheelSpeeds, "wheel_speeds", 10)

        self.get_logger().info("Manual Motor Controller initialized")

    def joy_callback(self, msg):
        # Get joystick values (assuming left stick)
        vertical_value = msg.axes[1]  # Left stick vertical
        horizontal_value = msg.axes[0]  # Left stick horizontal

        # Calculate wheel speeds
        wheel_speeds = self.calculate_wheel_speeds(vertical_value, horizontal_value)

        # Apply speeds to wheels
        for wheel, speed in wheel_speeds.items():
            control_wheel(wheel, speed)

        # Publish wheel speeds
        speeds_msg = WheelSpeeds()
        speeds_msg.front_left = wheel_speeds["front_left"]
        speeds_msg.front_right = wheel_speeds["front_right"]
        speeds_msg.back_left = wheel_speeds["back_left"]
        speeds_msg.back_right = wheel_speeds["back_right"]
        self.publisher.publish(speeds_msg)

    def calculate_wheel_speeds(self, vertical_value, horizontal_value):
        """
        Calculate the speed for each wheel based on vertical and horizontal values.
        Values range from -1 to 1, where:
        - vertical: -1 is full forward, 1 is full backward
        - horizontal: -1 is full left, 1 is full right

        Returns a dictionary with speeds for each wheel (-100 to 100)
        """
        # Initialize speeds
        speeds = {"front_left": 0, "front_right": 0, "back_left": 0, "back_right": 0}

        # Apply deadzone so that the robot doesn't move when the controller is in the center
        if abs(vertical_value) <= 0.1 and abs(horizontal_value) <= 0.1:
            return speeds

        # Calculate base speed (-100 to 100)
        vertical_speed = vertical_value * 100
        horizontal_speed = horizontal_value * 100

        # Handle vertical movement
        if vertical_value < -0.1:  # Forward
            speeds = {wheel: vertical_speed for wheel in speeds.keys()}
        elif vertical_value > 0.1:  # Backward
            speeds = {wheel: vertical_speed for wheel in speeds.keys()}

        # Handle horizontal movement
        if horizontal_value < -0.1:  # Turn left
            # Right wheels get additional speed
            speeds["front_right"] += horizontal_speed
            speeds["back_right"] += horizontal_speed
            # Left wheels get reduced speed
            speeds["front_left"] = max(-100, speeds["front_left"] - horizontal_speed)
            speeds["back_left"] = max(-100, speeds["back_left"] - horizontal_speed)
        elif horizontal_value > 0.1:  # Turn right
            # Left wheels get additional speed
            speeds["front_left"] += horizontal_speed
            speeds["back_left"] += horizontal_speed
            # Right wheels get reduced speed
            speeds["front_right"] = max(-100, speeds["front_right"] - horizontal_speed)
            speeds["back_right"] = max(-100, speeds["back_right"] - horizontal_speed)

        # Ensure speeds are within -100 to 100 range
        for wheel in speeds:
            speeds[wheel] = max(-100, min(100, speeds[wheel]))

        return speeds

    def __del__(self):
        """Cleanup when node is destroyed"""
        stop_all()
        cleanup()


def main(args=None):
    rclpy.init(args=args)
    node = ManualMotorController()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
