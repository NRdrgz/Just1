import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from just1_interface.msg import WheelSpeeds
from just1_motors.utils_motors import setup, stop_all, control_wheel, cleanup


class ManualMotorController(Node):
    """
    This node is used to control the robot using a joystick.
    It subscribes to the joystick topic and publishes the wheel speeds to the wheel_speeds topic.
    The left stick is used to control the robot and make it move forward, backward, and turn left and right.
    The right stick is used to control the robot and make it spin clockwise and counterclockwise, or move laterally left or right.
    The left stick has priority over the right stick. The left stick must be in deadzone for right stick to be used.
    A deadzone of 0.1 is applied to the joystick to prevent the robot from moving when the controller is in the center.
    """

    def __init__(self):
        super().__init__("manual_controller_node")

        # Initialize GPIO and motors
        setup()

        # Create subscriber for joystick input
        self.subscription = self.create_subscription(Joy, "joy", self.joy_callback, 10)

        # Create publisher for wheel speeds. Published at the same speed as the Joystick message
        self.publisher = self.create_publisher(WheelSpeeds, "wheel_speeds", 10)

        self.get_logger().info("Manual Motor Controller initialized")

    def joy_callback(self, msg):
        # Get joystick values from the left stick
        vertical_value_left = msg.axes[1]  # Left stick vertical
        horizontal_value_left = msg.axes[0]  # Left stick horizontal

        vertical_value_right = msg.axes[3]  # Right stick vertical
        horizontal_value_right = msg.axes[2]  # Right stick horizontal

        # Check if left stick is out of deadzone and if so, calculate wheel speeds
        if abs(vertical_value_left) > 0.1 or abs(horizontal_value_left) > 0.1:
            wheel_speeds = self.calculate_wheel_speeds_left_stick(
                vertical_value_left, horizontal_value_left
            )
        # Else, use right stick values to control the robot
        else:
            wheel_speeds = self.calculate_wheel_speeds_right_stick(
                vertical_value_right, horizontal_value_right
            )

        # Apply speeds to wheels
        for wheel, speed in wheel_speeds.items():
            control_wheel(wheel, speed)

        # Publish wheel speeds
        speeds_msg = WheelSpeeds()
        speeds_msg.front_left = float(wheel_speeds["front_left"])
        speeds_msg.front_right = float(wheel_speeds["front_right"])
        speeds_msg.back_left = float(wheel_speeds["back_left"])
        speeds_msg.back_right = float(wheel_speeds["back_right"])
        self.publisher.publish(speeds_msg)

    def calculate_wheel_speeds_left_stick(self, vertical_value, horizontal_value):
        """
        Calculate the speed for each wheel based on vertical and horizontal values of the left joystick.
        Values range from -1 to 1, where:
        - vertical: -1 is full forward, 1 is full backward
        - horizontal: -1 is full left, 1 is full right

        Returns a dictionary with speeds for each wheel (-100 to 100)

        It first assigns base speed depending on the vertical value. Then it adds horizontal speed to the right wheels depending on the horizontal value in order to turn left or right.
        """
        # Initialize speeds
        speeds = {"front_left": 0, "front_right": 0, "back_left": 0, "back_right": 0}

        # Apply deadzone so that the robot doesn't move when the controller is in the center
        if abs(vertical_value) <= 0.1 and abs(horizontal_value) <= 0.1:
            return speeds

        # Calculate base speed (0 to 100)
        vertical_speed = abs(vertical_value) * 100
        horizontal_speed = abs(horizontal_value) * 100

        # Handle vertical movement
        if vertical_value < -0.1:  # Forward
            speeds = {wheel: vertical_speed for wheel in speeds.keys()}
        elif vertical_value > 0.1:  # Backward
            speeds = {wheel: -vertical_speed for wheel in speeds.keys()}

        # Handle horizontal movement
        if horizontal_value < -0.1:  # Turn left
            # Right wheels get additional speed
            speeds["front_right"] += horizontal_speed
            speeds["back_right"] += horizontal_speed
            # Left wheels get reduced speed
            speeds["front_left"] = max(0, speeds["front_left"] - horizontal_speed)
            speeds["back_left"] = max(0, speeds["back_left"] - horizontal_speed)
        elif horizontal_value > 0.1:  # Turn right
            # Left wheels get additional speed
            speeds["front_left"] += horizontal_speed
            speeds["back_left"] += horizontal_speed
            # Right wheels get reduced speed
            speeds["front_right"] = max(0, speeds["front_right"] - horizontal_speed)
            speeds["back_right"] = max(0, speeds["back_right"] - horizontal_speed)

        # Ensure speeds are within -100 to 100 range
        for wheel in speeds:
            speeds[wheel] = max(-100, min(100, speeds[wheel]))

        return speeds

    def calculate_wheel_speeds_right_stick(self, vertical_value, horizontal_value):
        """
        Calculate the speed for each wheel based on vertical and horizontal values of the right joystick.
        Values range from -1 to 1, where:
        - vertical: -1 is spin clockwise, 1 is spin counterclockwise
        - horizontal: -1 is move laterally left, 1 is move laterally right

        The movement cannot be at the same time horizontal and vertical. The highest speed will be used.
        """
        speeds = {"front_left": 0, "front_right": 0, "back_left": 0, "back_right": 0}

        # Apply deadzone so that the robot doesn't move when the controller is in the center
        if abs(vertical_value) <= 0.1 and abs(horizontal_value) <= 0.1:
            return speeds

        # Calculate base speed (0 to 100)
        vertical_speed = abs(vertical_value) * 100
        horizontal_speed = abs(horizontal_value) * 100

        # Handle vertical movement
        if vertical_speed >= horizontal_speed:
            if vertical_value < -0.1:  # Spin clockwise
                speeds["front_left"] = vertical_speed
                speeds["front_right"] = -vertical_speed
                speeds["back_left"] = vertical_speed
                speeds["back_right"] = -vertical_speed
            elif vertical_value > 0.1:  # Spin counterclockwise
                speeds["front_left"] = -vertical_speed
                speeds["front_right"] = vertical_speed
                speeds["back_left"] = -vertical_speed
                speeds["back_right"] = vertical_speed
        # Handle horizontal movement
        elif horizontal_speed >= vertical_speed:
            if horizontal_value < -0.1:  # Move laterally left
                speeds["front_left"] = -horizontal_speed
                speeds["front_right"] = horizontal_speed
                speeds["back_left"] = horizontal_speed
                speeds["back_right"] = -horizontal_speed
            elif horizontal_value > 0.1:  # Move laterally right
                speeds["front_left"] = horizontal_speed
                speeds["front_right"] = -horizontal_speed
                speeds["back_left"] = -horizontal_speed
                speeds["back_right"] = horizontal_speed
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


if __name__ == "__main__":
    main()
