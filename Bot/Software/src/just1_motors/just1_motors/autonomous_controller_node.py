import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from just1_interface.msg import WheelSpeeds
from just1_motors.utils_motors import setup, stop_all, control_wheel, cleanup
import math


class AutonomousMotorController(Node):
    """
    Listens to the /cmd_vel topic and publishes the wheel speeds to the wheel_speeds topic.
    Controls a mecanum drive robot with 4 wheels.
    """

    def __init__(self):
        super().__init__("autonomous_motor_controller")

        # Initialize motor hardware
        setup()

        # Robot parameters for mecanum drive
        self.wheel_radius = 0.035  # meters (35mm radius)
        self.wheel_separation_x = (
            0.11  # meters (distance between front and back wheels)
        )
        self.wheel_separation_y = (
            0.13  # meters (distance between left and right wheels)
        )

        # Create subscriber for cmd_vel
        self.cmd_vel_subscription = self.create_subscription(
            Twist, "cmd_vel", self.cmd_vel_callback, 10
        )

        # Create publisher for wheel speeds
        self.wheel_speeds_publisher = self.create_publisher(
            WheelSpeeds, "wheel_speeds", 10
        )

        # Create timer for safety timeout
        self.last_cmd_time = self.get_clock().now()
        self.cmd_timeout = 0.5  # seconds
        self.safety_timer = self.create_timer(0.1, self.safety_check)

        self.get_logger().info("Autonomous Motor Controller initialized")

    def cmd_vel_callback(self, msg):
        """
        Callback function for cmd_vel messages.
        Converts linear and angular velocities to wheel speeds for mecanum drive.
        """
        # Update last command time
        self.last_cmd_time = self.get_clock().now()

        # Extract velocities
        linear_x = msg.linear.x
        linear_y = msg.linear.y
        angular_z = msg.angular.z

        # Calculate wheel speeds for mecanum drive
        wheel_speeds = self.calculate_mecanum_wheel_speeds(
            linear_x, linear_y, angular_z
        )

        # Publish wheel speeds
        wheel_speeds_msg = WheelSpeeds()
        wheel_speeds_msg.front_left = wheel_speeds["front_left"]
        wheel_speeds_msg.front_right = wheel_speeds["front_right"]
        wheel_speeds_msg.back_left = wheel_speeds["back_left"]
        wheel_speeds_msg.back_right = wheel_speeds["back_right"]

        self.wheel_speeds_publisher.publish(wheel_speeds_msg)

        # Control motors
        self.control_motors(wheel_speeds)

    def calculate_mecanum_wheel_speeds(self, linear_x, linear_y, angular_z):
        """
        Calculate wheel speeds for mecanum drive based on desired velocities.

        Args:
            linear_x: Forward/backward velocity (m/s)
            linear_y: Left/right velocity (m/s)
            angular_z: Rotational velocity (rad/s)

        Returns:
            dict: Wheel speeds for each wheel
        """
        # Mecanum wheel speed equations
        # Front left: vx - vy - (Lx + Ly) * ω
        # Front right: vx + vy + (Lx + Ly) * ω
        # Back left: vx + vy - (Lx + Ly) * ω
        # Back right: vx - vy + (Lx + Ly) * ω

        Lx = self.wheel_separation_x / 2.0
        Ly = self.wheel_separation_y / 2.0

        front_left = linear_x - linear_y - (Lx + Ly) * angular_z
        front_right = linear_x + linear_y + (Lx + Ly) * angular_z
        back_left = linear_x + linear_y - (Lx + Ly) * angular_z
        back_right = linear_x - linear_y + (Lx + Ly) * angular_z

        # Convert from m/s to RPM (approximate)
        # RPM = (m/s) / (2 * π * radius) * 60
        rpm_factor = 60.0 / (2.0 * math.pi * self.wheel_radius)

        wheel_speeds = {
            "front_left": front_left * rpm_factor,
            "front_right": front_right * rpm_factor,
            "back_left": back_left * rpm_factor,
            "back_right": back_right * rpm_factor,
        }

        return wheel_speeds

    def control_motors(self, wheel_speeds):
        """
        Control motors based on calculated wheel speeds.

        Args:
            wheel_speeds: dict containing speeds for each wheel
        """
        # Convert RPM to percentage (-100 to 100)
        # Max forward speed is 0.5 m/s so max RPM is 123
        max_rpm = 123
        min_absolute_percentage = 50

        for wheel_name, rpm in wheel_speeds.items():
            # Clamp RPM to max_rpm
            rpm = max(-max_rpm, min(max_rpm, rpm))

            # Convert to percentage
            percentage = int((rpm / max_rpm) * 100)

            # If the absolute value of the percentage is less than the minimum absolute percentage, set it to the minimum absolute percentage
            if abs(percentage) < min_absolute_percentage and abs(percentage) >= 0:
                percentage = min_absolute_percentage * (percentage / abs(percentage))

            # Control the wheel
            control_wheel(wheel_name, percentage)

    def safety_check(self):
        """
        Safety check to stop motors if no commands received for a while.
        """
        current_time = self.get_clock().now()
        time_since_last_cmd = (current_time - self.last_cmd_time).nanoseconds / 1e9

        if time_since_last_cmd > self.cmd_timeout:
            # Stop all motors if no command received
            stop_all()

    def on_shutdown(self):
        """
        Cleanup when node is shutting down.
        """
        self.get_logger().info("Shutting down Autonomous Motor Controller")
        stop_all()
        cleanup()


def main(args=None):
    rclpy.init(args=args)

    autonomous_controller = AutonomousMotorController()

    try:
        rclpy.spin(autonomous_controller)
    except KeyboardInterrupt:
        pass
    finally:
        autonomous_controller.on_shutdown()
        autonomous_controller.destroy_node()


if __name__ == "__main__":
    main()
