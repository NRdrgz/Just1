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

        self.get_logger().info("Autonomous Motor Controller initialized")

    def cmd_vel_callback(self, msg):
        """
        Callback function for cmd_vel messages.
        Converts linear and angular velocities to wheel speeds for mecanum drive.
        """
        # Extract velocities
        linear_x = msg.linear.x
        linear_y = msg.linear.y
        angular_z = msg.angular.z

        # Calculate wheel speeds for mecanum drive (in RPM)
        wheel_speeds = self.calculate_mecanum_wheel_speeds(
            linear_x, linear_y, angular_z
        )

        # Normalize wheel speeds to percent (50-100 or -50 to -100)
        normalized_wheel_speeds = self.normalize_wheel_speeds(wheel_speeds)

        # Publish normalized wheel speeds
        wheel_speeds_msg = WheelSpeeds()
        wheel_speeds_msg.front_left = normalized_wheel_speeds["front_left"]
        wheel_speeds_msg.front_right = normalized_wheel_speeds["front_right"]
        wheel_speeds_msg.back_left = normalized_wheel_speeds["back_left"]
        wheel_speeds_msg.back_right = normalized_wheel_speeds["back_right"]

        self.wheel_speeds_publisher.publish(wheel_speeds_msg)

        # Control motors with normalized speeds
        self.control_motors(normalized_wheel_speeds)

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

    def normalize_wheel_speeds(self, wheel_speeds):
        """
        Normalize wheel speeds to be between 50 and 100 (or -50 and -100) when not zero.
        Args:
            wheel_speeds: dict containing speeds for each wheel (in RPM)
        Returns:
            dict: Normalized wheel speeds (in percent)
        """
        max_rpm = 123
        min_effective_percent = 50
        normalized = {}
        for wheel_name, rpm in wheel_speeds.items():
            # Clamp RPM to max_rpm
            rpm = max(-max_rpm, min(max_rpm, rpm))
            # Convert to percentage
            percentage = int((rpm / max_rpm) * 100)
            # Normalize percentage to be between 50 and 100 (or -50 and -100) when not zero
            if percentage > 0:
                percentage = int(
                    min_effective_percent + (percentage / 100) * min_effective_percent
                )
            elif percentage < 0:
                percentage = int(
                    -min_effective_percent + (percentage / 100) * min_effective_percent
                )
            # If percentage is 0, keep it as 0
            normalized[wheel_name] = percentage
        return normalized

    def control_motors(self, wheel_speeds):
        """
        Control motors based on calculated wheel speeds.

        Args:
            wheel_speeds: dict containing speeds for each wheel (in percent)
        """
        for wheel_name, percentage in wheel_speeds.items():
            control_wheel(wheel_name, percentage)

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
