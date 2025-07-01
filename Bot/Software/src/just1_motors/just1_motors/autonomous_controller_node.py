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
        Converts linear and angular velocities to normalized wheel speeds for mecanum drive and publishes them.
        """
        # Extract velocities
        linear_x = msg.linear.x
        linear_y = msg.linear.y
        angular_z = msg.angular.z

        # Robot parameters
        Lx = self.wheel_separation_x / 2.0
        Ly = self.wheel_separation_y / 2.0
        wheel_radius = self.wheel_radius
        max_rpm = 123
        min_effective_percent = 50

        # Mecanum wheel speed equations
        front_left = linear_x - linear_y - (Lx + Ly) * angular_z
        front_right = linear_x + linear_y + (Lx + Ly) * angular_z
        back_left = linear_x + linear_y - (Lx + Ly) * angular_z
        back_right = linear_x - linear_y + (Lx + Ly) * angular_z

        # Convert from m/s to RPM (approximate)
        rpm_factor = 60.0 / (2.0 * math.pi * wheel_radius)
        wheel_rpms = {
            "front_left": front_left * rpm_factor,
            "front_right": front_right * rpm_factor,
            "back_left": back_left * rpm_factor,
            "back_right": back_right * rpm_factor,
        }

        # Normalize RPM to percentage between 50 and 100 (or -50 and -100)
        wheel_percentages = {}
        for wheel_name, rpm in wheel_rpms.items():
            rpm = max(-max_rpm, min(max_rpm, rpm))
            percentage = int((rpm / max_rpm) * 100)
            if percentage > 0:
                percentage = int(
                    min_effective_percent + (percentage / 100) * min_effective_percent
                )
            elif percentage < 0:
                percentage = int(
                    -min_effective_percent + (percentage / 100) * min_effective_percent
                )
            # If percentage is 0, keep it as 0
            wheel_percentages[wheel_name] = percentage
            # Control the wheel
            control_wheel(wheel_name, percentage)

        # Publish normalized wheel speeds
        wheel_speeds_msg = WheelSpeeds()
        wheel_speeds_msg.front_left = wheel_percentages["front_left"]
        wheel_speeds_msg.front_right = wheel_percentages["front_right"]
        wheel_speeds_msg.back_left = wheel_percentages["back_left"]
        wheel_speeds_msg.back_right = wheel_percentages["back_right"]
        self.wheel_speeds_publisher.publish(wheel_speeds_msg)

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
