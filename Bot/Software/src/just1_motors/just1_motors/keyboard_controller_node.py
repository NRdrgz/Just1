import rclpy
from rclpy.node import Node
from just1_interface.msg import WheelSpeeds
from just1_motors.utils_motors import setup, stop_all, control_wheel, cleanup
import keyboard
import threading
import time


class KeyboardMotorController(Node):
    """
    This node is used to control the robot using keyboard keys.
    It reads keyboard input and publishes the wheel speeds to the wheel_speeds topic.
    
    Controls:
    - Up Arrow: Move forward
    - Down Arrow: Move backward
    - Left Arrow: Spin counterclockwise
    - Right Arrow: Spin clockwise
    """

    def __init__(self):
        super().__init__("keyboard_controller_node")

        # Initialize GPIO and motors
        setup()

        # Create publisher for wheel speeds
        self.publisher = self.create_publisher(WheelSpeeds, "wheel_speeds", 10)

        # Start keyboard listener in a separate thread
        self.keyboard_thread = threading.Thread(target=self._keyboard_listener)
        self.keyboard_thread.daemon = True
        self.keyboard_thread.start()

        # Start control loop in a separate thread
        self.control_thread = threading.Thread(target=self._control_loop)
        self.control_thread.daemon = True
        self.control_thread.start()

        self.get_logger().info("Keyboard Motor Controller initialized")

    def _keyboard_listener(self):
        """Listen for keyboard events"""
        keyboard.wait()

    def _control_loop(self):
        """Main control loop that updates wheel speeds based on currently pressed keys"""
        while rclpy.ok():
            wheel_speeds = self._calculate_wheel_speeds()
            
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

            time.sleep(0.01)  # Control loop frequency

    def _calculate_wheel_speeds(self):
        """Calculate wheel speeds based on currently pressed keys"""
        speeds = {"front_left": 0, "front_right": 0, "back_left": 0, "back_right": 0}
        base_speed = 40  # Base speed for movements

        # Check which keys are currently pressed
        if keyboard.is_pressed('up'):
            speeds = {wheel: base_speed for wheel in speeds.keys()}
        elif keyboard.is_pressed('down'):
            speeds = {wheel: -base_speed for wheel in speeds.keys()}
        elif keyboard.is_pressed('left'):
            speeds["front_left"] = -base_speed
            speeds["front_right"] = base_speed
            speeds["back_left"] = -base_speed
            speeds["back_right"] = base_speed
        elif keyboard.is_pressed('right'):
            speeds["front_left"] = base_speed
            speeds["front_right"] = -base_speed
            speeds["back_left"] = base_speed
            speeds["back_right"] = -base_speed

        return speeds


def main(args=None):
    rclpy.init(args=args)
    node = KeyboardMotorController()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Stop all motors and cleanup GPIO before destroying the node
        stop_all()
        cleanup()
        node.destroy_node()


if __name__ == "__main__":
    main() 