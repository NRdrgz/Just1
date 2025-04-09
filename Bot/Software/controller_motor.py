import pygame
import time
import sys
from utils_motor import setup, stop_all, control_wheel, cleanup


def calculate_wheel_speeds(vertical_value, horizontal_value):
    """
    Calculate the speed for each wheel based on vertical and horizontal values.
    See controller_motor_guide.png for the controller mapping.
    Values range from -1 to 1, where:
    - vertical: -1 is full forward, 1 is full backward
    - horizontal: -1 is full left, 1 is full right

    Returns a dictionary with speeds for each wheel (0-100)
    """
    # Initialize speeds
    speeds = {"front_left": 0, "front_right": 0, "back_left": 0, "back_right": 0}

    # Apply deadzone so that the robot doesn't move when the controller is in the center
    if abs(vertical_value) <= 0.1 and abs(horizontal_value) <= 0.1:
        return speeds

    # Calculate base speed (0-100)
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

    # Ensure speeds are within 0-100 range
    for wheel in speeds:
        speeds[wheel] = max(0, min(100, abs(speeds[wheel])))

    return speeds


def main():
    """Main function to control the bot using a controller"""
    print("Initializing pygame and GPIO...")
    pygame.init()
    pygame.joystick.init()
    setup()

    # Check if any joysticks are connected
    joystick_count = pygame.joystick.get_count()
    if joystick_count == 0:
        print(
            "No controllers detected. Please connect a controller and restart the program."
        )
        cleanup()
        return

    print(f"Found {joystick_count} controller(s)")

    # Initialize the first joystick
    joystick = pygame.joystick.Joystick(0)
    joystick.init()

    print(f"\nController Information:")
    print(f"Name: {joystick.get_name()}")
    print(f"ID: {joystick.get_id()}")

    print("Move the left joystick to control movement:")
    print("- Up/Down: Forward/Backward")
    print("- Left/Right: Turn Left/Right")
    print("- Diagonal: Combined movement")

    # Initialize axis values
    vertical_value = 0
    horizontal_value = 0

    try:
        while True:
            # Process pygame events
            for event in pygame.event.get():
                if event.type == pygame.JOYAXISMOTION:
                    # Update the current axis values
                    if event.axis == 1:  # Left stick vertical axis
                        vertical_value = event.value
                    elif event.axis == 0:  # Left stick horizontal axis
                        horizontal_value = event.value

            # Calculate wheel speeds
            wheel_speeds = calculate_wheel_speeds(vertical_value, horizontal_value)

            # Apply speeds to wheels
            for wheel, speed in wheel_speeds.items():
                if speed > 0:
                    direction = "forward" if vertical_value < 0 else "backward"
                    control_wheel(wheel, direction, speed)
                else:
                    control_wheel(wheel, "forward", 0)

            # Small delay to prevent high CPU usage
            time.sleep(0.01)

    except KeyboardInterrupt:
        print("\nProgram terminated by user")
    finally:
        stop_all()
        cleanup()
        pygame.quit()
        print("Pygame and GPIO closed")


if __name__ == "__main__":
    main()
