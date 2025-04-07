import pygame
import time
import sys


def main():
    """Test function to verify controller connection and inputs"""
    print("Initializing pygame...")
    pygame.init()
    pygame.joystick.init()

    # Check if any joysticks are connected
    joystick_count = pygame.joystick.get_count()
    if joystick_count == 0:
        print(
            "No controllers detected. Please connect a controller and restart the program."
        )
        return

    print(f"Found {joystick_count} controller(s)")

    # Initialize the first joystick
    joystick = pygame.joystick.Joystick(0)
    joystick.init()

    # Get controller information
    print(f"\nController Information:")
    print(f"Name: {joystick.get_name()}")
    print(f"ID: {joystick.get_id()}")
    print(f"Number of axes: {joystick.get_numaxes()}")
    print(f"Number of buttons: {joystick.get_numbuttons()}")
    print(f"Number of hats: {joystick.get_numhats()}")

    print("\nPress Ctrl+C to exit the test.")
    print("Move the joysticks to see the values change.")

    try:
        # Main loop to display controller inputs
        while True:
            # Process pygame events
            for event in pygame.event.get():
                if event.type == pygame.JOYAXISMOTION:
                    print(f"Axis {event.axis}: {event.value:.3f}")
                elif event.type == pygame.JOYBUTTONDOWN:
                    print(f"Button {event.button} pressed")
                elif event.type == pygame.JOYBUTTONUP:
                    print(f"Button {event.button} released")
                elif event.type == pygame.JOYHATMOTION:
                    print(f"Hat {event.hat}: {event.value}")

            # Get current values of all axes
            axes_values = [joystick.get_axis(i) for i in range(joystick.get_numaxes())]

            # Clear the terminal and print current values
            print("\033[H\033[J", end="")  # Clear screen

            print("=== Controller Test ===")
            print(f"Controller: {joystick.get_name()}")
            print("\nAxes Values:")
            for i, value in enumerate(axes_values):
                print(f"Axis {i}: {value:.3f}")

            print("\nPress Ctrl+C to exit")

            # Small delay to prevent high CPU usage
            time.sleep(1)

    except KeyboardInterrupt:
        print("\nTest terminated by user")
    finally:
        pygame.quit()
        print("Pygame closed")


if __name__ == "__main__":
    main()
