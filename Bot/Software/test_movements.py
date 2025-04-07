import time
from utils_motor import setup, stop_all, control_wheel

# Movement speed (0-100)
SPEED = 50


def move_forward():
    """Move the robot forward"""
    print("Moving forward...")
    # All wheels forward
    control_wheel("front_left", "forward", SPEED)
    control_wheel("front_right", "forward", SPEED)
    control_wheel("back_left", "forward", SPEED)
    control_wheel("back_right", "forward", SPEED)


def move_backward():
    """Move the robot backward"""
    print("Moving backward...")
    # All wheels backward
    control_wheel("front_left", "backward", SPEED)
    control_wheel("front_right", "backward", SPEED)
    control_wheel("back_left", "backward", SPEED)
    control_wheel("back_right", "backward", SPEED)


def move_left():
    """Move the robot left (strafing)"""
    print("Moving left...")
    # Front left and back right backward, front right and back left forward
    control_wheel("front_left", "backward", SPEED)
    control_wheel("front_right", "forward", SPEED)
    control_wheel("back_left", "forward", SPEED)
    control_wheel("back_right", "backward", SPEED)


def move_right():
    """Move the robot right (strafing)"""
    print("Moving right...")
    # Front left and back right forward, front right and back left backward
    control_wheel("front_left", "forward", SPEED)
    control_wheel("front_right", "backward", SPEED)
    control_wheel("back_left", "backward", SPEED)
    control_wheel("back_right", "forward", SPEED)


def spin_clockwise():
    """Spin the robot clockwise in place"""
    print("Spinning clockwise...")
    # Left wheels forward, right wheels backward
    control_wheel("front_left", "forward", SPEED)
    control_wheel("front_right", "backward", SPEED)
    control_wheel("back_left", "forward", SPEED)
    control_wheel("back_right", "backward", SPEED)


def spin_counterclockwise():
    """Spin the robot counterclockwise in place"""
    print("Spinning counterclockwise...")
    # Left wheels backward, right wheels forward
    control_wheel("front_left", "backward", SPEED)
    control_wheel("front_right", "forward", SPEED)
    control_wheel("back_left", "backward", SPEED)
    control_wheel("back_right", "forward", SPEED)


def move_diagonal_forward_left():
    """Move diagonally forward-left"""
    print("Moving diagonally forward-left...")
    # Front right and back left forward, others stopped
    control_wheel("front_left", "forward", 0)
    control_wheel("front_right", "forward", SPEED)
    control_wheel("back_left", "forward", SPEED)
    control_wheel("back_right", "forward", 0)


def move_diagonal_forward_right():
    """Move diagonally forward-right"""
    print("Moving diagonally forward-right...")
    # Front left and back right forward, others stopped
    control_wheel("front_left", "forward", SPEED)
    control_wheel("front_right", "forward", 0)
    control_wheel("back_left", "forward", 0)
    control_wheel("back_right", "forward", SPEED)


def move_diagonal_backward_right():
    """Move diagonally backward-right"""
    print("Moving diagonally backward-right...")
    # Front right and back left backward, others stopped
    control_wheel("front_left", "backward", 0)
    control_wheel("front_right", "backward", SPEED)
    control_wheel("back_left", "backward", SPEED)
    control_wheel("back_right", "backward", 0)


def move_diagonal_backward_left():
    """Move diagonally backward-left"""
    print("Moving diagonally backward-left...")
    # Front left and back right forward, others stopped
    control_wheel("front_left", "backward", SPEED)
    control_wheel("front_right", "backward", 0)
    control_wheel("back_left", "backward", 0)
    control_wheel("back_right", "backward", SPEED)


def move_curve_to_right():
    """Move the robot to the right"""
    print("Moving to the right...")
    # Front left and back right forward, others stopped
    control_wheel("front_left", "forward", SPEED * 1.2)
    control_wheel("front_right", "forward", SPEED)
    control_wheel("back_left", "forward", SPEED * 1.2)
    control_wheel("back_right", "forward", SPEED)


def move_curve_to_left():
    """Move the robot to the left"""
    print("Moving to the left...")
    # Front left and back right forward, others stopped
    control_wheel("front_left", "forward", SPEED)
    control_wheel("front_right", "forward", SPEED * 1.2)
    control_wheel("back_left", "forward", SPEED)
    control_wheel("back_right", "forward", SPEED * 1.2)


def move_lateral_arc():
    """Move the robot in a lateral arc"""
    print("Moving in a lateral arc...")
    # Front left forward, front right backward
    control_wheel("front_left", "forward", SPEED)
    control_wheel("front_right", "backward", SPEED)
    control_wheel("back_left", "backward", 0)
    control_wheel("back_right", "forward", 0)


def do_square():
    """Execute a square pattern"""
    print("Starting square pattern...")
    movements = [
        (move_forward, 2),
        (stop_all, 1),
        (move_right, 2),
        (stop_all, 1),
        (move_backward, 2),
        (stop_all, 1),
        (move_left, 2),
        (stop_all, 1),
    ]

    for movement, duration in movements:
        movement()
        time.sleep(duration)
        stop_all()
        time.sleep(0.5)  # Brief pause between movements


def main():
    setup()

    try:
        print("\nMecanum Wheel Movement Test Menu:")
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
        print("0: Stop All")
        print("q: Quit")

        while True:
            choice = input("\nSelect movement (0-9, q to quit): ").lower()

            if choice == "q":
                break
            elif choice == "0":
                stop_all()
            elif choice == "1":
                move_forward()
                time.sleep(2)
                stop_all()
            elif choice == "2":
                move_backward()
                time.sleep(2)
                stop_all()
            elif choice == "3":
                move_left()
                time.sleep(2)
                stop_all()
            elif choice == "4":
                move_right()
                time.sleep(2)
                stop_all()
            elif choice == "5":
                spin_clockwise()
                time.sleep(2)
                stop_all()
            elif choice == "6":
                spin_counterclockwise()
                time.sleep(2)
                stop_all()
            elif choice == "7":
                move_diagonal_forward_right()
                time.sleep(2)
                stop_all()
            elif choice == "8":
                move_diagonal_forward_left()
                time.sleep(2)
                stop_all()
            elif choice == "9":
                move_diagonal_backward_right()
                time.sleep(2)
                stop_all()
            elif choice == "10":
                move_diagonal_backward_left()
                time.sleep(2)
                stop_all()
            elif choice == "11":
                move_curve_to_right()
                time.sleep(2)
                stop_all()
            elif choice == "12":
                move_curve_to_left()
                time.sleep(2)
                stop_all()
            elif choice == "13":
                move_lateral_arc()
                time.sleep(2)
                stop_all()
            elif choice == "14":
                do_square()
            else:
                print("Invalid choice! Please select 0-9 or q")

    except KeyboardInterrupt:
        print("\nProgram stopped by user")
    finally:
        stop_all()
        from utils_motor import cleanup

        cleanup()


if __name__ == "__main__":
    main()
