import RPi.GPIO as GPIO
import time
from utils_motor import setup, stop_all, test_wheel


def main():
    setup()

    while True:
        print("\nWheel Test Menu:")
        print("1: Front Left")
        print("2: Front Right")
        print("3: Back Left")
        print("4: Back Right")
        print("5: Stop All")
        print("6: Exit")

        choice = input("Select wheel to test (1-6): ")

        if choice == "1":
            print("Testing Front Left wheel")
            test_wheel("front_left")
        elif choice == "2":
            print("Testing Front Right wheel")
            test_wheel("front_right")
        elif choice == "3":
            print("Testing Back Left wheel")
            test_wheel("back_left")
        elif choice == "4":
            print("Testing Back Right wheel")
            test_wheel("back_right")
        elif choice == "5":
            print("Stopping all motors")
            stop_all()
        elif choice == "6":
            print("Exiting...")
            stop_all()
            GPIO.cleanup()
            break
        else:
            print("Invalid choice! Please select 1-6")


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\nProgram stopped by user")
        stop_all()
        GPIO.cleanup()
