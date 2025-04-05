import RPi.GPIO as GPIO
import time

# Define GPIO pins for L298N #1 (Front Wheels)
front_in1 = 4
front_in2 = 17
front_in3 = 22
front_in4 = 27
front_ena = 18  # PWM
front_enb = 23  # PWM

# Define GPIO pins for L298N #2 (Back Wheels)
back_in1 = 6
back_in2 = 5
back_in3 = 13
back_in4 = 19
back_ena = 12  # PWM
back_enb = 16  # PWM

PWM_FREQ = 1000  # 1000 Hz frequency


def setup():
    # Set GPIO mode
    GPIO.setmode(GPIO.BCM)
    GPIO.setwarnings(False)

    # Setup all pins as outputs
    pins = [
        front_in1,
        front_in2,
        front_in3,
        front_in4,
        front_ena,
        front_enb,
        back_in1,
        back_in2,
        back_in3,
        back_in4,
        back_ena,
        back_enb,
    ]

    for pin in pins:
        GPIO.setup(pin, GPIO.OUT)

    # Setup PWM pins
    global pwm_front_a, pwm_front_b, pwm_back_a, pwm_back_b
    pwm_front_a = GPIO.PWM(front_ena, PWM_FREQ)
    pwm_front_b = GPIO.PWM(front_enb, PWM_FREQ)
    pwm_back_a = GPIO.PWM(back_ena, PWM_FREQ)
    pwm_back_b = GPIO.PWM(back_enb, PWM_FREQ)

    # Start PWM with 0% duty cycle
    pwm_front_a.start(0)
    pwm_front_b.start(0)
    pwm_back_a.start(0)
    pwm_back_b.start(0)


def stop_all():
    # Stop all motors
    pins = [
        front_in1,
        front_in2,
        front_in3,
        front_in4,
        back_in1,
        back_in2,
        back_in3,
        back_in4,
    ]
    for pin in pins:
        GPIO.output(pin, GPIO.LOW)

    # Set all PWM to 0
    pwm_front_a.ChangeDutyCycle(0)
    pwm_front_b.ChangeDutyCycle(0)
    pwm_back_a.ChangeDutyCycle(0)
    pwm_back_b.ChangeDutyCycle(0)


def test_wheel(in1, in2, pwm):
    try:
        # Forward
        GPIO.output(in1, GPIO.HIGH)
        GPIO.output(in2, GPIO.LOW)
        pwm.ChangeDutyCycle(50)  # 50% speed
        print("Spinning forward for 2 seconds...")
        time.sleep(2)

        # Stop
        GPIO.output(in1, GPIO.LOW)
        GPIO.output(in2, GPIO.LOW)
        pwm.ChangeDutyCycle(0)
        print("Stopping for 1 second...")
        time.sleep(1)

        # Backward
        GPIO.output(in1, GPIO.LOW)
        GPIO.output(in2, GPIO.HIGH)
        pwm.ChangeDutyCycle(50)  # 50% speed
        print("Spinning backward for 2 seconds...")
        time.sleep(2)

        # Stop
        GPIO.output(in1, GPIO.LOW)
        GPIO.output(in2, GPIO.LOW)
        pwm.ChangeDutyCycle(0)

    except KeyboardInterrupt:
        stop_all()


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
            test_wheel(front_in1, front_in2, pwm_front_a)
        elif choice == "2":
            print("Testing Front Right wheel")
            test_wheel(front_in3, front_in4, pwm_front_b)
        elif choice == "3":
            print("Testing Back Left wheel")
            test_wheel(back_in1, back_in2, pwm_back_a)
        elif choice == "4":
            print("Testing Back Right wheel")
            test_wheel(back_in3, back_in4, pwm_back_b)
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
