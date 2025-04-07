import RPi.GPIO as GPIO
import time

# Define GPIO pins for L298N #1 (Front Wheels)
front_left_in1 = 6
front_left_in2 = 5
front_left_ena = 12  # PWM
front_right_in1 = 13
front_right_in2 = 19
front_right_ena = 16  # PWM

# Define GPIO pins for L298N #2 (Back Wheels)
back_left_in1 = 22
back_left_in2 = 27
back_left_ena = 23  # PWM
back_right_in1 = 4
back_right_in2 = 17
back_right_ena = 18  # PWM

PWM_FREQ = 1000  # 1000 Hz frequency

# Global PWM objects
pwm_front_left = None
pwm_front_right = None
pwm_back_left = None
pwm_back_right = None


def setup():
    """Initialize GPIO pins and PWM channels"""
    # Set GPIO mode
    GPIO.setmode(GPIO.BCM)
    GPIO.setwarnings(False)

    # Setup all pins as outputs
    pins = [
        front_left_in1,
        front_left_in2,
        front_left_ena,
        front_right_in1,
        front_right_in2,
        front_right_ena,
        back_left_in1,
        back_left_in2,
        back_left_ena,
        back_right_in1,
        back_right_in2,
        back_right_ena,
    ]

    for pin in pins:
        GPIO.setup(pin, GPIO.OUT)

    # Setup PWM pins
    global pwm_front_left, pwm_front_right, pwm_back_left, pwm_back_right
    pwm_front_left = GPIO.PWM(front_left_ena, PWM_FREQ)
    pwm_front_right = GPIO.PWM(front_right_ena, PWM_FREQ)
    pwm_back_left = GPIO.PWM(back_left_ena, PWM_FREQ)
    pwm_back_right = GPIO.PWM(back_right_ena, PWM_FREQ)

    # Start PWM with 0% duty cycle
    pwm_front_left.start(0)
    pwm_front_right.start(0)
    pwm_back_left.start(0)
    pwm_back_right.start(0)


def stop_all():
    """Stop all motors"""
    # Stop all motors
    pins = [
        front_left_in1,
        front_left_in2,
        front_right_in1,
        front_right_in2,
        back_left_in1,
        back_left_in2,
        back_right_in1,
        back_right_in2,
    ]
    for pin in pins:
        GPIO.output(pin, GPIO.LOW)

    # Set all PWM to 0
    pwm_front_left.ChangeDutyCycle(0)
    pwm_front_right.ChangeDutyCycle(0)
    pwm_back_left.ChangeDutyCycle(0)
    pwm_back_right.ChangeDutyCycle(0)


def control_wheel(wheel_name, direction, speed):
    """
    Control a specific wheel

    Args:
        wheel_name (str): 'front_left', 'front_right', 'back_left', 'back_right'
        direction (str): 'forward' or 'backward'
        speed (int): 0-100, representing percentage of max speed
    """
    # Map wheel names to their corresponding pins and PWM objects
    wheel_map = {
        "front_left": {
            "in1": front_left_in1,
            "in2": front_left_in2,
            "pwm": pwm_front_left,
        },
        "front_right": {
            "in1": front_right_in1,
            "in2": front_right_in2,
            "pwm": pwm_front_right,
        },
        "back_left": {"in1": back_left_in1, "in2": back_left_in2, "pwm": pwm_back_left},
        "back_right": {
            "in1": back_right_in1,
            "in2": back_right_in2,
            "pwm": pwm_back_right,
        },
    }

    if wheel_name not in wheel_map:
        raise ValueError(
            f"Invalid wheel name: {wheel_name}. Must be one of {list(wheel_map.keys())}"
        )

    if direction not in ["forward", "backward"]:
        raise ValueError("Direction must be 'forward' or 'backward'")

    if not 0 <= speed <= 100:
        raise ValueError("Speed must be between 0 and 100")

    wheel = wheel_map[wheel_name]

    if direction == "forward":
        GPIO.output(wheel["in1"], GPIO.HIGH)
        GPIO.output(wheel["in2"], GPIO.LOW)
    else:  # backward
        GPIO.output(wheel["in1"], GPIO.LOW)
        GPIO.output(wheel["in2"], GPIO.HIGH)

    wheel["pwm"].ChangeDutyCycle(speed)


def test_wheel(wheel_name, duration=2):
    """
    Test a specific wheel by running it forward and backward

    Args:
        wheel_name (str): 'front_left', 'front_right', 'back_left', 'back_right'
        duration (int): Duration in seconds for each direction
    """
    print(f"Testing {wheel_name} wheel...")

    # Forward
    control_wheel(wheel_name, "forward", 50)
    print(f"{wheel_name} spinning forward for {duration} seconds...")
    time.sleep(duration)

    # Stop
    control_wheel(wheel_name, "forward", 0)
    print("Stopping for 1 second...")
    time.sleep(1)

    # Backward
    control_wheel(wheel_name, "backward", 50)
    print(f"{wheel_name} spinning backward for {duration} seconds...")
    time.sleep(duration)

    # Stop
    control_wheel(wheel_name, "backward", 0)


def cleanup():
    """Clean up GPIO resources"""
    stop_all()
    GPIO.cleanup()
