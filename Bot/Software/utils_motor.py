import lgpio
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

# Global handle
h = None


def setup():
    """Initialize GPIO pins and PWM channels"""
    global h

    # Open GPIO chip
    h = lgpio.gpiochip_open(0)

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
        lgpio.gpio_claim_output(h, pin)

    # Initialize PWM on all ENA pins
    for pwm_pin in [front_left_ena, front_right_ena, back_left_ena, back_right_ena]:
        lgpio.tx_pwm(h, pwm_pin, PWM_FREQ, 0)


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
        lgpio.gpio_write(h, pin, 0)

    # Set all PWM to 0
    for pwm_pin in [front_left_ena, front_right_ena, back_left_ena, back_right_ena]:
        lgpio.tx_pwm(h, pwm_pin, PWM_FREQ, 0)


def control_wheel(wheel_name, direction, speed):
    """
    Control a specific wheel

    Args:
        wheel_name (str): 'front_left', 'front_right', 'back_left', 'back_right'
        direction (str): 'forward' or 'backward'
        speed (int): 0-100, representing percentage of max speed
    """
    # Map wheel names to their corresponding pins
    wheel_map = {
        "front_left": {
            "in1": front_left_in1,
            "in2": front_left_in2,
            "ena": front_left_ena,
        },
        "front_right": {
            "in1": front_right_in1,
            "in2": front_right_in2,
            "ena": front_right_ena,
        },
        "back_left": {
            "in1": back_left_in1,
            "in2": back_left_in2,
            "ena": back_left_ena,
        },
        "back_right": {
            "in1": back_right_in1,
            "in2": back_right_in2,
            "ena": back_right_ena,
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
    # Convert speed (0-100) to duty cycle (0-1000000)
    duty_cycle = int((speed / 100.0) * 1000000)
    # Ensure duty cycle is within valid range
    duty_cycle = max(0, min(1000000, duty_cycle))

    if direction == "forward":
        lgpio.gpio_write(h, wheel["in1"], 1)
        lgpio.gpio_write(h, wheel["in2"], 0)
    else:  # backward
        lgpio.gpio_write(h, wheel["in1"], 0)
        lgpio.gpio_write(h, wheel["in2"], 1)

    lgpio.tx_pwm(h, wheel["ena"], PWM_FREQ, duty_cycle)


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
    if h is not None:
        lgpio.gpiochip_close(h)
