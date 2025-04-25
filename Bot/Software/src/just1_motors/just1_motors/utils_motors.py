from gpiozero import Device, Motor
from gpiozero.pins.lgpio import LGPIOFactory
import time

# Set the GPIO factory to use lgpio
Device.pin_factory = LGPIOFactory()

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

# Global motor objects
front_left_motor = None
front_right_motor = None
back_left_motor = None
back_right_motor = None


def setup():
    """Initialize motor objects"""
    global front_left_motor, front_right_motor, back_left_motor, back_right_motor

    # Create motor objects with explicit PWM enable pins
    front_left_motor = Motor(
        forward=front_left_in1, backward=front_left_in2, enable=front_left_ena, pwm=True
    )
    front_right_motor = Motor(
        forward=front_right_in1,
        backward=front_right_in2,
        enable=front_right_ena,
        pwm=True,
    )
    back_left_motor = Motor(
        forward=back_left_in1, backward=back_left_in2, enable=back_left_ena, pwm=True
    )
    back_right_motor = Motor(
        forward=back_right_in1, backward=back_right_in2, enable=back_right_ena, pwm=True
    )


def stop_all():
    """Stop all motors"""
    front_left_motor.stop()
    front_right_motor.stop()
    back_left_motor.stop()
    back_right_motor.stop()


def control_wheel(wheel_name, speed):
    """
    Control a specific wheel

    Args:
        wheel_name (str): 'front_left', 'front_right', 'back_left', 'back_right'
        speed (int): -100 to 100, where:
                    - positive values indicate forward movement
                    - negative values indicate backward movement
                    - 0 means stop
    """
    # Map wheel names to their corresponding motor objects
    wheel_map = {
        "front_left": front_left_motor,
        "front_right": front_right_motor,
        "back_left": back_left_motor,
        "back_right": back_right_motor,
    }

    if wheel_name not in wheel_map:
        raise ValueError(
            f"Invalid wheel name: {wheel_name}. Must be one of {list(wheel_map.keys())}"
        )

    if not -100 <= speed <= 100:
        raise ValueError("Speed must be between -100 and 100")

    motor = wheel_map[wheel_name]
    speed = speed / 100.0  # Convert to 0-1 range

    if speed > 0:
        motor.forward(abs(speed))
    elif speed < 0:
        motor.backward(abs(speed))
    else:
        motor.stop()


def test_wheel(wheel_name, duration=2):
    """
    Test a specific wheel by running it forward and backward

    Args:
        wheel_name (str): 'front_left', 'front_right', 'back_left', 'back_right'
        duration (int): Duration in seconds for each direction
    """
    print(f"Testing {wheel_name} wheel...")

    # Forward
    control_wheel(wheel_name, 50)
    print(f"{wheel_name} spinning forward for {duration} seconds...")
    time.sleep(duration)

    # Stop
    control_wheel(wheel_name, 0)
    print("Stopping for 1 second...")
    time.sleep(1)

    # Backward
    control_wheel(wheel_name, -50)
    print(f"{wheel_name} spinning backward for {duration} seconds...")
    time.sleep(duration)

    # Stop
    control_wheel(wheel_name, 0)


def cleanup():
    """Clean up motor resources"""
    stop_all()
    # gpiozero handles cleanup automatically
