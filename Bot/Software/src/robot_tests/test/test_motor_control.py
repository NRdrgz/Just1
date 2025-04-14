import pytest
from robot_utils.motor_control import calculate_wheel_speeds


def test_calculate_wheel_speeds_forward():
    # Test forward movement
    speeds = calculate_wheel_speeds(-1.0, 0.0)
    assert speeds["front_left"] == 100
    assert speeds["front_right"] == 100
    assert speeds["back_left"] == 100
    assert speeds["back_right"] == 100


def test_calculate_wheel_speeds_backward():
    # Test backward movement
    speeds = calculate_wheel_speeds(1.0, 0.0)
    assert speeds["front_left"] == 100
    assert speeds["front_right"] == 100
    assert speeds["back_left"] == 100
    assert speeds["back_right"] == 100


def test_calculate_wheel_speeds_left():
    # Test left turn
    speeds = calculate_wheel_speeds(0.0, -1.0)
    assert speeds["front_left"] == 0
    assert speeds["front_right"] == 100
    assert speeds["back_left"] == 0
    assert speeds["back_right"] == 100


def test_calculate_wheel_speeds_right():
    # Test right turn
    speeds = calculate_wheel_speeds(0.0, 1.0)
    assert speeds["front_left"] == 100
    assert speeds["front_right"] == 0
    assert speeds["back_left"] == 100
    assert speeds["back_right"] == 0


def test_calculate_wheel_speeds_forward_left():
    # Test forward-left movement
    speeds = calculate_wheel_speeds(-0.5, -0.5)
    assert speeds["front_left"] < speeds["front_right"]
    assert speeds["back_left"] < speeds["back_right"]


def test_calculate_wheel_speeds_forward_right():
    # Test forward-right movement
    speeds = calculate_wheel_speeds(-0.5, 0.5)
    assert speeds["front_left"] > speeds["front_right"]
    assert speeds["back_left"] > speeds["back_right"]


def test_calculate_wheel_speeds_deadzone():
    # Test deadzone
    speeds = calculate_wheel_speeds(0.05, 0.05)
    assert speeds["front_left"] == 0
    assert speeds["front_right"] == 0
    assert speeds["back_left"] == 0
    assert speeds["back_right"] == 0
