# Robot Control Software

This is a ROS2-based control system for a four-wheeled robot with joystick control.

## Package Structure

The software is organized into several ROS2 packages:

- `joystick_driver`: Handles joystick input and publishes to ROS2 topics
- `manual_motor_controller`: Controls the motors based on joystick input
- `robot_state_publisher`: Publishes robot state information and transforms
- `robot_utils`: Common utilities for motor control and other functions
- `robot_tests`: Test suite for robot functionality
- `robot_bringup`: Launch files and configurations for bringing up the robot

## Dependencies

- ROS2 (Humble or later)
- Python 3.8 or later
- Additional Python packages (see requirements.txt):
  - pygame
  - RPi.GPIO
  - pytest

## Installation

1. Clone this repository into your ROS2 workspace's src directory:
   ```bash
   cd ~/ros2_ws/src
   git clone <repository_url>
   ```

2. Install Python dependencies:
   ```bash
   pip install -r requirements.txt
   ```

3. Build the packages:
   ```bash
   cd ~/ros2_ws
   colcon build
   ```

4. Source the workspace:
   ```bash
   source ~/ros2_ws/install/setup.bash
   ```

## Usage

To start the robot control system:

```bash
ros2 launch robot_bringup robot.launch.py
```

This will start:
- Joystick driver node
- Manual motor controller node
- Robot state publisher node

## Testing

To run the test suite:

```bash
colcon test --packages-select robot_tests
```

## Node Information

### joystick_driver
- Publishes: `/joy` (sensor_msgs/Joy)
- Rate: 100Hz

### manual_motor_controller
- Subscribes: `/joy` (sensor_msgs/Joy)
- Publishes: `/cmd_vel` (geometry_msgs/Twist)

### robot_state_publisher
- Subscribes: `/cmd_vel` (geometry_msgs/Twist)
- Publishes: TF transforms (odom → base_link)

## Control Mapping

The robot uses a standard gamepad controller:
- Left stick up/down: Forward/Backward movement
- Left stick left/right: Turn left/right
- Deadzone: 10% on both axes

## License

Apache License 2.0 