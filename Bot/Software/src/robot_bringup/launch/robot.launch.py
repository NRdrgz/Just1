from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition


def generate_launch_description():
    # Create launch description
    ld = LaunchDescription()

    # Add argument for mode selection
    mode_arg = DeclareLaunchArgument(
        "mode", default_value="manual", description="Operation mode: manual or test"
    )
    ld.add_action(mode_arg)

    # Joystick driver node (used in both modes)
    joystick_node = Node(
        package="joystick_driver",
        executable="joystick_node",
        name="joystick_driver",
        output="screen",
    )
    ld.add_action(joystick_node)

    # Manual control nodes
    manual_controller_node = Node(
        package="manual_motor_controller",
        executable="controller_node",
        name="manual_motor_controller",
        output="screen",
        condition=IfCondition(LaunchConfiguration("mode").equals("manual")),
    )
    ld.add_action(manual_controller_node)

    state_publisher_node = Node(
        package="robot_state_publisher",
        executable="state_publisher_node",
        name="robot_state_publisher",
        output="screen",
        condition=IfCondition(LaunchConfiguration("mode").equals("manual")),
    )
    ld.add_action(state_publisher_node)

    # Test mode nodes
    test_node = Node(
        package="robot_tests",
        executable="test_node",
        name="robot_tests",
        output="screen",
        condition=IfCondition(LaunchConfiguration("mode").equals("test")),
    )
    ld.add_action(test_node)

    return ld
