from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.conditions import IfCondition
from launch.actions import SetEnvironmentVariable


def generate_launch_description():
    # Create launch description
    ld = LaunchDescription()

    ################
    # Add arguments for mode selection
    ################

    mode_arg = DeclareLaunchArgument(
        "mode",
        default_value="manual",
        description="Operation mode: manual, diagnostics, autonomous",
    )
    ld.add_action(mode_arg)

    ################
    # Add arguments for diagnostics
    ################

    # To test wheel, run:
    # ros2 launch just1_bringup just1.launch.py mode:=diagnostics test_wheel:=front_left
    test_wheel_arg = DeclareLaunchArgument(
        "test_wheel",
        default_value="",
        description="Wheel to test (front_left, front_right, back_left, back_right)",
    )
    ld.add_action(test_wheel_arg)

    # To test movement, run:
    # ros2 launch just1_bringup just1.launch.py mode:=diagnostics test_movement:=forward speed:=75
    test_movement_arg = DeclareLaunchArgument(
        "test_movement",
        default_value="",
        description="Movement to test (forward, backward, left, right, etc.)",
    )
    ld.add_action(test_movement_arg)

    # To test joystick, run:
    # ros2 launch just1_bringup just1.launch.py mode:=diagnostics test_joystick:=True
    test_joystick_arg = DeclareLaunchArgument(
        "test_joystick",
        default_value="False",
        description="Whether to test joystick",
    )
    ld.add_action(test_joystick_arg)

    speed_arg = DeclareLaunchArgument(
        "speed",
        default_value="50",
        description="Movement speed (default: 50)",
    )
    ld.add_action(speed_arg)

    # Set environment variable to prevent joystick node from taking input focus
    ld.add_action(SetEnvironmentVariable("SDL_VIDEODRIVER", "dummy"))

    # Diagnostics mode nodes
    diagnostics_node = Node(
        package="just1_diagnostics",
        executable="diagnostics_node",
        name="just1_diagnostics",
        output="screen",
        parameters=[
            {
                "test_wheel": LaunchConfiguration("test_wheel"),
                "test_movement": LaunchConfiguration("test_movement"),
                "test_joystick": LaunchConfiguration("test_joystick"),
                "speed": LaunchConfiguration("speed"),
            }
        ],
        condition=IfCondition(
            PythonExpression(["'", LaunchConfiguration("mode"), "' == 'diagnostics'"])
        ),
    )
    ld.add_action(diagnostics_node)

    # Joystick driver node
    joystick_node = Node(
        package="just1_joystick_driver",
        executable="joystick_node",
        name="just1_joystick_driver",
        output="screen",
        condition=IfCondition(
            PythonExpression(
                [
                    "'",
                    LaunchConfiguration("mode"),
                    "' == 'manual' or '",
                    LaunchConfiguration("mode"),
                    "' == 'diagnostics'",
                ]
            )
        ),
    )
    ld.add_action(joystick_node)

    # Manual control nodes
    manual_controller_node = Node(
        package="just1_motors",
        executable="manual_controller_node",
        name="just1_motors",
        output="screen",
        condition=IfCondition(
            PythonExpression(["'", LaunchConfiguration("mode"), "' == 'manual'"])
        ),
    )
    ld.add_action(manual_controller_node)

    # Camera node
    # Deactivated for now as using official camera_ros
    # camera_node = Node(
    #     package="just1_camera",
    #     executable="camera_node",
    #     name="just1_camera",
    #     output="screen",
    #     condition=IfCondition(
    #         PythonExpression(["'", LaunchConfiguration("mode"), "' == 'manual'"])
    #     ),
    # )
    # ld.add_action(camera_node)

    # Camera node
    camera_node = Node(
        package="camera_ros",
        executable="camera_node",
        name="camera",
        output="screen",
        parameters=[
            {
                "width": 640,
                "height": 480,
                "orientation": "270",  # Flip the Camera
                "camera": 0,
                "format": "RGB",
            }
        ],
        condition=IfCondition(
            PythonExpression(["'", LaunchConfiguration("mode"), "' == 'manual'"])
        ),
    )
    ld.add_action(camera_node)

    # WebSocket bridge node
    # Deactivated for now as using Foxglove Bridge node instead
    # web_socket_bridge_node = Node(
    #     package="just1_camera",
    #     executable="camera_web_socket",
    #     name="just1_camera_web_socket",
    #     output="screen",
    #     condition=IfCondition(
    #         PythonExpression(["'", LaunchConfiguration("mode"), "' == 'manual'"])
    #     ),
    # )
    # ld.add_action(web_socket_bridge_node)

    # Camera encoder node
    # Deactivated for now as using official camera_ros
    # camera_encoder_node = Node(
    #     package="just1_camera",
    #     executable="camera_encoder_node",
    #     name="just1_camera_encoder",
    #     output="screen",
    #     condition=IfCondition(
    #         PythonExpression(["'", LaunchConfiguration("mode"), "' == 'manual'"])
    #     ),
    # )
    # ld.add_action(camera_encoder_node)

    # Foxglove Bridge node. This node is installed through sudo apt install ros-jazzy-foxglove-bridge
    foxglove_bridge_node = Node(
        package="foxglove_bridge",
        executable="foxglove_bridge",
        name="foxglove_bridge",
        output="screen",
        parameters=[
            {
                "port": 8765,
                "topic_whitelist": ["/wheel_speeds", "/camera/image_raw/compressed"],
            }
        ],
        condition=IfCondition(
            PythonExpression(["'", LaunchConfiguration("mode"), "' == 'manual'"])
        ),
    )
    ld.add_action(foxglove_bridge_node)

    return ld
