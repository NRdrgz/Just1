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
    camera_node = Node(
        package="just1_camera",
        executable="camera_node",
        name="just1_camera",
        output="screen",
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
    camera_encoder_node = Node(
        package="just1_camera",
        executable="camera_encoder_node",
        name="just1_camera_encoder",
        output="screen",
        condition=IfCondition(
            PythonExpression(["'", LaunchConfiguration("mode"), "' == 'manual'"])
        ),
    )
    ld.add_action(camera_encoder_node)

    # Foxglove Bridge node. This node is installed through sudo apt install ros-jazzy-foxglove-bridge
    foxglove_bridge_node = Node(
        package="foxglove_bridge",
        executable="foxglove_bridge",
        name="foxglove_bridge",
        output="screen",
        parameters=[
            {
                "port": 8765,
                "topic_whitelist": [
                    "/wheel_speeds",
                    "/camera/image_compressed",
                    "/rtabmap/map",
                    "/rtabmap/cloud_map",
                    "/rtabmap/global_path",
                    "/rtabmap/local_path",
                    "/rtabmap/odom",
                    "/rtabmap/pose",
                    "/rtabmap/landmarks",
                    "/rtabmap/landmarks_cloud",
                ],
            }
        ],
        condition=IfCondition(
            PythonExpression(["'", LaunchConfiguration("mode"), "' == 'manual'"])
        ),
    )
    ld.add_action(foxglove_bridge_node)

    # RTAB-Map node for SLAM
    rtabmap_node = Node(
        package="rtabmap_ros",
        executable="rtabmap",
        name="rtabmap",
        output="screen",
        parameters=[
            {
                "subscribe_depth": False,
                "subscribe_scan": False,
                "frame_id": "base_link",
                "database_path": "~/rtabmap.db",
                "queue_size": 10,
                "use_sim_time": False,
                "publish_tf": True,
                "publish_tf_map": True,
                # Visual odometry parameters
                "Vis/FeatureType": "ORB",  # Using ORB features for visual odometry
                "Vis/CorType": "1",  # Use FLANN for feature matching
                "Vis/MaxFeatures": "400",  # Maximum number of features to detect
                "Vis/MinInliers": "20",  # Minimum inliers for motion estimation
                "Vis/EstimationType": "1",  # Use PnP for pose estimation
                "Vis/PnPRefineIterations": "1",  # Number of PnP refinement iterations
                "Odom/Strategy": "0",  # 0=Frame-to-Frame, 1=Frame-to-Map
                "Odom/ResetCountdown": "0",  # Don't reset odometry
                "Odom/GuessMotion": "true",  # Use previous motion as initial guess
                "Odom/VisKeyFrameThr": "0.6",  # Keyframe threshold
                "Odom/VisMinInliers": "15",  # Minimum inliers for visual odometry
                "Odom/VisMaxSize": "1500",  # Maximum image size for processing
                "Odom/VisCorNNDR": "0.6",  # Nearest neighbor distance ratio
                "Odom/VisCorType": "1",  # Use FLANN for visual odometry matching
            }
        ],
        remappings=[
            ("rgb/image", "/camera/image_raw"),
            ("rgb/camera_info", "/camera/camera_info"),
        ],
        condition=IfCondition(
            PythonExpression(["'", LaunchConfiguration("mode"), "' == 'manual'"])
        ),
    )
    ld.add_action(rtabmap_node)

    return ld
