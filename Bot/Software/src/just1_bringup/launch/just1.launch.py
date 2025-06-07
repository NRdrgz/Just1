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
            }
        ],
        condition=IfCondition(
            PythonExpression(["'", LaunchConfiguration("mode"), "' == 'manual'"])
        ),
    )
    ld.add_action(foxglove_bridge_node)

    # IMU node
    imu_node = Node(
        package="just1_imu",
        executable="imu_node",
        name="just1_imu",
        output="screen",
        condition=IfCondition(
            PythonExpression(["'", LaunchConfiguration("mode"), "' == 'manual'"])
        ),
    )
    ld.add_action(imu_node)

    # Filter IMU node
    # Published to /imu/data
    filter_imu_node = Node(
        package="imu_filter_madgwick",
        executable="imu_filter_madgwick_node",
        name="imu_filter_madgwick",
        output="screen",
        parameters=[
            {"use_mag": False},
            {"stateless": True},
            {"publish_tf": False},  # We let ICP deal with the odom->base_link tf
            {"gain": 0.3},
            {"zeta": 0.02},
        ],
    )
    ld.add_action(filter_imu_node)

    # Scan IMU Sync node
    scan_imu_sync_node = Node(
        package="just1_imu",
        executable="scan_imu_sync",
        name="scan_imu_sync",
        output="screen",
    )
    ld.add_action(scan_imu_sync_node)

    # Static transform publisher to define the position and orientation of the IMU relative to the robot's base
    base_link_to_imu_tf_node = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="base_link_to_base_imu",
        # Arguments: [x, y, z, roll, pitch, yaw, parent_frame, child_frame]
        # x: 0.07 meters (IMU mounted 7cm in front of base)
        # y: 0 meters (no offset in y direction)
        # z: meters (no offset in y direction)
        # yaw: 0 radians (no yaw rotation)
        # pitch: 0 radians (no pitch rotation)
        # roll: 0 radians (no roll rotation)
        # parent_frame: base_link (robot's base frame)
        # child_frame: imu_link (IMU's frame)
        arguments=["0.07", "0", "0", "0", "0", "0", "base_link", "imu_link"],
    )
    ld.add_action(base_link_to_imu_tf_node)

    ## Source from Lidar Driver https://wiki.youyeetoo.com/en/Lidar/D300
    # LiDAR node configuration for LD19 model
    ldlidar_node = Node(
        package="ldlidar_stl_ros2",
        executable="ldlidar_stl_ros2_node",
        name="LD19",
        output="screen",
        parameters=[
            # Model name of the LiDAR sensor
            {"product_name": "LDLiDAR_LD19"},
            # ROS2 topic name for publishing laser scan data
            {"topic_name": "scan"},
            # Frame ID for the LiDAR sensor in the TF tree
            {"frame_id": "base_laser"},
            # Serial port where the LiDAR is connected
            {"port_name": "/dev/ttyUSB0"},
            # Baud rate for serial communication with the LiDAR
            {"port_baudrate": 230400},
            # Direction of laser scan (True = clockwise, False = counter-clockwise)
            {"laser_scan_dir": True},
            # Enable/disable angle cropping functionality
            {"enable_angle_crop_func": False},
            # Minimum angle for cropping (in degrees)
            {"angle_crop_min": 135.0},
            # Maximum angle for cropping (in degrees)
            {"angle_crop_max": 225.0},
        ],
    )

    # Static transform publisher to define the position and orientation of the LiDAR relative to the robot's base
    base_link_to_laser_tf_node = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="base_link_to_base_laser_ld19",
        # Arguments: [x, y, z, roll, pitch, yaw, parent_frame, child_frame]
        # x: 0 meters (no offset in x direction)
        # y: 0 meters (no offset in y direction)
        # z: 0.12 meters (LiDAR mounted 12cm above base)
        # yaw: 1.5708 radians (90 degrees rotation around z-axis)
        # pitch: 0 radians (no pitch rotation)
        # roll: 1.5708 radians (no pitch rotation)
        # parent_frame: base_link (robot's base frame)
        # child_frame: base_laser (LiDAR's frame)
        arguments=["0", "0", "0.12", "1.5708", "0", "0", "base_link", "base_laser"],
    )

    ld.add_action(ldlidar_node)
    ld.add_action(base_link_to_laser_tf_node)

    # ICP Odometry Node
    icp_odom_node = Node(
        package="rtabmap_odom",
        executable="icp_odometry",
        name="icp_odometry",
        output="screen",
        parameters=[
            {"frame_id": "base_link"},  # Robot base frame
            {"odom_frame_id": "odom"},  # Odometry frame to publish
            {"scan_topic": "/scan"},  # 2D Lidar topic
            {"publish_tf": True},  # Publishes TF from odom -> base_link
            {"queue_size": 10},
        ],
        remappings=[
            ("/imu", "/imu/synced"),
            ("/scan", "/scan/synced"),
        ],
        arguments=["--ros-args", "--log-level", "warn"],
    )

    ld.add_action(icp_odom_node)

    # Rtabmap SLAM Node
    rtabmap_slam_node = Node(
        package="rtabmap_slam",
        executable="rtabmap",
        name="rtabmap_slam",
        output="screen",
        parameters=[
            {"frame_id": "base_link"},
            {"odom_frame_id": "odom"},
            {"subscribe_depth": False},
            {"subscribe_rgb": False},
            {"subscribe_scan": True},
            {"subscribe_imu": True},
            {"wait_for_transform": 0.2},
            {"approx_sync": True},
        ],
        remappings=[
            ("/imu", "/imu/synced"),
            ("/scan", "/scan/synced"),
        ],
        arguments=["--ros-args", "--log-level", "warn"],
    )

    ld.add_action(rtabmap_slam_node)

    return ld
