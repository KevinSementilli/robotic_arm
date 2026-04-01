#!/usr/bin/env python3
"""
MoveIt2 Perception Pipeline Launch File - Static Camera Mount

Launches the complete perception system for robotic arm:
- Intel RealSense camera with depth image
- Static camera TF (fixed position relative to world frame)
- MoveIt2 demo (includes move_group, RViz, controllers)
- Depth image is fed into MoveIt's planning scene via Octomap

Use this launch file when the camera is mounted separately from the robot
(e.g., on a tripod or fixed mount) rather than attached to the robot arm.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Launch arguments
    camera_frame_x = LaunchConfiguration('camera_frame_x')
    camera_frame_y = LaunchConfiguration('camera_frame_y')
    camera_frame_z = LaunchConfiguration('camera_frame_z')

    # Package paths
    moveit_config_pkg = FindPackageShare('robo_arm_moveit')

    # 1. Launch RealSense camera with depth image
    realsense_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('realsense2_camera'),
                'launch',
                'rs_launch.py'
            ])
        ),
        launch_arguments={
            'enable_depth': 'true',
            'enable_color': 'true',
            'align_depth.enable': 'true',
            'initial_reset': 'true',
            'rgb_camera.color_profile': '424x240x5',
            'depth_module.depth_profile': '424x240x5',
        }.items()
    )

    # 2. Camera TF - static transform from world to camera_link
    camera_tf_publisher = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='camera_tf_publisher',
        arguments=[
            camera_frame_x, camera_frame_y, camera_frame_z,
            '0', '0', '0',
            'world',
            'camera_link'
        ],
        output='screen'
    )

    # 3. Launch MoveIt demo (includes everything)
    demo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                moveit_config_pkg,
                'launch',
                'demo.launch.py'
            ])
        )
    )

    # Declare launch arguments
    declare_camera_x = DeclareLaunchArgument(
        'camera_frame_x',
        default_value='0.5',
        description='Camera X position relative to world frame (meters)'
    )

    declare_camera_y = DeclareLaunchArgument(
        'camera_frame_y',
        default_value='0.0',
        description='Camera Y position relative to world frame (meters)'
    )

    declare_camera_z = DeclareLaunchArgument(
        'camera_frame_z',
        default_value='0.5',
        description='Camera Z position relative to world frame (meters)'
    )

    return LaunchDescription([
        # Declare arguments
        declare_camera_x,
        declare_camera_y,
        declare_camera_z,

        # Launch camera first
        realsense_launch,
        camera_tf_publisher,

        # Delay MoveIt to allow camera to initialize
        TimerAction(
            period=3.0,
            actions=[demo_launch]
        ),
    ])
