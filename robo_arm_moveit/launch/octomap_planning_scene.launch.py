#!/usr/bin/env python3
"""
MoveIt2 Perception Pipeline Launch File - Explicit Component Launch

Launches the complete perception system for robotic arm:
- robot_state_publisher (publishes robot and camera TF from URDF)
- Intel RealSense camera with depth image
- move_group node
- RViz
- ros2_control with controllers
- Camera TF is published by robot_state_publisher from URDF (attached to claw_base)
- Depth image is fed into MoveIt's planning scene via Octomap
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():

    # Command-line arguments
    rviz_config_arg = DeclareLaunchArgument(
        "rviz_config",
        default_value="moveit.rviz",
        description="RViz configuration file",
    )

    ros2_control_hardware_type = DeclareLaunchArgument(
        "ros2_control_hardware_type",
        default_value="mock",
        description=(
            "ROS 2 control hardware mode used by robot xacro "
            "-- possible values: [mock, gazebo, real]"
        ),
    )

    # MoveIt Configuration
    moveit_config = (
        MoveItConfigsBuilder("robo_arm", package_name="robo_arm_moveit")
        .robot_description(
            file_path=os.path.join(
                get_package_share_directory("robo_arm_description"),
                "urdf",
                "robot.urdf.xacro"
            ),
            mappings={
                "HW_mode": LaunchConfiguration("ros2_control_hardware_type"),
            },
        )
        .robot_description_semantic(
            file_path=os.path.join(
                get_package_share_directory("robo_arm_moveit"),
                "config",
                "robo_arm.srdf"
            )
        )
        .robot_description_kinematics(
            file_path=os.path.join(
                get_package_share_directory("robo_arm_moveit"),
                "config",
                "kinematics.yaml"
            )
        )
        .planning_scene_monitor(
            publish_robot_description=True,
            publish_robot_description_semantic=True
        )
        .trajectory_execution(
            file_path=os.path.join(
                get_package_share_directory("robo_arm_moveit"),
                "config",
                "moveit_controllers.yaml"
            )
        )
        .planning_pipelines(
            pipelines=["ompl", "pilz_industrial_motion_planner", "stomp", "chomp"]
        )
        .sensors_3d(
            file_path=os.path.join(
                get_package_share_directory("robo_arm_moveit"),
                "config",
                "sensors_3d.yaml"
            )
        )
        .to_moveit_configs()
    )

    # RealSense camera launch
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
            'publish_tf': 'false',  # TF published by robot_state_publisher from URDF
        }.items()
    )

    # Start the actual move_group node/action server
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[moveit_config.to_dict()],
        arguments=["--ros-args", "--log-level", "info"],
    )

    # RViz
    rviz_base = LaunchConfiguration("rviz_config")
    rviz_config = PathJoinSubstitution(
        [FindPackageShare("robo_arm_moveit"), "config", rviz_base]
    )
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        output="log",
        arguments=["-d", rviz_config],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.planning_pipelines,
            moveit_config.robot_description_kinematics,
            moveit_config.joint_limits,
        ],
    )

    # Static TF - world to bottom_base
    static_tf_node = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_transform_publisher",
        output="log",
        arguments=["0.0", "0.0", "0.0", "0.0", "0.0", "0.0", "world", "bottom_base"],
    )

    # robot_state_publisher - publishes TF for robot + camera
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="both",
        parameters=[moveit_config.robot_description],
    )

    # ros2_control node
    ros2_controllers_path = os.path.join(
        get_package_share_directory("robo_arm_controller"),
        "config",
        "ros2_controllers.yaml",
    )
    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[ros2_controllers_path],
        remappings=[
            ("/controller_manager/robot_description", "/robot_description"),
        ],
        output="screen",
    )

    # Spawn joint_state_broadcaster
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager",
        ],
    )

    # Spawn robo_arm_controller
    robo_arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["robo_arm_controller", "-c", "/controller_manager"],
    )

    # Spawn claw_controller
    claw_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["claw_controller", "-c", "/controller_manager"],
    )

    return LaunchDescription(
        [
            rviz_config_arg,
            ros2_control_hardware_type,
            realsense_launch,
            rviz_node,
            static_tf_node,
            robot_state_publisher,
            move_group_node,
            ros2_control_node,
            joint_state_broadcaster_spawner,
            robo_arm_controller_spawner,
            claw_controller_spawner,
        ]
    )
