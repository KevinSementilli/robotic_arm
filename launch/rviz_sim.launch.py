import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():

    package_name = 'robotic_arm'

    # Declare use_sim_time argument
    use_sim_time = LaunchConfiguration('use_sim_time')

    # Include rsp.launch.py
    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory(package_name), 'launch', 'rsp.launch.py')),
        launch_arguments={
            'use_sim_time': use_sim_time,
        }.items()  # Pass use_sim_time as an argument
    )

    # Joint State Publisher GUI Node
    node_joint_state_publisher_gui = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    rviz_config_file = os.path.join(
        get_package_share_directory(package_name), 'config', 'config.rviz'
    )

    node_rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # Launch Description
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='False',
            description='Use sim time if true'),

        rsp,
        node_joint_state_publisher_gui,
        node_rviz
    ])
