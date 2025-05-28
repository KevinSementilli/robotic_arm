import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.actions import DeclareLaunchArgument, LogInfo, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():

    package_name = 'robotic_arm'

    sim_mode = LaunchConfiguration('sim_mode')
    cmd_mode = LaunchConfiguration('cmd_mode')
    real_time = LaunchConfiguration('real_time')

    sim_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [os.path.join(get_package_share_directory(package_name), 'launch' ,'sim.launch.py')]), 
            launch_arguments={'sim_mode': sim_mode}.items()
    )

    spawn_controllers = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [os.path.join(get_package_share_directory(package_name), 'launch' ,'spawn_controllers.launch.py')]), 
            launch_arguments={'cmd_mode': cmd_mode, 'real_time': real_time}.items()
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'sim_mode',
            default_value='foxglove',
            description='Simulation mode: rviz or foxglove'
        ),
        DeclareLaunchArgument(
            'cmd_mode',
            default_value='position',
            description='choose between position and speed control'
        ),
        DeclareLaunchArgument(
            'real_time',
            default_value='false',
            description='Enable real-time control'
        ),
        spawn_controllers,
        sim_node,
    ])