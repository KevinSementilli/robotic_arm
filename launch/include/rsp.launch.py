import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, Command
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
import xacro


def generate_launch_description():

    # Check if we're told to use sim time
    cmd_mode = LaunchConfiguration('cmd_mode')
    real_time = LaunchConfiguration('real_time')

    # Process the URDF file
    pkg_path = os.path.join(get_package_share_directory('robotic_arm'))
    xacro_file = os.path.join(pkg_path,'urdf','robot.urdf.xacro')

    robot_description_config = Command(['xacro ', xacro_file, ' cmd_mode:=', cmd_mode, ' real_time:=', real_time])
    
    # Create a robot_state_publisher node
    params = {'robot_description': robot_description_config, 'cmd_mode': cmd_mode, 'real_time': real_time}

    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[params]
    )


    # Launch!
    return LaunchDescription([
        DeclareLaunchArgument(
            'cmd_mode',
            default_value='position',
            description='choose between position and speed control'),
        
        DeclareLaunchArgument(
            'real_time',
            default_value='false',
            description='use real time if true'),

        node_robot_state_publisher
    ])