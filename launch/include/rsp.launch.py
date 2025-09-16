import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, Command
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
import xacro


def generate_launch_description():

    # Check if we're told to use sim time
    HW_mode = LaunchConfiguration('HW_mode')
    cmd_mode = LaunchConfiguration('cmd_mode')

    # Process the URDF file
    pkg_path = os.path.join(get_package_share_directory('robotic_arm'))
    xacro_file = os.path.join(pkg_path,'urdf','robot.urdf.xacro')

    robot_description_config = Command(['xacro ', xacro_file, ' HW_mode:=', HW_mode, ' cmd_mode:=', cmd_mode])
    
    # Create a robot_state_publisher node
    params = {'robot_description': robot_description_config, 'HW_mode': HW_mode , 'cmd_mode': cmd_mode}

    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[params]
    )


    # Launch!
    return LaunchDescription([
        DeclareLaunchArgument(
            'HW_mode',
            default_value='mock',
            description='Options : mock, real, gazebo'),
        DeclareLaunchArgument(
            'cmd_mode',
            default_value='position',
            description='choose between position and speed control'),

        node_robot_state_publisher
    ])