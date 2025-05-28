import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessStart

from launch_ros.actions import Node
from launch.actions import TimerAction

def generate_launch_description():

    package_name='demo_arm'

    cmd_mode = LaunchConfiguration('cmd_mode')
    real_time = LaunchConfiguration('real_time')

    # launch robot_state_publisher 
    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [os.path.join(get_package_share_directory(package_name), 'launch' ,'rsp.launch.py')]), 
            launch_arguments={'cmd_mode': cmd_mode, 'real_time': real_time}.items()
    )

    teleop_vel_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [os.path.join(get_package_share_directory(package_name), 'launch' ,'teleop_vel.launch.py')]), 
    )

    controller_config = os.path.join(
        get_package_share_directory(package_name), 'config', 'vel_controller.yaml'
    )

    # run the ros2_control_node to handle controller spawning and loading
    # remap topic to /robot_description
    controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[controller_config],
        remappings=[("/controller_manager/robot_description", "robot_description")],
        output="screen",
    )

    delayed_controller_manager = TimerAction(
        period=3.0, 
        actions={controller_manager}
    )

    # spawn the velocity controller
    vel_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["vel_controller"],
        output="screen",
    )

    delayed_vel_controller_spawner = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=controller_manager,
            on_start=[vel_controller_spawner],
        )
    )

    # spawn the joint broadcaster
    joint_broad_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_broad"],
        output="screen",
    )

    delayed_joint_broad_spawner = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=controller_manager,
            on_start=[joint_broad_spawner],
        )
    )

    return LaunchDescription([
        rsp,
        delayed_controller_manager,
        
        delayed_vel_controller_spawner,
        delayed_joint_broad_spawner,

        teleop_vel_node,
    ])
