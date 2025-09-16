import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():

    package_name='robotic_arm'

    # Launch configuration variables
    HW_mode = LaunchConfiguration('HW_mode')
    cmd_mode = LaunchConfiguration('cmd_mode')
    use_sim_time = LaunchConfiguration('use_sim_time')

    # launch robot_state_publisher 
    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [os.path.join(get_package_share_directory(package_name), 'launch/include' ,'rsp.launch.py')]), 
            launch_arguments={
                'HW_mode' : HW_mode,
                'cmd_mode': cmd_mode, 
                'use_sim_time': use_sim_time
            }.items()
    )

    # Configure the Gazebo environment
    world_path = os.path.join(get_package_share_directory(package_name), 'worlds', 'empty.world')
    if not os.path.exists(world_path):
        world_path = ''  # Use default empty world

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')),
        launch_arguments={
            # 'verbose': 'true',
            'world': world_path,
            'pause': 'false',
            'use_sim_time': use_sim_time
        }.items()
    )

    # Spawn the robot
    spawn_entity = Node(
        package='gazebo_ros', 
        executable='spawn_entity.py',
        arguments=[
            '-topic', '/robot_description',
            '-entity', 'robotic_arm'],
        output='screen'
    )

    # Add controller manager and spawners
    controller_config = os.path.join(
        get_package_share_directory(package_name), 'config', 'trajectory_controller.yaml'
    )

    controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[controller_config],
        remappings=[("/controller_manager/robot_description", "robot_description")],
        output="screen",
    )

    delayed_controller_manager = TimerAction(
        period=3.0,
        actions=[controller_manager]
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )

    delayed_joint_state_broadcaster_spawner = TimerAction(
        period=4.0,
        actions=[joint_state_broadcaster_spawner],
    )

    traj_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["trajectory_controller", "--controller-manager", "/controller_manager"],
    )

    delayed_traj_controller_spawner = TimerAction(
        period=5.0,
        actions=[traj_controller_spawner],
    )

    return LaunchDescription([ 
        DeclareLaunchArgument(
            'HW_mode',
            default_value='gazebo',
            description='Options : mock, real, gazebo'),
        DeclareLaunchArgument(
            'cmd_mode',
            default_value='position',
            description='choose between position and speed control'),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock if true'),

        rsp,
        gazebo,
        spawn_entity,
        delayed_controller_manager,
        delayed_joint_state_broadcaster_spawner,
        delayed_traj_controller_spawner
    ])