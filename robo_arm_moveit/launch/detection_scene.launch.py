import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    # Launch arguments
    use_rviz = LaunchConfiguration("use_rviz")
    use_vision = LaunchConfiguration("use_vision")
    detections_topic = LaunchConfiguration("detections_topic")
    planning_frame = LaunchConfiguration("planning_frame")
    stale_timeout = LaunchConfiguration("stale_timeout")
    update_rate = LaunchConfiguration("update_rate")

    robot_description_path = os.path.join(
        get_package_share_directory("robo_arm_description"),
        "urdf",
        "robot.urdf.xacro",
    )
    moveit_config = (
        MoveItConfigsBuilder("robo_arm", package_name="robo_arm_moveit")
        .planning_pipelines(pipelines=["ompl"])
        .robot_description(file_path=robot_description_path)
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .sensors_3d(file_path="config/sensors_3d_empty.yaml")  # Use empty sensors config
        .to_moveit_configs()
    )

    # Move group node (without octomap - we didn't call .sensors_3d() so no octomap)
    run_move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[moveit_config.to_dict()],
    )

    # RViz
    rviz_config_file = (
        get_package_share_directory("robo_arm_moveit") + "/config/moveit.rviz"
    )
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.planning_pipelines,
        ],
        condition=IfCondition(use_rviz),
    )

    # Static TF: world -> bottom_base
    static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_transform_publisher",
        output="log",
        arguments=["0.0", "0.0", "0.0", "0.0", "0.0", "0.0", "world", "bottom_base"],
    )

    # Robot state publisher
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="both",
        parameters=[moveit_config.robot_description],
    )

    # ros2_control
    ros2_controllers_path = os.path.join(
        get_package_share_directory("robo_arm_controller"),
        "config",
        "ros2_controllers.yaml",
    )
    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[moveit_config.robot_description, ros2_controllers_path],
        output="both",
    )

    # Load controllers
    load_controllers = []
    for controller in [
        "robo_arm_controller",
        "claw_controller",
        "joint_state_broadcaster",
    ]:
        load_controllers += [
            ExecuteProcess(
                cmd=["ros2 run controller_manager spawner {}".format(controller)],
                shell=True,
                output="screen",
            )
        ]

    # Detection scene builder node
    detection_scene_builder = Node(
        package="robo_arm_moveit",
        executable="detection_scene_builder",
        name="detection_scene_builder",
        output="screen",
        parameters=[{
            "detections_topic": detections_topic,
            "planning_frame": planning_frame,
            "stale_timeout": stale_timeout,
            "update_rate": update_rate,
        }],
    )

    # Optionally include the 3D detection launch
    # use_robot_tf:=false because this launch already provides TF
    yolo_detection_3d_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare("robo_arm_vision"),
            "/launch/yolo_detection_3d.launch.py"
        ]),
        condition=IfCondition(use_vision),
        launch_arguments={
            "use_realsense": "true",
            "use_robot_tf": "false",  # TF provided by this launch file
        }.items(),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_rviz", default_value="true", description="Launch RViz"),
            DeclareLaunchArgument("use_vision", default_value="false", description="Launch YOLO 3D detection"),
            DeclareLaunchArgument("detections_topic", default_value="/detections_3d", description="Detection3DArray topic"),
            DeclareLaunchArgument("planning_frame", default_value="world", description="MoveIt planning frame"),
            DeclareLaunchArgument("stale_timeout", default_value="2.0", description="Seconds before removing stale objects"),
            DeclareLaunchArgument("update_rate", default_value="2.0", description="Planning scene update rate (Hz)"),
            rviz_node,
            static_tf,
            robot_state_publisher,
            run_move_group_node,
            ros2_control_node,
            detection_scene_builder,
            yolo_detection_3d_launch,
        ]
        + load_controllers
    )
