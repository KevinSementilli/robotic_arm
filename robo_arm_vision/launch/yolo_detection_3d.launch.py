"""
YOLO 3D Detection Launch File

Launches the YOLO 3D detection node with optional robot TF setup.
When use_robot_tf:=true, includes robot_state_publisher and ros2_control
for standalone operation. Otherwise, assumes TF is provided externally.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory

import os

# Virtual environment site-packages path
VENV_SITE_PACKAGES = os.path.expanduser(
    "~/projects/robo_arm_ws/.venv/lib/python3.12/site-packages"
)


def generate_launch_description():
    # Launch arguments
    use_realsense = LaunchConfiguration("use_realsense")
    use_robot_tf = LaunchConfiguration("use_robot_tf")

    model_path = LaunchConfiguration("model_path")
    device = LaunchConfiguration("device")
    input_size = LaunchConfiguration("input_size")
    confidence_threshold = LaunchConfiguration("confidence_threshold")
    color_topic = LaunchConfiguration("color_topic")
    depth_topic = LaunchConfiguration("depth_topic")
    camera_info_topic = LaunchConfiguration("camera_info_topic")
    annotated_topic = LaunchConfiguration("annotated_topic")
    detections_topic = LaunchConfiguration("detections_topic")
    target_frame = LaunchConfiguration("target_frame")
    min_depth = LaunchConfiguration("min_depth")
    max_depth = LaunchConfiguration("max_depth")

    # RealSense camera launch
    # IMPORTANT: publish_tf must be false - TF comes from URDF via robot_state_publisher
    realsense_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("realsense2_camera"),
                "launch",
                "rs_launch.py"
            ])
        ),
        condition=IfCondition(use_realsense),
        launch_arguments={
            "rgb_camera.color_profile": "424x240x5",
            "depth_module.depth_profile": "424x240x5",
            "enable_depth": "true",
            "enable_color": "true",
            "align_depth.enable": "true",
            "pointcloud.enable": "false",
            "publish_tf": "false",  # TF published by robot_state_publisher from URDF
        }.items(),
    )

    # Robot description (processed with xacro)
    robot_description_path = os.path.join(
        get_package_share_directory("robo_arm_description"),
        "urdf",
        "robot.urdf.xacro",
    )
    robot_description = Command(["xacro ", robot_description_path, " HW_mode:=mock"])

    # robot_state_publisher - publishes TF for robot + camera (camera attached to claw_base in URDF)
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="both",
        condition=IfCondition(use_robot_tf),
        parameters=[{"robot_description": robot_description}],
    )

    # Static TF: world -> bottom_base (connects robot to world frame)
    static_tf_node = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_transform_publisher",
        output="log",
        condition=IfCondition(use_robot_tf),
        arguments=["0.0", "0.0", "0.0", "0.0", "0.0", "0.0", "world", "bottom_base"],
    )

    # ros2_control node (needed for joint states -> robot_state_publisher -> TF)
    ros2_controllers_path = os.path.join(
        get_package_share_directory("robo_arm_controller"),
        "config",
        "ros2_controllers.yaml",
    )
    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        condition=IfCondition(use_robot_tf),
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
        condition=IfCondition(use_robot_tf),
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager",
        ],
    )

    # Build PYTHONPATH with venv site-packages prepended
    existing_pythonpath = os.environ.get("PYTHONPATH", "")
    venv_pythonpath = f"{VENV_SITE_PACKAGES}:{existing_pythonpath}" if existing_pythonpath else VENV_SITE_PACKAGES

    yolo_3d_node = Node(
        package="robo_arm_vision",
        executable="yolo_detection_3d_node",
        name="yolo_detection_3d_node",
        output="screen",
        parameters=[{
            "model_path": model_path,
            "device": device,
            "input_size": input_size,
            "confidence_threshold": confidence_threshold,
            "color_topic": color_topic,
            "depth_topic": depth_topic,
            "camera_info_topic": camera_info_topic,
            "annotated_topic": annotated_topic,
            "detections_topic": detections_topic,
            "target_frame": target_frame,
            "min_depth": min_depth,
            "max_depth": max_depth,
        }],
        additional_env={"PYTHONPATH": venv_pythonpath},
    )

    return LaunchDescription([
        # Launch arguments
        DeclareLaunchArgument("use_realsense", default_value="true", description="Launch realsense2_camera"),
        DeclareLaunchArgument("use_robot_tf", default_value="true", description="Launch robot TF (robot_state_publisher, ros2_control)"),
        DeclareLaunchArgument("model_path", default_value="yolov8n.pt", description="Ultralytics model path (.pt)"),
        DeclareLaunchArgument("device", default_value="cpu", description="Ultralytics device, e.g. cpu or 0"),
        DeclareLaunchArgument("input_size", default_value="640", description="Model inference image size"),
        DeclareLaunchArgument("confidence_threshold", default_value="0.45", description="Detection confidence threshold"),
        DeclareLaunchArgument("color_topic", default_value="/camera/camera/color/image_raw", description="Input color topic"),
        DeclareLaunchArgument("depth_topic", default_value="/camera/camera/aligned_depth_to_color/image_raw", description="Input aligned depth topic"),
        DeclareLaunchArgument("camera_info_topic", default_value="/camera/camera/color/camera_info", description="Camera info topic"),
        DeclareLaunchArgument("annotated_topic", default_value="/camera/color/image_annotated", description="Output annotated image topic"),
        DeclareLaunchArgument("detections_topic", default_value="/detections_3d", description="Output Detection3DArray topic"),
        DeclareLaunchArgument("target_frame", default_value="world", description="Target TF frame for detections"),
        DeclareLaunchArgument("min_depth", default_value="0.1", description="Minimum valid depth in meters"),
        DeclareLaunchArgument("max_depth", default_value="20.0", description="Maximum valid depth in meters"),
        # Nodes
        realsense_launch,
        static_tf_node,
        robot_state_publisher,
        ros2_control_node,
        joint_state_broadcaster_spawner,
        yolo_3d_node,
    ])
