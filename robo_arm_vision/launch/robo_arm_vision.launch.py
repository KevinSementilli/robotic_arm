from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_realsense = LaunchConfiguration("use_realsense")
    size = LaunchConfiguration("size")
    fs = LaunchConfiguration("fs")
    profile = [size, "x", fs]

    model_path = LaunchConfiguration("model_path")
    device = LaunchConfiguration("device")
    input_size = LaunchConfiguration("input_size")
    confidence_threshold = LaunchConfiguration("confidence_threshold")
    color_topic = LaunchConfiguration("color_topic")
    annotated_topic = LaunchConfiguration("annotated_topic")
    detections_topic = LaunchConfiguration("detections_topic")

    realsense_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([FindPackageShare("realsense2_camera"), "/launch/rs_launch.py"]),
        condition=IfCondition(use_realsense),
        launch_arguments={
            "rgb_camera.color_profile": profile,
            "depth_module.depth_profile": profile,
            "depth_module.infra_profile": profile,
            "enable_depth": "true",
            "pointcloud.enable": "true",
            "align_depth.enable": "true",
        }.items(),
    )

    yolo_node = Node(
        package="robo_arm_vision",
        executable="robo_arm_vision_node",
        name="robo_arm_vision_node",
        output="screen",
        parameters=[{
            "model_path": model_path,
            "device": device,
            "input_size": input_size,
            "confidence_threshold": confidence_threshold,
            "color_topic": color_topic,
            "annotated_topic": annotated_topic,
            "detections_topic": detections_topic,
        }],
    )

    return LaunchDescription([
        DeclareLaunchArgument("use_realsense", default_value="true", description="Launch realsense2_camera"),
        DeclareLaunchArgument("size", default_value="424x240", description="Frame size as WIDTHxHEIGHT"),
        DeclareLaunchArgument("fs", default_value="15", description="Frame rate (fps)"),
        DeclareLaunchArgument("model_path", default_value="yolov8n.pt", description="Ultralytics model path (.pt)"),
        DeclareLaunchArgument("device", default_value="cpu", description="Ultralytics device, e.g. cpu or 0"),
        DeclareLaunchArgument("input_size", default_value="640", description="Model inference image size"),
        DeclareLaunchArgument("confidence_threshold", default_value="0.45", description="Detection confidence threshold"),
        DeclareLaunchArgument("color_topic", default_value="/camera/camera/color/image_raw", description="Input color topic"),
        DeclareLaunchArgument("annotated_topic", default_value="/camera/color/image_annotated", description="Output annotated image topic"),
        DeclareLaunchArgument("detections_topic", default_value="/detections_2d", description="Output Detection2DArray topic"),
        realsense_launch,
        yolo_node,
    ])
