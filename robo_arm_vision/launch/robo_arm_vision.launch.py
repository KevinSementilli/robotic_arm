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

    hef = LaunchConfiguration("hef")
    stream_fps = LaunchConfiguration("stream_fps")
    host = LaunchConfiguration("host")
    port = LaunchConfiguration("port")
    confidence_threshold = LaunchConfiguration("confidence_threshold")
    depth_unit_scale = LaunchConfiguration("depth_unit_scale")
    color_topic = LaunchConfiguration("color_topic")
    depth_topic = LaunchConfiguration("depth_topic")
    annotated_topic = LaunchConfiguration("annotated_topic")
    detections_topic = LaunchConfiguration("detections_topic")
    enriched_topic = LaunchConfiguration("enriched_topic")

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
            "hef": hef,
            "stream_fps": stream_fps,
            "host": host,
            "port": port,
            "confidence_threshold": confidence_threshold,
            "depth_unit_scale": depth_unit_scale,
            "color_topic": color_topic,
            "depth_topic": depth_topic,
            "annotated_topic": annotated_topic,
            "detections_topic": detections_topic,
            "enriched_topic": enriched_topic,
        }],
    )

    return LaunchDescription([
        DeclareLaunchArgument("use_realsense", default_value="true", description="Launch realsense2_camera"),
        DeclareLaunchArgument("size", default_value="424x240", description="Frame size as WIDTHxHEIGHT"),
        DeclareLaunchArgument("fs", default_value="15", description="Frame rate (fps)"),
        DeclareLaunchArgument("hef", default_value="yolov8n.hef", description="Hailo HEF file path"),
        DeclareLaunchArgument("stream_fps", default_value="12", description="Web stream FPS cap"),
        DeclareLaunchArgument("host", default_value="0.0.0.0", description="Web host binding"),
        DeclareLaunchArgument("port", default_value="5000", description="Web page and WebSocket port"),
        DeclareLaunchArgument("confidence_threshold", default_value="0.45", description="Detection confidence threshold"),
        DeclareLaunchArgument("depth_unit_scale", default_value="0.001", description="Depth raw unit scale to meters"),
        DeclareLaunchArgument("color_topic", default_value="/camera/camera/color/image_raw", description="Input color topic"),
        DeclareLaunchArgument("depth_topic", default_value="/camera/camera/aligned_depth_to_color/image_raw", description="Input depth topic"),
        DeclareLaunchArgument("annotated_topic", default_value="/camera/color/image_annotated", description="Output annotated image topic"),
        DeclareLaunchArgument("detections_topic", default_value="/detections_2d", description="Output Detection2DArray topic"),
        DeclareLaunchArgument("enriched_topic", default_value="/detections_enriched", description="Output JSON detection+distance topic"),
        realsense_launch,
        yolo_node,
    ])
