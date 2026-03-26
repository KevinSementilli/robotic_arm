from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def launch_setup(context, *args, **kwargs):

    # Realsense parameters
    size = LaunchConfiguration("size").perform(context)
    fs = LaunchConfiguration("fs").perform(context)
    profile = f"{size}x{fs}"

    enable_depth = LaunchConfiguration("enable_depth")
    pointcloud_enable = LaunchConfiguration("pointcloud_enable")
    align_depth_enable = LaunchConfiguration("align_depth_enable")

    # Foxglove parameters
    ip = LaunchConfiguration("ip")
    port = LaunchConfiguration("port")

    realsense_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [FindPackageShare("realsense2_camera"), "/launch/rs_launch.py"]
        ),
        launch_arguments={
            "rgb_camera.color_profile": profile,
            "depth_module.depth_profile": profile,
            "depth_module.infra_profile": profile,
            "enable_depth": enable_depth,
            "pointcloud.enable": pointcloud_enable,
            "align_depth.enable": align_depth_enable,
        }.items(),
    )

    foxglove_node = Node(
        package="foxglove_bridge",
        executable="foxglove_bridge",
        name="foxglove_bridge",
        output="screen",
        parameters=[{
            "address": ip,
            "port": port,
        }],
    )

    return [foxglove_node, realsense_launch]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "size",
            default_value="424x240",
            description="Frame size as WIDTHxHEIGHT",
        ),
        DeclareLaunchArgument(
            "fs",
            default_value="5",
            description="Frame rate (fps)",
        ),
        DeclareLaunchArgument(
            "enable_depth",
            default_value="false",
            description="Enable depth stream",
        ),
        DeclareLaunchArgument(
            "pointcloud_enable",
            default_value="false",
            description="Enable point cloud generation",
        ),
        DeclareLaunchArgument(
            "align_depth_enable",
            default_value="false",
            description="Align depth to color",
        ),
        DeclareLaunchArgument(
            "ip",
            default_value="0.0.0.0",
            description="Foxglove bridge bind address",
        ),
        DeclareLaunchArgument(
            "port",
            default_value="8765",
            description="Foxglove bridge port",
        ),
        OpaqueFunction(function=launch_setup),
    ])
