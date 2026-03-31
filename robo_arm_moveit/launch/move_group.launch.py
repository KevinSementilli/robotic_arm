import os
from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_move_group_launch
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    moveit_config = (
        MoveItConfigsBuilder("robo_arm", package_name="robo_arm_moveit")
        .robot_description_semantic(
            file_path=os.path.join(
                get_package_share_directory("robo_arm_moveit"),
                "config",
                "robo_arm.srdf"
            )
        )
        .trajectory_execution(
            file_path=os.path.join(
                get_package_share_directory("robo_arm_controller"),
                "config",
                "ros2_controllers.yaml"
            )
        )
        .sensors_3d(
            file_path=os.path.join(
                get_package_share_directory("robo_arm_moveit"),
                "config",
                "sensors_3d.yaml"
            )
        )
        .to_moveit_configs()
    )
    return generate_move_group_launch(moveit_config)
