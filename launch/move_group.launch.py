from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_move_group_launch


def generate_launch_description():
    moveit_config = (
        MoveItConfigsBuilder("ur5e_robotiq", package_name="rbkairos_moveit_config")
        .robot_description(file_path="config/ur5e_robotiq.urdf.xacro")
        .robot_description_semantic(file_path="config/ur5e_robotiq.srdf.xacro")
        .to_moveit_configs()
    )
    return generate_move_group_launch(moveit_config)
