from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_demo_launch


def launch_setup(context, *args, **kwargs):
    """Setup function that runs at launch time to resolve prefix arguments."""
    # Get the prefix values at launch time
    # For standalone: both are empty (arm joints are shoulder_pan_joint, gripper is gripper_robotiq...)
    # For Gazebo: arm_prefix=rbkairos_arm_, gripper_prefix=rbkairos_gripper_
    arm_prefix = LaunchConfiguration("arm_prefix").perform(context)
    gripper_prefix = LaunchConfiguration("gripper_prefix").perform(context)

    # Build MoveIt config with xacro support for URDF and SRDF
    # Pass prefixes to SRDF xacro
    moveit_config = (
        MoveItConfigsBuilder("ur5e_robotiq", package_name="rbkairos_moveit_config")
        .robot_description(
            file_path="config/ur5e_robotiq.urdf.xacro",
        )
        .robot_description_semantic(
            file_path="config/ur5e_robotiq.srdf.xacro",
            mappings={"arm_prefix": arm_prefix, "gripper_prefix": gripper_prefix},
        )
        .to_moveit_configs()
    )

    # Generate the standard demo launch
    demo_launch = generate_demo_launch(moveit_config)

    return list(demo_launch.entities)


def generate_launch_description():
    # Declare prefix arguments for parameterized configs
    # For standalone: empty arm_prefix, gripper_ for gripper_prefix (matching URDF)
    # For Gazebo: rbkairos_arm_ and rbkairos_gripper_
    arm_prefix_arg = DeclareLaunchArgument(
        "arm_prefix",
        default_value="",
        description="Prefix for arm joints (empty for standalone, 'rbkairos_arm_' for Gazebo)"
    )
    gripper_prefix_arg = DeclareLaunchArgument(
        "gripper_prefix",
        default_value="gripper_",
        description="Prefix for gripper joints ('gripper_' for standalone, 'rbkairos_gripper_' for Gazebo)"
    )

    return LaunchDescription([
        arm_prefix_arg,
        gripper_prefix_arg,
        OpaqueFunction(function=launch_setup),
    ])
