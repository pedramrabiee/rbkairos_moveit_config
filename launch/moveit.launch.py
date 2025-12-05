#!/usr/bin/env python3
"""
MoveIt2 launch file for RB-Kairos+ mobile manipulator

This launch file starts the move_group node and optionally RViz with MoveIt plugin.
It expects the robot simulation (rbkairos_bringup) to already be running.

Author: Pedram Rabiee (prabiee@3laws.io)
"""

import os
import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Package paths
    moveit_config_pkg = get_package_share_directory('rbkairos_moveit_config')
    bringup_pkg = get_package_share_directory('rbkairos_bringup')

    # Launch arguments
    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz',
        default_value='false',
        description='Launch RViz with MoveIt plugin (set false if RViz already running)'
    )

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time'
    )

    namespace_arg = DeclareLaunchArgument(
        'namespace',
        default_value='rbkairos',
        description='Robot namespace'
    )

    # Get launch configurations
    use_rviz = LaunchConfiguration('use_rviz')
    use_sim_time = LaunchConfiguration('use_sim_time')
    namespace = LaunchConfiguration('namespace')

    # Robot description from URDF
    robot_description_content = Command([
        'xacro ',
        PathJoinSubstitution([bringup_pkg, 'urdf', 'rbkairos_plus_with_robotiq.urdf.xacro']),
        ' namespace:=rbkairos',
        ' prefix:=rbkairos_',
        ' use_gripper:=true',
        ' gripper_type:=robotiq_2f_85',
        ' gazebo_ignition:=true',
        ' ur_type:=ur5e',
    ])

    robot_description = {'robot_description': robot_description_content}

    # SRDF
    srdf_path = os.path.join(moveit_config_pkg, 'srdf', 'rbkairos.srdf')
    with open(srdf_path, 'r') as f:
        robot_description_semantic = {'robot_description_semantic': f.read()}

    # Kinematics - load as YAML content
    kinematics_yaml_path = os.path.join(moveit_config_pkg, 'config', 'kinematics.yaml')
    with open(kinematics_yaml_path, 'r') as f:
        kinematics_config = yaml.safe_load(f)

    # Joint limits - load as YAML content
    joint_limits_yaml_path = os.path.join(moveit_config_pkg, 'config', 'joint_limits.yaml')
    with open(joint_limits_yaml_path, 'r') as f:
        joint_limits_config = yaml.safe_load(f)

    # Planning configs - load OMPL and CHOMP
    ompl_planning_yaml_path = os.path.join(moveit_config_pkg, 'config', 'ompl_planning.yaml')
    with open(ompl_planning_yaml_path, 'r') as f:
        ompl_planning_config = yaml.safe_load(f)

    chomp_planning_yaml_path = os.path.join(moveit_config_pkg, 'config', 'chomp_planning.yaml')
    with open(chomp_planning_yaml_path, 'r') as f:
        chomp_planning_config = yaml.safe_load(f)

    # OMPL planning pipeline configuration
    ompl_pipeline_config = {
        'ompl': {
            'planning_plugin': 'ompl_interface/OMPLPlanner',
            'request_adapters': 'default_planner_request_adapters/AddTimeOptimalParameterization default_planner_request_adapters/ResolveConstraintFrames default_planner_request_adapters/FixWorkspaceBounds default_planner_request_adapters/FixStartStateBounds default_planner_request_adapters/FixStartStateCollision default_planner_request_adapters/FixStartStatePathConstraints',
            'start_state_max_bounds_error': 0.1,
            'simplify_solutions': True,
            **{k: v for k, v in ompl_planning_config.items() if k not in ['planning_plugin', 'request_adapters', 'start_state_max_bounds_error', 'simplify_solutions', 'use_constraints_approximations']}
        }
    }

    # CHOMP planning pipeline configuration (optimization-based, smoother paths)
    chomp_pipeline_config = {
        'chomp': chomp_planning_config
    }

    # Controllers - load as YAML content
    moveit_controllers_yaml_path = os.path.join(moveit_config_pkg, 'config', 'moveit_controllers.yaml')
    with open(moveit_controllers_yaml_path, 'r') as f:
        moveit_controllers_config = yaml.safe_load(f)

    # MoveIt configuration dictionary
    moveit_config = {
        'robot_description_kinematics': kinematics_config,
        'robot_description_planning': joint_limits_config,
    }

    # Planning pipeline configuration - enable both OMPL and CHOMP
    # Select which one to use in RViz MotionPlanning panel dropdown
    planning_pipeline_config = {
        'planning_pipelines': ['ompl', 'chomp'],
        'default_planning_pipeline': 'ompl',
    }

    # Trajectory execution configuration
    trajectory_execution = {
        'moveit_manage_controllers': True,
        'trajectory_execution.allowed_execution_duration_scaling': 1.2,
        'trajectory_execution.allowed_goal_duration_margin': 0.5,
        'trajectory_execution.allowed_start_tolerance': 0.01,
        # Ensure trajectory doesn't have duplicate timestamps at start
        'trajectory_execution.execution_duration_monitoring': False,
    }

    # Time parameterization settings for OMPL
    time_parameterization = {
        'robot_description_planning.default_velocity_scaling_factor': 0.5,
        'robot_description_planning.default_acceleration_scaling_factor': 0.5,
    }

    # Planning scene monitor configuration
    planning_scene_monitor_parameters = {
        'publish_planning_scene': True,
        'publish_geometry_updates': True,
        'publish_state_updates': True,
        'publish_transforms_updates': True,
    }

    # Move Group node
    move_group_node = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        namespace=namespace,
        output='screen',
        parameters=[
            robot_description,
            robot_description_semantic,
            moveit_config,
            planning_pipeline_config,
            ompl_pipeline_config,   # OMPL planner config
            chomp_pipeline_config,  # CHOMP planner config
            trajectory_execution,
            time_parameterization,
            planning_scene_monitor_parameters,
            moveit_controllers_config,
            {'use_sim_time': use_sim_time},
        ],
        remappings=[
            ('/joint_states', '/rbkairos/joint_states'),
        ],
    )

    # RViz with MoveIt plugin
    rviz_config_path = os.path.join(moveit_config_pkg, 'config', 'moveit.rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2_moveit',
        arguments=['-d', rviz_config_path] if os.path.exists(rviz_config_path) else [],
        parameters=[
            robot_description,
            robot_description_semantic,
            {'robot_description_kinematics': kinematics_config},
            {'use_sim_time': use_sim_time},
        ],
        condition=IfCondition(use_rviz),
    )

    return LaunchDescription([
        use_rviz_arg,
        use_sim_time_arg,
        namespace_arg,
        move_group_node,
        rviz_node,
    ])
