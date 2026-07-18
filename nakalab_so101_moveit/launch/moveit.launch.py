import os
from typing import Any

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def load_yaml(path: str) -> dict[str, Any]:
    path = os.path.abspath(path)
    with open(path, 'r', encoding='utf-8') as stream:
        value = yaml.safe_load(stream)
    if not isinstance(value, dict):
        raise RuntimeError(f'Expected YAML mapping: {path}')
    return value


def load_text(path: str) -> str:
    path = os.path.abspath(path)
    with open(path, 'r', encoding='utf-8') as stream:
        return stream.read()


def generate_launch_description() -> LaunchDescription:
    ld = LaunchDescription()


    # default value
    package_share = get_package_share_directory('nakalab_so101_moveit')
    controller_share = get_package_share_directory('nakalab_so101_controller')
    default_controlled_xacro = os.path.join(
        controller_share, 'urdf', 'nakalab_so101_controlled.urdf.xacro'
    )
    default_srdf = os.path.join(package_share, 'config', 'nakalab_so101.srdf')
    default_kinematics = os.path.join(package_share, 'config', 'kinematics.yaml')
    default_joint_limits = os.path.join(package_share, 'config', 'joint_limits.yaml')
    default_ompl = os.path.join(package_share, 'config', 'ompl_planning.yaml')
    default_moveit_controllers = os.path.join(
        package_share, 'config', 'moveit_controllers.yaml'
    )
    default_rviz_config = os.path.join(package_share, 'config', 'moveit.rviz')


    # launch configuration
    controlled_xacro = LaunchConfiguration('controlled_xacro')
    srdf = LaunchConfiguration('srdf')
    kinematics = LaunchConfiguration('kinematics')
    joint_limits = LaunchConfiguration('joint_limits')
    ompl = LaunchConfiguration('ompl')
    moveit_controllers = LaunchConfiguration('moveit_controllers')
    rviz_config = LaunchConfiguration('rviz_config')
    use_sim_hardware = LaunchConfiguration('use_sim_hardware')
    launch_rviz = LaunchConfiguration('launch_rviz')
    camera_type = LaunchConfiguration('camera_type')


    # launch arguments
    declare_controlled_xacro = DeclareLaunchArgument(
        'controlled_xacro',
        default_value=default_controlled_xacro,
        description='Path to the controlled SO-101 xacro file.',
    )
    declare_srdf = DeclareLaunchArgument(
        'srdf', default_value=default_srdf, description='Path to the SRDF file.'
    )
    declare_kinematics = DeclareLaunchArgument(
        'kinematics', default_value=default_kinematics,
        description='Path to the kinematics configuration.',
    )
    declare_joint_limits = DeclareLaunchArgument(
        'joint_limits', default_value=default_joint_limits,
        description='Path to the MoveIt joint limits configuration.',
    )
    declare_ompl = DeclareLaunchArgument(
        'ompl', default_value=default_ompl,
        description='Path to the OMPL planning configuration.',
    )
    declare_moveit_controllers = DeclareLaunchArgument(
        'moveit_controllers', default_value=default_moveit_controllers,
        description='Path to the MoveIt controller configuration.',
    )
    declare_rviz_config = DeclareLaunchArgument(
        'rviz_config', default_value=default_rviz_config,
        description='Path to the RViz configuration.',
    )
    declare_use_sim_hardware = DeclareLaunchArgument(
        'use_sim_hardware',
        default_value='false',
        description='Use ros2_control mock hardware.',
    )
    declare_launch_rviz = DeclareLaunchArgument(
        'launch_rviz',
        default_value='true',
        description='Launch RViz with MoveIt displays.',
    )
    declare_camera_type = DeclareLaunchArgument(
        'camera_type',
        default_value='',
        description=(
            'Camera type mounted on the arm. Empty disables the camera; '
            'supported value: d435.'
        ),
    )
    ld.add_action(declare_controlled_xacro)
    ld.add_action(declare_srdf)
    ld.add_action(declare_kinematics)
    ld.add_action(declare_joint_limits)
    ld.add_action(declare_ompl)
    ld.add_action(declare_moveit_controllers)
    ld.add_action(declare_rviz_config)
    ld.add_action(declare_use_sim_hardware)
    ld.add_action(declare_launch_rviz)
    ld.add_action(declare_camera_type)


    # nodes
    def setup_nodes(context):
        controlled_xacro_path = context.perform_substitution(controlled_xacro)
        srdf_path = context.perform_substitution(srdf)
        kinematics_path = context.perform_substitution(kinematics)
        joint_limits_path = context.perform_substitution(joint_limits)
        ompl_path = context.perform_substitution(ompl)
        moveit_controllers_path = context.perform_substitution(moveit_controllers)
        rviz_config_path = context.perform_substitution(rviz_config)

        robot_description = {
            'robot_description': ParameterValue(
                Command([
                    FindExecutable(name='xacro'),
                    ' ',
                    controlled_xacro_path,
                    ' use_sim_hardware:=',
                    use_sim_hardware,
                    ' camera_type:=',
                    camera_type,
                ]),
                value_type=str,
            )
        }
        robot_description_semantic = {
            'robot_description_semantic': load_text(srdf_path),
        }
        robot_description_kinematics = {
            'robot_description_kinematics': load_yaml(kinematics_path),
        }
        robot_description_planning = {
            'robot_description_planning': load_yaml(joint_limits_path),
        }
        ompl_parameters = {
            'planning_pipelines': ['ompl'],
            'default_planning_pipeline': 'ompl',
            'ompl': load_yaml(ompl_path),
        }
        moveit_controller_parameters = load_yaml(moveit_controllers_path)
        planning_scene_monitor_parameters = {
            'publish_planning_scene': True,
            'publish_geometry_updates': True,
            'publish_state_updates': True,
            'publish_transforms_updates': True,
            'publish_robot_description': True,
            'publish_robot_description_semantic': True,
            'monitor_dynamics': False,
        }

        controller_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(controller_share, 'launch', 'controller.launch.py')
            ),
            launch_arguments={
                'use_sim_hardware': use_sim_hardware,
                'camera_type': camera_type,
            }.items(),
        )
        move_group = Node(
            package='moveit_ros_move_group',
            executable='move_group',
            name='move_group',
            output='screen',
            emulate_tty=True,
            parameters=[
                robot_description,
                robot_description_semantic,
                robot_description_kinematics,
                robot_description_planning,
                ompl_parameters,
                moveit_controller_parameters,
                planning_scene_monitor_parameters,
                {
                    'allow_trajectory_execution': True,
                    'use_sim_time': False,
                },
            ],
        )
        rviz = Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            emulate_tty=True,
            arguments=['-d', rviz_config_path],
            condition=IfCondition(launch_rviz),
            parameters=[
                robot_description,
                robot_description_semantic,
                robot_description_kinematics,
                robot_description_planning,
                ompl_parameters,
                {'use_sim_time': False},
            ],
        )
        return [controller_launch, move_group, rviz]

    ld.add_action(OpaqueFunction(function=setup_nodes))


    return ld
