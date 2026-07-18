import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    ld = LaunchDescription()


    # default value
    package_share = get_package_share_directory('nakalab_so101_controller')
    default_controlled_xacro = os.path.join(
        package_share, 'urdf', 'nakalab_so101_controlled.urdf.xacro'
    )
    default_controllers_yaml = os.path.join(
        package_share, 'config', 'ros2_controllers.yaml'
    )


    # launch configuration
    controlled_xacro = LaunchConfiguration('controlled_xacro')
    controllers_yaml = LaunchConfiguration('controllers_yaml')
    use_sim_hardware = LaunchConfiguration('use_sim_hardware')
    camera_type = LaunchConfiguration('camera_type')


    # launch arguments
    declare_controlled_xacro = DeclareLaunchArgument(
        'controlled_xacro',
        default_value=default_controlled_xacro,
        description='Path to the controlled SO-101 xacro file.',
    )
    declare_controllers_yaml = DeclareLaunchArgument(
        'controllers_yaml',
        default_value=default_controllers_yaml,
        description='Path to the ros2_control controller configuration.',
    )
    declare_use_sim_hardware = DeclareLaunchArgument(
        'use_sim_hardware',
        default_value='false',
        description=(
            'Use mock_components/GenericSystem instead of the real '
            'JointState topic interface.'
        ),
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
    ld.add_action(declare_controllers_yaml)
    ld.add_action(declare_use_sim_hardware)
    ld.add_action(declare_camera_type)


    # derived robot description
    robot_description = ParameterValue(
        Command([
            'xacro ',
            controlled_xacro,
            ' use_sim_hardware:=',
            use_sim_hardware,
            ' camera_type:=',
            camera_type,
        ]),
        value_type=str,
    )


    # nodes
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        emulate_tty=True,
        sigterm_timeout='2.0',
        sigkill_timeout='2.0',
        parameters=[{'robot_description': robot_description}],
    )
    ros2_control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        output='screen',
        emulate_tty=True,
        sigterm_timeout='2.0',
        sigkill_timeout='2.0',
        parameters=[
            {'robot_description': robot_description},
            controllers_yaml,
        ],
        remappings=[
            ('/robot_joint_states', '/follower/joint_states'),
            ('/robot_joint_commands', '/follower/joint_commands'),
        ],
    )

    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'joint_state_broadcaster',
            '--controller-manager',
            '/controller_manager',
        ],
        output='screen',
        emulate_tty=True,
        sigterm_timeout='2.0',
        sigkill_timeout='2.0',
    )
    follower_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'so101_follower_controller',
            '--controller-manager',
            '/controller_manager',
        ],
        output='screen',
        emulate_tty=True,
        sigterm_timeout='2.0',
        sigkill_timeout='2.0',
    )
    gripper_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'so101_gripper_controller',
            '--controller-manager',
            '/controller_manager',
        ],
        output='screen',
        emulate_tty=True,
        sigterm_timeout='2.0',
        sigkill_timeout='2.0',
    )
    start_trajectory_controllers = RegisterEventHandler(
        OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[
                follower_controller_spawner,
                gripper_controller_spawner,
            ],
        )
    )
    ld.add_action(robot_state_publisher)
    ld.add_action(ros2_control_node)
    ld.add_action(joint_state_broadcaster_spawner)
    ld.add_action(start_trajectory_controllers)

    return ld
