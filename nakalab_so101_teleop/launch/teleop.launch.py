#!/usr/bin/env python3
#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, Command, PythonExpression
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory
import platform
import os


def generate_launch_description():
    ld = LaunchDescription()


    # default value
    default_robot_description_path = os.path.join(
        get_package_share_directory('nakalab_so101_description'), 'urdf', 'so101.urdf.xacro'
    )
    default_rviz_path = os.path.join(
        get_package_share_directory('nakalab_so101_teleop'), 'rviz', 'teleop.rviz'
    )


    # launch configuration
    device = LaunchConfiguration('device')
    camera_type = LaunchConfiguration('camera_type')
    use_rviz = LaunchConfiguration('use_rviz')


    # launch arguments
    declare_device = DeclareLaunchArgument(
        'device', default_value='/dev/ttyACM0',
        description='Path to SO-101 leader arm device.'
    )
    declare_camera_type = DeclareLaunchArgument(
        'camera_type', default_value='',
        description='Camera type of mouted arm camera. When value is empty, spawn the arm without camera. Support camera type value is...\n - d435'
    )
    declare_use_rviz = DeclareLaunchArgument(
        'use_rviz', default_value='true',
        description='Show robot description by Rviz2'
    )
    ld.add_action(declare_device)
    ld.add_action(declare_camera_type)
    ld.add_action(declare_use_rviz)


    # make robot description
    robot_description = Command([
        'xacro ', default_robot_description_path, ' ',
        'camera_type:=', camera_type, ' ',
    ])


    # nodes
    leader_arm_driver_node = Node(
        package='nakalab_so101',
        executable='leader_arm_driver_node',
        output='screen',
        emulate_tty=True,
        parameters=[
            {'device': device}
        ],
        condition=IfCondition(
            PythonExpression(["'", platform.system(), "'", " == 'Linux'"])
        )
    )
    leader_arm_driver_node_py = Node(
        package='nakalab_so101_py',
        executable='leader_arm_driver_node',
        output='screen',
        emulate_tty=True,
        remappings=[('leader/joint_states', 'follower/joint_commands')],
        parameters=[
            {'device': device}
        ],
        condition=IfCondition(
            PythonExpression(["'", platform.system(), "'", " != 'Linux'"])
        )
    )
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        emulate_tty=True,
        remappings=[('joint_states', 'follower/joint_states')],
        parameters=[
            {'robot_description': robot_description}
        ]
    )
    rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        output='screen',
        emulate_tty=True,
        arguments=['-d', default_rviz_path],
        condition=IfCondition(use_rviz)
    )
    ld.add_action(leader_arm_driver_node)
    ld.add_action(leader_arm_driver_node_py)
    ld.add_action(robot_state_publisher)
    ld.add_action(rviz2)


    return ld