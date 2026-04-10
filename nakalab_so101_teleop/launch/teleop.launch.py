#!/usr/bin/env python3
#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory
import platform
import os


def generate_launch_description():
    ld = LaunchDescription()


    # default value
    default_rviz_path = os.path.join(
        get_package_share_directory('nakalab_so101_teleop'), 'rviz', 'teleop.rviz'
    )


    # launch configuration
    device = LaunchConfiguration('device')
    use_rviz = LaunchConfiguration('use_rviz')


    # launch arguments
    declare_device = DeclareLaunchArgument(
        'device', default_value='/dev/ttyACM0',
        description='Path to SO-101 leader arm device.'
    )
    declare_use_rviz = DeclareLaunchArgument(
        'use_rviz', default_value='true',
        description='Show robot description by Rviz2'
    )
    ld.add_action(declare_device)
    ld.add_action(declare_use_rviz)


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
    ld.add_action(rviz2)


    return ld
