#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    ld = LaunchDescription()


    # default value
    default_robot_description_path = os.path.join(
        get_package_share_directory('nakalab_so101_description'), 'urdf', 'so101.urdf.xacro'
    )
    default_rviz_path = os.path.join(
        get_package_share_directory('nakalab_so101_description'), 'rviz', 'robot_description.rviz'
    )


    # launch configuration
    camera_type = LaunchConfiguration('camera_type')
    use_rviz = LaunchConfiguration('use_rviz')
    use_jspg = LaunchConfiguration('use_jspg')


    # launch arguments
    declare_camera_type = DeclareLaunchArgument(
        'camera_type', default_value='',
        description='Camera type of mouted arm camera. When value is empty, spawn the arm without camera. Support camera type value is...\n - d435'
    )
    declare_use_rviz = DeclareLaunchArgument(
        'use_rviz', default_value='false',
        description='Show robot description by Rviz2'
    )
    declare_use_jspg = DeclareLaunchArgument(
        'use_jspg', default_value='false',
        description='Use Joint State Publisher GUI.'
    )
    ld.add_action(declare_camera_type)
    ld.add_action(declare_use_rviz)
    ld.add_action(declare_use_jspg)


    # make robot description
    robot_description = Command([
        'xacro ', default_robot_description_path, ' ',
        'camera_type:=', camera_type, ' ',
    ])


    # nodes
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        emulate_tty=True,
        parameters=[
            {'robot_description': robot_description}
        ]
    )
    joint_state_publisher_gui = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        output='screen',
        emulate_tty=True,
        condition=IfCondition(use_jspg)
    )
    rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        output='screen',
        emulate_tty=True,
        arguments=['-d', default_rviz_path],
        condition=IfCondition(use_rviz)
    )
    ld.add_action(robot_state_publisher)
    ld.add_action(joint_state_publisher_gui)
    ld.add_action(rviz2)


    return ld