#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import EnvironmentVariable, LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    ld = LaunchDescription()


    # default variables
    default_d435_params_file = os.path.join(
        get_package_share_directory('nakalab_so101'),
        'params', 'd435.yaml'
    )
    default_calibration_root = os.path.join(
        get_package_share_directory('nakalab_so101_description'),
        'calibration'
    )


    # launch configurations
    device = LaunchConfiguration('device')
    camera_type = LaunchConfiguration('camera_type')
    calibration_file = LaunchConfiguration('calibration_file')


    # launch arguments
    declare_device = DeclareLaunchArgument(
        'device',
        default_value=EnvironmentVariable('FOLLOWER_DEVICE', default_value='/dev/ttyACM0'),
        description='Path to SO-101 follower arm device.'
    )
    declare_camera_type = DeclareLaunchArgument(
        'camera_type', default_value='',
        description=(
            'Camera type of mounted arm camera. When value is empty, spawn the arm '
            'without camera. Supported values: d435'
        )
    )
    declare_calibration_file = DeclareLaunchArgument(
        'calibration_file', default_value='',
        description='Optional SO-101 follower calibration JSON path.'
    )
    ld.add_action(declare_device)
    ld.add_action(declare_camera_type)
    ld.add_action(declare_calibration_file)


    # nodes
    follower_arm_driver_node = Node(
        package='nakalab_so101',
        executable='follower_arm_driver_node',
        output='screen',
        emulate_tty=True,
        parameters=[{
            'device': device,
            'calibration_file': calibration_file,
            'calibration_root': default_calibration_root,
        }]
    )
    estop_gui_node = Node(
        package='nakalab_so101_py',
        executable='estop_gui',
        output='screen',
        emulate_tty=True,
    )
    realsense_d435 = Node(
        package='realsense2_camera',
        executable='realsense2_camera_node',
        name='d435',
        namespace='so101_camera',
        output='screen',
        emulate_tty=True,
        parameters=[default_d435_params_file],
        condition=IfCondition(
            PythonExpression(["'", camera_type, "'", " == 'd435'"])
        )
    )
    ld.add_action(follower_arm_driver_node)
    ld.add_action(estop_gui_node)
    ld.add_action(realsense_d435)


    return ld
