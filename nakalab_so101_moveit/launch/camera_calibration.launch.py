#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():

    ld = LaunchDescription()


    # default value
    calibrate_launch_path = os.path.join(
        get_package_share_directory('easy_handeye2'),
        'launch/calibrate.launch.py'
    )
    handeye_parameters = [{
        'name': 'nakalab_so101_d435_pose_calib',
        'calibration_type': 'eye_in_hand',
        'robot_base_frame': 'arm_base_link',

        # URDF上でカメラマウントの実際の親になっているフレーム
        'robot_effector_frame': 'gripper_link',

        # 画像による姿勢推定座標系
        'tracking_base_frame': 'so101_camera_color_optical_frame',
        'tracking_marker_frame': 'calibration_marker',
    }]


    # configures
    marker_id = LaunchConfiguration('marker_id')
    marker_size = LaunchConfiguration('marker_size')


    # launch arguments
    declare_marker_id = DeclareLaunchArgument(
        'marker_id', default_value='0'
    )
    declare_marker_size = DeclareLaunchArgument(
        'marker_size', default_value='0.05'
    )
    ld.add_action(declare_marker_id)
    ld.add_action(declare_marker_size)


    aruco_detector = Node(
        package='aruco_ros',
        executable='single',
        name='calibration_boad_detector',
        emulate_tty=True,
        parameters=[
            {
                'image_is_rectified': True,
                'marker_id': marker_id,
                'marker_size': marker_size,
                'marker_frame': 'calibration_marker',
                'camera_frame': 'so101_camera_color_optical_frame',
                'corner_refinement': 'LINES'
            }
        ],
        remappings=[
            ('/camera_info', '/so101_camera/color/camera_info'),
            ('/image', '/so101_camera/color/image_raw')
        ]
    )
    handeye_server = Node(
        package='easy_handeye2',
        executable='handeye_server',
        name='handeye_server',
        emulate_tty=True,
        parameters=handeye_parameters,
        output='screen',
    )

    handeye_gui = Node(
        package='easy_handeye2',
        executable='rqt_calibrator.py',
        name='handeye_rqt_calibrator',
        emulate_tty=True,
        parameters=handeye_parameters,
        output='screen',
    )
    ld.add_action(aruco_detector)
    ld.add_action(handeye_server)
    ld.add_action(handeye_gui)


    calibrate_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(calibrate_launch_path),
        launch_arguments={
            'name': 'nakalab_so101_d435_pose_calib',
            'freehand_robot_movement': 'false',
            'calibration_type': 'eye_in_hand',
            'robot_base_frame': 'arm_base_link',
            'robot_effector_frame': 'gripper_frame_link',
            'tracking_base_frame': 'so101_camera_color_optical_frame',
            'tracking_marker_frame': 'calibration_marker',
        }.items()
    )
    #ld.add_action(calibrate_launch)


    return ld