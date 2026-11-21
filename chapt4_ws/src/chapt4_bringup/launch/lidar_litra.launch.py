#!/usr/bin/env python3
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    bringup_share = get_package_share_directory('chapt4_bringup')
    default_params = os.path.join(
        bringup_share, 'config', 'ltme_02a.yaml')

    params_arg = DeclareLaunchArgument(
        'litra_params_file', default_value=default_params,
        description='LitraTech ltme_node parameter file')
    model_arg = DeclareLaunchArgument(
        'litra_model', default_value='LTME-02A',
        description='Litra model: LTME-02A, LT-R1, LT-R2, LT-I1 or LT-I2')
    address_arg = DeclareLaunchArgument(
        'litra_address', default_value='192.168.10.160',
        description='Litra device IPv4 address')
    frame_arg = DeclareLaunchArgument(
        'litra_frame', default_value='litra_link',
        description='LaserScan frame_id of the Litra lidar')
    x_arg = DeclareLaunchArgument(
        'litra_x', default_value='0.08',
        description='Litra optical center x in base_link (m)')
    y_arg = DeclareLaunchArgument(
        'litra_y', default_value='0.0',
        description='Litra optical center y in base_link (m)')
    z_arg = DeclareLaunchArgument(
        'litra_z', default_value='0.21',
        description='Litra optical center z in base_link (m)')
    yaw_arg = DeclareLaunchArgument(
        'litra_yaw', default_value='0.0',
        description='Litra yaw in base_link (rad)')
    pitch_arg = DeclareLaunchArgument(
        'litra_pitch', default_value='0.0',
        description='Litra pitch in base_link (rad)')
    roll_arg = DeclareLaunchArgument(
        'litra_roll', default_value='0.0',
        description='Litra roll in base_link (rad)')

    lidar = Node(
        package='ltme_node',
        executable='ltme_node',
        name='ltme_node',
        output='screen',
        parameters=[
            LaunchConfiguration('litra_params_file'),
            {
                'device_model': LaunchConfiguration('litra_model'),
                'device_address': LaunchConfiguration('litra_address'),
                'frame_id': LaunchConfiguration('litra_frame'),
            },
        ],
        remappings=[('scan', '/scan')],
    )

    static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='litra_static_tf',
        output='screen',
        arguments=[
            '--x', LaunchConfiguration('litra_x'),
            '--y', LaunchConfiguration('litra_y'),
            '--z', LaunchConfiguration('litra_z'),
            '--yaw', LaunchConfiguration('litra_yaw'),
            '--pitch', LaunchConfiguration('litra_pitch'),
            '--roll', LaunchConfiguration('litra_roll'),
            '--frame-id', 'base_link',
            '--child-frame-id', LaunchConfiguration('litra_frame'),
        ],
    )

    return LaunchDescription([
        params_arg,
        model_arg,
        address_arg,
        frame_arg,
        x_arg,
        y_arg,
        z_arg,
        yaw_arg,
        pitch_arg,
        roll_arg,
        lidar,
        static_tf,
    ])
