#!/usr/bin/env python3
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    chapt3_bringup_share = get_package_share_directory('chapt3_bringup')
    chapt4_bringup_share = get_package_share_directory('chapt4_bringup')
    chapt4_slam_share = get_package_share_directory('chapt4_slam')

    chapt3_launch = os.path.join(
        chapt3_bringup_share, 'launch', 'bringup.launch.py')
    slam_launch = os.path.join(
        chapt4_slam_share, 'launch', 'slam.launch.py')
    default_slam_params = os.path.join(
        chapt4_slam_share, 'config', 'slam_toolbox_mapping.yaml')
    default_rviz = os.path.join(
        chapt4_slam_share, 'rviz', 'mapping.rviz')
    default_litra_params = os.path.join(
        chapt4_bringup_share, 'config', 'ltme_02a.yaml')
    litra_lidar_launch = os.path.join(
        chapt4_bringup_share, 'launch', 'lidar_litra.launch.py')

    port_arg = DeclareLaunchArgument(
        'port', default_value='/dev/ds_chassis',
        description='Chassis RS485 serial device')
    lidar_arg = DeclareLaunchArgument(
        'lidar', default_value='true',
        description='Start the selected 2D lidar and publish /scan')
    lidar_type_arg = DeclareLaunchArgument(
        'lidar_type', default_value='litra',
        description='2D lidar type: litra (LTME-02A) or rplidar (Chapt3 A1)')
    lidar_port_arg = DeclareLaunchArgument(
        'lidar_port', default_value='/dev/rplidar',
        description='RPLIDAR serial device')
    lidar_baudrate_arg = DeclareLaunchArgument(
        'lidar_baudrate', default_value='115200',
        description='RPLIDAR serial baudrate')
    lidar_scan_mode_arg = DeclareLaunchArgument(
        'lidar_scan_mode', default_value='Standard',
        description='RPLIDAR scan mode (Standard/Express)')

    litra_params_arg = DeclareLaunchArgument(
        'litra_params_file', default_value=default_litra_params,
        description='LitraTech ltme_node parameter file')
    litra_model_arg = DeclareLaunchArgument(
        'litra_model', default_value='LTME-02A',
        description='Litra model: LTME-02A, LT-R1, LT-R2, LT-I1 or LT-I2')
    litra_address_arg = DeclareLaunchArgument(
        'litra_address', default_value='192.168.10.160',
        description='Litra device IPv4 address')
    litra_frame_arg = DeclareLaunchArgument(
        'litra_frame', default_value='litra_link',
        description='LaserScan frame_id of the Litra lidar')
    litra_x_arg = DeclareLaunchArgument(
        'litra_x', default_value='0.08',
        description='Litra optical center x in base_link (m)')
    litra_y_arg = DeclareLaunchArgument(
        'litra_y', default_value='0.0',
        description='Litra optical center y in base_link (m)')
    litra_z_arg = DeclareLaunchArgument(
        'litra_z', default_value='0.21',
        description='Litra optical center z in base_link (m)')
    litra_yaw_arg = DeclareLaunchArgument(
        'litra_yaw', default_value='0.0',
        description='Litra yaw in base_link (rad)')
    litra_pitch_arg = DeclareLaunchArgument(
        'litra_pitch', default_value='0.0',
        description='Litra pitch in base_link (rad)')
    litra_roll_arg = DeclareLaunchArgument(
        'litra_roll', default_value='0.0',
        description='Litra roll in base_link (rad)')

    slam_arg = DeclareLaunchArgument(
        'slam', default_value='true',
        description='Start SLAM Toolbox mapping')
    slam_params_arg = DeclareLaunchArgument(
        'slam_params_file', default_value=default_slam_params,
        description='SLAM Toolbox mapping parameter file')
    slam_delay_arg = DeclareLaunchArgument(
        'slam_delay', default_value='4.0',
        description='Delay before starting SLAM Toolbox (seconds)')
    sim_time_arg = DeclareLaunchArgument(
        'use_sim_time', default_value='false',
        description='Use simulation clock (must stay false on the real robot)')
    autostart_arg = DeclareLaunchArgument(
        'autostart', default_value='true',
        description='Configure and activate the SLAM Toolbox lifecycle node')

    rviz_arg = DeclareLaunchArgument(
        'rviz', default_value='false',
        description='Start RViz2 on this machine')
    rviz_config_arg = DeclareLaunchArgument(
        'rviz_config', default_value=default_rviz,
        description='RViz2 config path')

    rplidar_enabled = PythonExpression([
        "'true' if '", LaunchConfiguration('lidar'), "' == 'true' and '",
        LaunchConfiguration('lidar_type'), "' == 'rplidar' else 'false'",
    ])
    litra_enabled = PythonExpression([
        "'true' if '", LaunchConfiguration('lidar'), "' == 'true' and '",
        LaunchConfiguration('lidar_type'), "' == 'litra' else 'false'",
    ])

    bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(chapt3_launch),
        launch_arguments={
            'port': LaunchConfiguration('port'),
            'lidar': rplidar_enabled,
            'lidar_port': LaunchConfiguration('lidar_port'),
            'lidar_baudrate': LaunchConfiguration('lidar_baudrate'),
            'lidar_scan_mode': LaunchConfiguration('lidar_scan_mode'),
        }.items(),
    )

    litra_lidar = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(litra_lidar_launch),
        launch_arguments={
            'litra_params_file': LaunchConfiguration('litra_params_file'),
            'litra_model': LaunchConfiguration('litra_model'),
            'litra_address': LaunchConfiguration('litra_address'),
            'litra_frame': LaunchConfiguration('litra_frame'),
            'litra_x': LaunchConfiguration('litra_x'),
            'litra_y': LaunchConfiguration('litra_y'),
            'litra_z': LaunchConfiguration('litra_z'),
            'litra_yaw': LaunchConfiguration('litra_yaw'),
            'litra_pitch': LaunchConfiguration('litra_pitch'),
            'litra_roll': LaunchConfiguration('litra_roll'),
        }.items(),
        condition=IfCondition(litra_enabled),
    )

    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(slam_launch),
        launch_arguments={
            'slam_params_file': LaunchConfiguration('slam_params_file'),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'autostart': LaunchConfiguration('autostart'),
        }.items(),
        condition=IfCondition(LaunchConfiguration('slam')),
    )
    slam_delayed = TimerAction(
        period=LaunchConfiguration('slam_delay'),
        actions=[slam],
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', LaunchConfiguration('rviz_config')],
        output='screen',
        condition=IfCondition(LaunchConfiguration('rviz')),
    )
    rviz_delayed = TimerAction(period=6.0, actions=[rviz])

    return LaunchDescription([
        port_arg,
        lidar_arg,
        lidar_type_arg,
        lidar_port_arg,
        lidar_baudrate_arg,
        lidar_scan_mode_arg,
        litra_params_arg,
        litra_model_arg,
        litra_address_arg,
        litra_frame_arg,
        litra_x_arg,
        litra_y_arg,
        litra_z_arg,
        litra_yaw_arg,
        litra_pitch_arg,
        litra_roll_arg,
        slam_arg,
        slam_params_arg,
        slam_delay_arg,
        sim_time_arg,
        autostart_arg,
        rviz_arg,
        rviz_config_arg,
        litra_lidar,
        bringup,
        slam_delayed,
        rviz_delayed,
    ])
