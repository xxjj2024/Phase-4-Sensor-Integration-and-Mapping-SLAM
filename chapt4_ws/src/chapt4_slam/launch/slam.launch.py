#!/usr/bin/env python3
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    slam_share = get_package_share_directory('chapt4_slam')
    slam_toolbox_share = get_package_share_directory('slam_toolbox')

    default_params = os.path.join(
        slam_share, 'config', 'slam_toolbox_mapping.yaml')
    slam_toolbox_launch = os.path.join(
        slam_toolbox_share, 'launch', 'online_async_launch.py')

    params_arg = DeclareLaunchArgument(
        'slam_params_file', default_value=default_params,
        description='SLAM Toolbox mapping parameter file')
    sim_time_arg = DeclareLaunchArgument(
        'use_sim_time', default_value='false',
        description='Use simulation clock (must stay false on the real robot)')
    autostart_arg = DeclareLaunchArgument(
        'autostart', default_value='true',
        description='Configure and activate the SLAM Toolbox lifecycle node')

    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(slam_toolbox_launch),
        launch_arguments={
            'slam_params_file': LaunchConfiguration('slam_params_file'),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'autostart': LaunchConfiguration('autostart'),
            'use_lifecycle_manager': 'false',
        }.items(),
    )

    return LaunchDescription([
        params_arg,
        sim_time_arg,
        autostart_arg,
        slam,
    ])
