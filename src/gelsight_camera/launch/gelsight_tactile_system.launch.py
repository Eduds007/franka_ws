#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition


def generate_launch_description():
    # Get the path to config files
    gelsight_camera_config = os.path.join(
        get_package_share_directory('gelsight_camera'),
        'config',
        'gelsight_params.yaml'
    )
    
    tactile_sensor_config = os.path.join(
        get_package_share_directory('gelsight_camera'),
        'config',
        'tactile_sensor_params.yaml'
    )
    
    # Declare launch arguments
    left_camera_id_arg = DeclareLaunchArgument(
        'left_camera_id',
        default_value='0',
        description='Left camera device ID'
    )
    
    right_camera_id_arg = DeclareLaunchArgument(
        'right_camera_id',
        default_value='1',
        description='Right camera device ID'
    )
    
    frame_rate_arg = DeclareLaunchArgument(
        'frame_rate',
        default_value='30.0',
        description='Camera frame rate in Hz'
    )
    
    enable_tactile_analysis_arg = DeclareLaunchArgument(
        'enable_tactile_analysis',
        default_value='true',
        description='Enable tactile force analysis'
    )
    
    # GelSight left camera node
    gelsight_left_camera_node = Node(
        package='gelsight_camera',
        executable='gelsight_camera_node',
        name='gelsight_left_camera_node',
        parameters=[
            gelsight_camera_config,
            {
                'camera_id': LaunchConfiguration('left_camera_id'),
                'frame_rate': LaunchConfiguration('frame_rate'),
                'camera_side': 'left',
            }
        ],
        output='screen',
        emulate_tty=True,
    )
    
    # GelSight right camera node
    gelsight_right_camera_node = Node(
        package='gelsight_camera',
        executable='gelsight_camera_node',
        name='gelsight_right_camera_node',
        parameters=[
            gelsight_camera_config,
            {
                'camera_id': LaunchConfiguration('right_camera_id'),
                'frame_rate': LaunchConfiguration('frame_rate'),
                'camera_side': 'right',
            }
        ],
        output='screen',
        emulate_tty=True,
    )
    
    # Tactile sensor analysis node
    tactile_sensor_node = Node(
        package='gelsight_camera',
        executable='tactile_sensor_node',
        name='tactile_sensor_node',
        parameters=[
            tactile_sensor_config,
            {
                'frame_rate': LaunchConfiguration('frame_rate'),
            }
        ],
        output='screen',
        emulate_tty=True,
        condition=IfCondition(LaunchConfiguration('enable_tactile_analysis'))
    )
    
    return LaunchDescription([
        left_camera_id_arg,
        right_camera_id_arg,
        frame_rate_arg,
        enable_tactile_analysis_arg,
        gelsight_left_camera_node,
        gelsight_right_camera_node,
        tactile_sensor_node,
    ])