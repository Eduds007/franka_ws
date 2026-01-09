#!/usr/bin/env python3
"""
GelSight Camera Launch File
Launches dual GelSight camera nodes for tactile sensing
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    # Get the path to the config file
    config_file = os.path.join(
        get_package_share_directory('tactile_cameras'),
        'config',
        'gelsight_params.yaml'
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
    
    # GelSight left camera node (C++)
    gelsight_left_node = Node(
        package='tactile_cameras',
        executable='gelsight_camera_node',
        name='gelsight_left_camera_node',
        output='screen',
        parameters=[{
            'camera_id': LaunchConfiguration('left_camera_id'),
            'frame_rate': LaunchConfiguration('frame_rate'),
            'camera_side': 'left',
            'image_width': 640,
            'image_height': 480
        }]
    )
    
    # GelSight right camera node (C++)
    gelsight_right_node = Node(
        package='tactile_cameras',
        executable='gelsight_camera_node',
        name='gelsight_right_camera_node',
        output='screen',
        parameters=[{
            'camera_id': LaunchConfiguration('right_camera_id'),
            'frame_rate': LaunchConfiguration('frame_rate'),
            'camera_side': 'right',
            'image_width': 640,
            'image_height': 480
        }]
    )
    
    return LaunchDescription([
        left_camera_id_arg,
        right_camera_id_arg,
        frame_rate_arg,
        gelsight_left_node,
        gelsight_right_node
    ])
