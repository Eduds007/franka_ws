#!/usr/bin/env python3
"""
Launch file for Cable Pose Estimator Node with config file support
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Get package share directory
    pkg_share = FindPackageShare('tactile_cameras')
    
    # Declare launch arguments
    use_config_arg = DeclareLaunchArgument(
        'use_config',
        default_value='true',
        description='Use config file instead of individual parameters'
    )
    
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=PathJoinSubstitution([
            pkg_share,
            'config',
            'cable_pose_estimator.yaml'
        ]),
        description='Path to config file'
    )
    
    camera_topic_arg = DeclareLaunchArgument(
        'camera_topic',
        default_value='/gelsight_cam1/image_raw',
        description='Camera topic to subscribe to'
    )
    
    debug_visualization_arg = DeclareLaunchArgument(
        'debug_visualization',
        default_value='true',
        description='Enable debug visualization images'
    )
    
    # Create node with config file
    cable_pose_estimator_node = Node(
        package='tactile_cameras',
        executable='cable_pose_estimator.py',
        name='cable_pose_estimator',
        output='screen',
        parameters=[LaunchConfiguration('config_file')],
        remappings=[
            ('camera_topic', LaunchConfiguration('camera_topic')),
        ]
    )
    
    return LaunchDescription([
        use_config_arg,
        config_file_arg,
        camera_topic_arg,
        debug_visualization_arg,
        cable_pose_estimator_node,
    ])
