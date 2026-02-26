#!/usr/bin/env python3
"""
Launch file for Cable Pose Estimator Node
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    # Declare launch arguments
    camera_topic_arg = DeclareLaunchArgument(
        'camera_topic',
        default_value='/gelsight_cam1/image_raw',
        description='Camera topic to subscribe to'
    )
    
    border_size_arg = DeclareLaunchArgument(
        'border_size',
        default_value='120',
        description='Border size in pixels to ignore'
    )
    
    diff_threshold_arg = DeclareLaunchArgument(
        'diff_threshold',
        default_value='10',
        description='Threshold for image difference detection'
    )
    
    min_contour_area_arg = DeclareLaunchArgument(
        'min_contour_area',
        default_value='500',
        description='Minimum contour area in pixels'
    )
    
    debug_visualization_arg = DeclareLaunchArgument(
        'debug_visualization',
        default_value='true',
        description='Enable debug visualization images'
    )
    
    publish_rate_arg = DeclareLaunchArgument(
        'publish_rate',
        default_value='30.0',
        description='Publishing rate in Hz'
    )
    
    # Create node
    cable_pose_estimator_node = Node(
        package='tactile_cameras',
        executable='cable_pose_estimator.py',
        name='cable_pose_estimator',
        output='screen',
        parameters=[{
            'camera_topic': LaunchConfiguration('camera_topic'),
            'border_size': LaunchConfiguration('border_size'),
            'diff_threshold': LaunchConfiguration('diff_threshold'),
            'min_contour_area': LaunchConfiguration('min_contour_area'),
            'num_top_regions': 5,
            'morph_kernel_size': 15,
            'publish_rate': LaunchConfiguration('publish_rate'),
            'debug_visualization': LaunchConfiguration('debug_visualization'),
            'pixel_to_mm': 0.0634,
        }]
    )
    
    return LaunchDescription([
        camera_topic_arg,
        border_size_arg,
        diff_threshold_arg,
        min_contour_area_arg,
        debug_visualization_arg,
        publish_rate_arg,
        cable_pose_estimator_node,
    ])
