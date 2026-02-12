#!/usr/bin/env python3
"""
Launch file for Tactile Contact Analyzer node
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    """Generate launch description for tactile contact analyzer."""
    
    # Declare launch arguments
    camera_topic_arg = DeclareLaunchArgument(
        'camera_topic',
        default_value='/gelsight_cam1/image_raw',
        description='Topic name for tactile camera images'
    )
    
    border_size_arg = DeclareLaunchArgument(
        'border_size',
        default_value='120',
        description='Border size in pixels to ignore'
    )
    
    diff_threshold_arg = DeclareLaunchArgument(
        'diff_threshold',
        default_value='10',
        description='Threshold for difference detection'
    )
    
    debug_viz_arg = DeclareLaunchArgument(
        'debug_visualization',
        default_value='true',
        description='Enable debug visualization'
    )
    
    # Contact analyzer node
    contact_analyzer_node = Node(
        package='tactile_cameras',
        executable='tactile_contact_analyzer',
        name='tactile_contact_analyzer',
        output='screen',
        parameters=[{
            'camera_topic': LaunchConfiguration('camera_topic'),
            'border_size': LaunchConfiguration('border_size'),
            'diff_threshold': LaunchConfiguration('diff_threshold'),
            'min_contour_area': 500,
            'num_top_regions': 5,
            'morph_kernel_size': 15,
            'publish_rate': 30.0,
            'debug_visualization': LaunchConfiguration('debug_visualization'),
            'auto_reference_capture': True,
        }]
    )
    
    return LaunchDescription([
        camera_topic_arg,
        border_size_arg,
        diff_threshold_arg,
        debug_viz_arg,
        contact_analyzer_node,
    ])
