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
    tension_control_config = os.path.join(
        get_package_share_directory('tension_control'),
        'config',
        'gelsight_params.yaml'
    )
    
    tactile_sensor_config = os.path.join(
        get_package_share_directory('tension_control'),
        'config',
        'tactile_sensor_params.yaml'
    )
    
    admittance_config = os.path.join(
        get_package_share_directory('tension_control'),
        'config',
        'admittance_params.yaml'
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
    
    enable_cameras_arg = DeclareLaunchArgument(
        'enable_cameras',
        default_value='true',
        description='Enable camera nodes (set false for simulation)'
    )
    
    enable_tactile_analysis_arg = DeclareLaunchArgument(
        'enable_tactile_analysis',
        default_value='true',
        description='Enable tactile force analysis'
    )
    
    enable_admittance_control_arg = DeclareLaunchArgument(
        'enable_admittance_control',
        default_value='true',
        description='Enable admittance controller'
    )
    
    desired_force_arg = DeclareLaunchArgument(
        'desired_force',
        default_value='5.0',
        description='Desired cable tension force in Newtons'
    )
    
    # GelSight left camera node
    gelsight_left_camera_node = Node(
        package='tension_control',
        executable='tension_control_node',
        name='gelsight_left_camera_node',
        parameters=[
            tension_control_config,
            {
                'camera_id': LaunchConfiguration('left_camera_id'),
                'frame_rate': LaunchConfiguration('frame_rate'),
                'camera_side': 'left',
            }
        ],
        output='screen',
        emulate_tty=True,
        condition=IfCondition(LaunchConfiguration('enable_cameras'))
    )
    
    # GelSight right camera node
    gelsight_right_camera_node = Node(
        package='tension_control',
        executable='tension_control_node',
        name='gelsight_right_camera_node',
        parameters=[
            tension_control_config,
            {
                'camera_id': LaunchConfiguration('right_camera_id'),
                'frame_rate': LaunchConfiguration('frame_rate'),
                'camera_side': 'right',
            }
        ],
        output='screen',
        emulate_tty=True,
        condition=IfCondition(LaunchConfiguration('enable_cameras'))
    )
    
    # Tactile sensor analysis node
    tactile_sensor_node = Node(
        package='tension_control',
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
    
    # Admittance controller node
    admittance_controller_node = Node(
        package='tension_control',
        executable='admittance_controller',
        name='admittance_controller',
        parameters=[
            admittance_config,
            {
                'desired_force_z': LaunchConfiguration('desired_force'),
            }
        ],
        output='screen',
        emulate_tty=True,
        condition=IfCondition(LaunchConfiguration('enable_admittance_control'))
    )
    
    return LaunchDescription([
        left_camera_id_arg,
        right_camera_id_arg,
        frame_rate_arg,
        enable_cameras_arg,
        enable_tactile_analysis_arg,
        enable_admittance_control_arg,
        desired_force_arg,
        gelsight_left_camera_node,
        gelsight_right_camera_node,
        tactile_sensor_node,
        admittance_controller_node,
    ])