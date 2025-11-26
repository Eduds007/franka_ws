#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    
    # Demo client node
    demo_client_node = Node(
        package='tension_control',
        executable='slide_cable_client.py',
        name='slide_cable_demo_client',
        output='screen',
    )
    
    # Admittance controller  
    admittance_controller_node = Node(
        package='tension_control',
        executable='admittance_controller',
        name='admittance_controller_for_primitive',
        output='screen',
        parameters=[{
            'control_frequency': 125.0,
            'mass_matrix': [8.0, 8.0, 12.0, 0.8, 0.8, 0.8],
            'damping_matrix': [40.0, 40.0, 60.0, 4.0, 4.0, 4.0],
            'stiffness_matrix': [80.0, 80.0, 120.0, 8.0, 8.0, 8.0],
            'desired_force': [0.0, 0.0, 5.0, 0.0, 0.0, 0.0],
            'force_deadzone': 0.1,
            'velocity_limit': 0.15,
            'auto_activate': False,
        }],
    )
    
    # Log information
    launch_info = LogInfo(
        msg=[
            '\\n=== Slide Along Cable Primitive Demo ===\\n',
            'Demo client will show example movements\\n',
            'Admittance controller ready for integration\\n',
            '\\nThe primitive provides:\\n',
            '  - Smooth cable sliding movements\\n',
            '  - Tension control integration\\n',
            '  - Safety limits and emergency stop\\n',
            '  - Real-time feedback\\n'
        ]
    )
    
    return LaunchDescription([
        launch_info,
        demo_client_node,
        admittance_controller_node,
    ])