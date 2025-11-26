#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Declare launch arguments
    control_frequency_arg = DeclareLaunchArgument(
        'control_frequency', 
        default_value='125.0',
        description='Control loop frequency in Hz'
    )
    
    feedback_frequency_arg = DeclareLaunchArgument(
        'feedback_frequency',
        default_value='10.0', 
        description='Feedback publishing frequency in Hz'
    )
    
    max_execution_time_arg = DeclareLaunchArgument(
        'max_execution_time',
        default_value='60.0',
        description='Maximum execution time for primitives in seconds'
    )
    
    slide_velocity_arg = DeclareLaunchArgument(
        'default_slide_velocity',
        default_value='0.05',
        description='Default sliding velocity in m/s'
    )
    
    desired_tension_arg = DeclareLaunchArgument(
        'default_desired_tension',
        default_value='5.0',
        description='Default desired cable tension in N'
    )
    
    safety_multiplier_arg = DeclareLaunchArgument(
        'safety_tension_multiplier',
        default_value='1.5',
        description='Safety multiplier for maximum tension limit'
    )
    
    enable_demo_arg = DeclareLaunchArgument(
        'enable_demo',
        default_value='false',
        description='Enable demo client to run example movements'
    )
    
    # Get launch configurations
    control_frequency = LaunchConfiguration('control_frequency')
    feedback_frequency = LaunchConfiguration('feedback_frequency')
    max_execution_time = LaunchConfiguration('max_execution_time')
    slide_velocity = LaunchConfiguration('default_slide_velocity')
    desired_tension = LaunchConfiguration('default_desired_tension')
    safety_multiplier = LaunchConfiguration('safety_tension_multiplier')
    enable_demo = LaunchConfiguration('enable_demo')
    
    # Slide along cable primitive node
    slide_primitive_node = Node(
        package='tension_control',
        executable='slide_along_cable_primitive.py',
        name='slide_along_cable_primitive',
        output='screen',
        parameters=[{
            'control_frequency': control_frequency,
            'feedback_frequency': feedback_frequency,
            'max_execution_time': max_execution_time,
            'velocity_smoothing_factor': 0.9,
            'trajectory_interpolation_points': 100,
            'safety_tension_multiplier': safety_multiplier,
            'emergency_stop_acceleration': 0.5,
        }],
        remappings=[
            # Input topics (from robot and sensors)
            ('/robot/current_pose', '/franka/end_effector_pose'),
            ('/gelsight/combined_tension', '/tactile_sensor/estimated_force'),
            
            # Output topics (to admittance controller)
            ('/slide_primitive/desired_velocity', '/admittance_controller/desired_velocity'),
        ]
    )
    
    # Demo client node (optional)
    demo_client_node = Node(
        package='tension_control',
        executable='slide_cable_client.py',
        name='slide_cable_demo_client',
        output='screen',
        condition=IfCondition(enable_demo),
        parameters=[{
            'default_slide_velocity': slide_velocity,
            'default_desired_tension': desired_tension,
        }]
    )
    
    # Admittance controller (if not already running)
    admittance_controller_node = Node(
        package='tension_control',
        executable='admittance_controller',
        name='admittance_controller_for_primitive',
        output='screen',
        parameters=[{
            'control_frequency': control_frequency,
            'mass_matrix': [10.0, 10.0, 10.0, 1.0, 1.0, 1.0],  # M matrix diagonal
            'damping_matrix': [50.0, 50.0, 50.0, 5.0, 5.0, 5.0],  # B matrix diagonal
            'stiffness_matrix': [100.0, 100.0, 100.0, 10.0, 10.0, 10.0],  # K matrix diagonal
            'desired_force': [0.0, 0.0, desired_tension, 0.0, 0.0, 0.0],  # Cable tension in Z
            'force_deadzone': 0.1,
            'velocity_limit': 0.2,
            'auto_activate': False,  # Primitive will activate when needed
        }],
        remappings=[
            ('/admittance_controller/desired_velocity', '/slide_primitive/desired_velocity'),
            ('/admittance_controller/cmd_vel', '/franka/cartesian_velocity_controller/cmd_vel'),
            ('/admittance_controller/external_force', '/tactile_sensor/estimated_force'),
        ]
    )
    
    # Log information
    launch_info = LogInfo(
        msg=[
            '\\n=== Slide Along Cable Primitive Launch ===\\n',
            'Control frequency: ', control_frequency, ' Hz\\n',
            'Feedback frequency: ', feedback_frequency, ' Hz\\n',  
            'Max execution time: ', max_execution_time, ' seconds\\n',
            'Default slide velocity: ', slide_velocity, ' m/s\\n',
            'Default desired tension: ', desired_tension, ' N\\n',
            'Safety tension multiplier: ', safety_multiplier, '\\n',
            'Demo client enabled: ', enable_demo, '\\n',
            '\\nTopics published:\\n',
            '  - /slide_primitive/desired_velocity (geometry_msgs/TwistStamped)\\n',
            '  - /slide_primitive/current_tension (std_msgs/Float32)\\n', 
            '  - /slide_primitive/status (std_msgs/String)\\n',
            '  - /slide_primitive/feedback (std_msgs/String)\\n',
            '\\nTopics subscribed:\\n',
            '  - /franka/end_effector_pose (geometry_msgs/PoseStamped)\\n',
            '  - /tactile_sensor/estimated_force (geometry_msgs/WrenchStamped)\\n',
            '\\nServices available:\\n',
            '  - /slide_primitive/start (custom service)\\n',
            '  - /slide_primitive/stop (custom service)\\n',
            '  - /slide_primitive/emergency_stop (std_srvs/Trigger)\\n'
        ]
    )
    
    return LaunchDescription([
        # Arguments
        control_frequency_arg,
        feedback_frequency_arg,
        max_execution_time_arg,
        slide_velocity_arg,
        desired_tension_arg,
        safety_multiplier_arg,
        enable_demo_arg,
        
        # Launch info
        launch_info,
        
        # Nodes
        slide_primitive_node,
        admittance_controller_node,
        demo_client_node,
    ])