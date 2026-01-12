#!/usr/bin/env python3
"""
Launch file para demonstração do MoveIt2 com robô Franka Panda REAL.
Este arquivo inicia o MoveIt configurado para o robô no IP 172.16.0.3
"""

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Incluir o launch do MoveIt para robô real
    moveit_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('franka_moveit_config'),
                'launch',
                'moveit.launch.py'
            ])
        ]),
        launch_arguments={
            'robot_ip': '172.16.0.3',
            'use_fake_hardware': 'false',
            'use_rviz': 'true',
        }.items()
    )
    
    # Nó do exemplo (iniciar após o MoveIt estar pronto)
    example_node = TimerAction(
        period=10.0,  # Aguardar 10 segundos para o MoveIt inicializar
        actions=[
            Node(
                package='franka_moveit_example',
                executable='moveit_real_robot',
                name='moveit_real_robot_example',
                output='screen',
            )
        ]
    )

    return LaunchDescription([
        moveit_launch,
        example_node,
    ])
