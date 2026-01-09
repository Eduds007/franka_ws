from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Declare launch arguments
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value='/home/nuc_6g_life_3/franka_ws/src/feats/src/feats/src/predict/predict_config.yaml',
        description='Path to FEATS configuration file'
    )
    
    camera1_topic_arg = DeclareLaunchArgument(
        'camera1_topic',
        default_value='/gelsight_cam1/image_raw',
        description='Topic name for camera 1'
    )
    
    camera2_topic_arg = DeclareLaunchArgument(
        'camera2_topic',
        default_value='/gelsight_cam2/image_raw',
        description='Topic name for camera 2'
    )
    
    # Create force estimator node
    force_estimator_node = Node(
        package='tension_estimator',
        executable='force_estimator_node',
        name='feats_force_estimator',
        output='screen',
        parameters=[{
            'config_file': LaunchConfiguration('config_file'),
            'camera1_topic': LaunchConfiguration('camera1_topic'),
            'camera2_topic': LaunchConfiguration('camera2_topic'),
        }]
    )
    
    return LaunchDescription([
        config_file_arg,
        camera1_topic_arg,
        camera2_topic_arg,
        force_estimator_node,
    ])
