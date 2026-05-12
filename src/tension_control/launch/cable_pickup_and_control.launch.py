#!/usr/bin/env python3
"""End-to-end launch: Franka + MoveIt + grasp orchestrator (deterministic, image-free).

Composes:
  - franka.launch.py            (controller_manager, JTC, joint_state + robot_state broadcasters, gripper)
  - move_group                  (MoveIt2 planning, no extra ros2_control_node)
  - cable_grasp_orchestrator    (FSM: pre-grasp → close → home → switch → trajectory)

multi_priority_controller is loaded INACTIVE; the orchestrator activates it
after the home pose is reached and anchor_position has been pushed via
SetParameters.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import yaml


def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)
    try:
        with open(absolute_file_path, 'r') as f:
            return yaml.safe_load(f)
    except EnvironmentError:
        return None


def generate_launch_description():
    robot_ip = LaunchConfiguration('robot_ip')
    use_fake_hardware = LaunchConfiguration('use_fake_hardware')
    use_rviz = LaunchConfiguration('use_rviz')
    orchestrator_params_file = LaunchConfiguration('orchestrator_params_file')

    default_orchestrator_params = os.path.join(
        get_package_share_directory('tension_control'),
        'config',
        'cable_grasp_orchestrator.yaml',
    )

    # ── Franka bringup (controller_manager, JTC, gripper, broadcasters) ────────
    franka_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([PathJoinSubstitution(
            [FindPackageShare('franka_bringup'), 'launch', 'franka.launch.py'])]),
        launch_arguments={
            'robot_ip': robot_ip,
            'use_fake_hardware': use_fake_hardware,
            'use_rviz': 'false',
            'load_gripper': 'true',
        }.items(),
    )

    jtc_spawner = TimerAction(
        period=3.0,
        actions=[
            Node(
                package='controller_manager',
                executable='spawner',
                arguments=['joint_trajectory_controller',
                           '--controller-manager', '/controller_manager'],
                output='screen',
            ),
        ],
    )

    mpc_loader = TimerAction(
        period=4.5,
        actions=[
            Node(
                package='controller_manager',
                executable='spawner',
                arguments=['multi_priority_controller',
                           '--controller-manager', '/controller_manager',
                           '--inactive'],
                output='screen',
            ),
        ],
    )

    # ── MoveIt move_group ──────────────────────────────────────────────────────
    franka_xacro_file = os.path.join(
        get_package_share_directory('franka_description'), 'robots', 'panda_arm.urdf.xacro')
    robot_description_config = Command(
        [FindExecutable(name='xacro'), ' ', franka_xacro_file, ' hand:=true',
         ' robot_ip:=', robot_ip, ' use_fake_hardware:=', use_fake_hardware,
         ' fake_sensor_commands:=false'])
    robot_description = {'robot_description': robot_description_config}

    franka_semantic_xacro_file = os.path.join(
        get_package_share_directory('franka_moveit_config'), 'srdf', 'panda_arm.srdf.xacro')
    robot_description_semantic_config = Command(
        [FindExecutable(name='xacro'), ' ', franka_semantic_xacro_file, ' hand:=true'])
    robot_description_semantic = {
        'robot_description_semantic': robot_description_semantic_config}

    kinematics_yaml = load_yaml('franka_moveit_config', 'config/kinematics.yaml') or {}
    ompl_planning_yaml = load_yaml('franka_moveit_config', 'config/ompl_planning.yaml') or {}
    ompl_planning_pipeline_config = {
        'move_group': {
            'planning_plugin': 'ompl_interface/OMPLPlanner',
            'request_adapters': 'default_planner_request_adapters/AddTimeOptimalParameterization '
                                'default_planner_request_adapters/ResolveConstraintFrames '
                                'default_planner_request_adapters/FixWorkspaceBounds '
                                'default_planner_request_adapters/FixStartStateBounds '
                                'default_planner_request_adapters/FixStartStateCollision '
                                'default_planner_request_adapters/FixStartStatePathConstraints',
            'start_state_max_bounds_error': 0.1,
        }
    }
    ompl_planning_pipeline_config['move_group'].update(ompl_planning_yaml)

    # The franka_moveit_config default expects a controller named
    # 'panda_arm_controller', but franka_bringup spawns it as
    # 'joint_trajectory_controller'. Build the moveit controller config inline
    # so MoveIt sends FollowJointTrajectory to the right action server.
    moveit_simple_controllers_yaml = {
        'controller_names': ['joint_trajectory_controller', 'panda_gripper'],
        'joint_trajectory_controller': {
            'action_ns': 'follow_joint_trajectory',
            'type': 'FollowJointTrajectory',
            'default': True,
            'joints': [
                'panda_joint1', 'panda_joint2', 'panda_joint3', 'panda_joint4',
                'panda_joint5', 'panda_joint6', 'panda_joint7',
            ],
        },
        'panda_gripper': {
            'action_ns': 'gripper_action',
            'type': 'GripperCommand',
            'default': True,
            'joints': ['panda_finger_joint1', 'panda_finger_joint2'],
        },
    }
    moveit_controllers = {
        'moveit_simple_controller_manager': moveit_simple_controllers_yaml,
        'moveit_controller_manager':
            'moveit_simple_controller_manager/MoveItSimpleControllerManager',
    }
    trajectory_execution = {
        'moveit_manage_controllers': True,
        'trajectory_execution.allowed_execution_duration_scaling': 1.2,
        'trajectory_execution.allowed_goal_duration_margin': 0.5,
        'trajectory_execution.allowed_start_tolerance': 0.01,
    }
    planning_scene_monitor_parameters = {
        'publish_planning_scene': True,
        'publish_geometry_updates': True,
        'publish_state_updates': True,
        'publish_transforms_updates': True,
    }

    move_group_node = TimerAction(
        period=5.0,
        actions=[
            Node(
                package='moveit_ros_move_group',
                executable='move_group',
                output='screen',
                parameters=[
                    robot_description,
                    robot_description_semantic,
                    kinematics_yaml,
                    ompl_planning_pipeline_config,
                    trajectory_execution,
                    moveit_controllers,
                    planning_scene_monitor_parameters,
                ],
            ),
        ],
    )

    rviz_base = os.path.join(get_package_share_directory('franka_moveit_config'), 'rviz')
    rviz_full_config = os.path.join(rviz_base, 'moveit.rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='log',
        arguments=['-d', rviz_full_config],
        parameters=[
            robot_description,
            robot_description_semantic,
            ompl_planning_pipeline_config,
            kinematics_yaml,
        ],
        condition=IfCondition(use_rviz),
    )

    # ── GelSight cameras + cable pose estimators ───────────────────────────────
    camera_publisher_node = Node(
        package='tactile_cameras',
        executable='camera_publisher.py',
        name='gelsight_camera_publisher',
        output='screen',
        parameters=[{
            'camera1_id': 0,
            'camera2_id': 2,
            'fps': 30.0,
            'width': 640,
            'height': 480,
        }],
    )
    cable_pose_estimator_cam1 = Node(
        package='tactile_cameras',
        executable='cable_pose_estimator.py',
        name='cable_pose_estimator_cam1',
        output='screen',
        parameters=[{
            'camera_topic': '/gelsight_cam1/image_raw',
            'diff_threshold': 5,         # default 10 — more sensitive
            'min_contour_area': 100,     # default 500 — accept smaller blobs
            'border_size': 60,           # default 120 — narrower border mask
        }],
    )
    cable_pose_estimator_cam2 = Node(
        package='tactile_cameras',
        executable='cable_pose_estimator.py',
        name='cable_pose_estimator_cam2',
        output='screen',
        parameters=[{'camera_topic': '/gelsight_cam2/image_raw'}],
    )

    # ── Orchestrator (FSM) ─────────────────────────────────────────────────────
    orchestrator_node = TimerAction(
        period=10.0,
        actions=[
            Node(
                package='tension_control',
                executable='cable_grasp_orchestrator.py',
                name='cable_grasp_orchestrator',
                output='screen',
                parameters=[orchestrator_params_file],
            ),
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument('robot_ip', description='Hostname or IP of the robot.'),
        DeclareLaunchArgument('use_fake_hardware', default_value='false'),
        DeclareLaunchArgument('use_rviz', default_value='false'),
        DeclareLaunchArgument('orchestrator_params_file',
                              default_value=default_orchestrator_params),
        franka_launch,
        jtc_spawner,
        mpc_loader,
        move_group_node,
        rviz_node,
        camera_publisher_node,
        cable_pose_estimator_cam1,
        orchestrator_node,
    ])
