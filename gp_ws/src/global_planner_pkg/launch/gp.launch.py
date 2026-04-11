#!/usr/bin/env python3

import os
import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, TimerAction

def generate_launch_description():

    pkg_path = get_package_share_directory('global_planner_pkg')
    world_file = os.path.join(pkg_path,'worlds', 'my_world.world')
    xacro_file = os.path.join(pkg_path, 'urdf', 'TARS.xacro')
    robot_description = xacro.process_file(xacro_file).toxml()

    # ── 1. Gazebo ──────────────────────────────────────────────────
    gazebo = ExecuteProcess(
        cmd=['gazebo', '--verbose',
             world_file,
             '-s', 'libgazebo_ros_init.so',      # ← needed for sensor topics (GPS/IMU)
             '-s', 'libgazebo_ros_factory.so'],
        output='screen'
    )

    # ── 2. Robot State Publisher ───────────────────────────────────
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description}]
    )

    # ── 3. Joint State Publisher ───────────────────────────────────
    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen'
    )

    # ── 4. Spawn TARS robot (after Gazebo loads — 3s delay) ────────
    spawn_robot = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-topic', 'robot_description',
            '-entity', 'tars_robot',
            '-x', '0.0',
            '-y', '0.0',
            '-z', '0.3',
        ],
        output='screen'
    )

    # ── 5. TARS Navigator (after robot spawns — 5s delay) ──────────
    tars_navigator = Node(
        package='global_planner_pkg',
        executable='tars_navigator',
        name='tars_navigator',
        output='screen',
        parameters=[{
            'linear_speed':      0.4,
            'angular_speed':     0.6,
            'goal_tolerance':    0.8,
            'heading_tolerance': 0.10,
        }]
    )

    # ── 6. RViz2 (after robot spawns — 5s delay) ───────────────────
    rviz_config = os.path.join(pkg_path, 'rviz', 'tars.rviz')
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config] if os.path.exists(rviz_config) else [],
    )

    return LaunchDescription([
        gazebo,
        robot_state_publisher,
        joint_state_publisher,
        TimerAction(period=3.0, actions=[spawn_robot]),
        TimerAction(period=5.0, actions=[tars_navigator, rviz]),
    ])
