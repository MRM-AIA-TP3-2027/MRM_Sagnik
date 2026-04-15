#!/usr/bin/env python3
"""
gp_launch.py — TARS GPS + Obstacle-Avoidance Launch File
Launches Gazebo, spawns TARS, starts the OA navigator and RViz2.
"""

import os
import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, TimerAction

def generate_launch_description():

    pkg_path   = get_package_share_directory('obstacle_avoidance')
    world_file = os.path.join(pkg_path, 'worlds', 'my_world.world')
    xacro_file = os.path.join(pkg_path, 'urdf',   'TARS.xacro')
    robot_desc = xacro.process_file(xacro_file).toxml()

    # ── 1. Gazebo ─────────────────────────────────────────────────────────────
    #    libgazebo_ros_init.so    → clock, /spawn_entity, etc.
    #    libgazebo_ros_factory.so → /spawn_entity service
    gazebo = ExecuteProcess(
        cmd=[
            'gazebo', '--verbose',
            world_file,
            '-s', 'libgazebo_ros_init.so',
            '-s', 'libgazebo_ros_factory.so',
        ],
        output='screen'
    )

    # ── 2. Robot State Publisher ──────────────────────────────────────────────
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_desc}]
    )

    # ── 3. Joint State Publisher ──────────────────────────────────────────────
    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen'
    )

    # ── 4. Spawn TARS (after Gazebo stabilises — 3 s delay) ──────────────────
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

    # ── 5. TARS OA Navigator (after robot spawns — 5 s delay) ────────────────
    #
    # Parameter reference
    # ───────────────────
    #   linear_speed      Forward cruise speed [m/s]
    #   angular_speed     Max rotation speed   [rad/s]
    #   goal_tolerance    Stop radius around goal [m]
    #   heading_tolerance Alignment dead-band  [rad]
    #
    # Obstacle-avoidance distances (hard-coded in constants:: namespace;
    # tune by editing the constants block at the top of navigator.cpp):
    #   OBS_STOP_DIST  = 0.55 m   emergency stop / backup threshold
    #   OBS_SLOW_DIST  = 1.30 m   start reducing speed
    #   WALL_TARGET_DIST = 0.85 m wall-follow gap during escape
    #
    tars_navigator = Node(
        package='obstacle_avoidance',
        executable='tars_navigator',
        name='tars_navigator',
        output='screen',
        parameters=[{
            'linear_speed':      0.40,
            'angular_speed':     0.60,
            'goal_tolerance':    0.80,
            'heading_tolerance': 0.10,
        }]
    )

    # ── 6. RViz2 (after robot spawns — 5 s delay) ────────────────────────────
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
