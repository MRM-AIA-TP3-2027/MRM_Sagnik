#!/usr/bin/env python3
"""
maze_launch.py — TARS Maze Navigation Launch File
Launches Gazebo with maze_world, spawns TARS at the maze ENTRY,
starts the OA navigator, goal_ui (publishes maze exit goal), and RViz2.

── Coordinate reference ──────────────────────────────────────────────────────
  Maze arena  : 18 m × 18 m  (x: -9 → +9,  y: -9 → +9)
  ENTRY gap   : x = -9,  y = 4.5 → 9   → spawn at (-8.5,  7.0)  ← robot start
  EXIT  gap   : x = +9,  y = -9 → -4.5 → goal  at ( 8.5, -7.0)  ← robot goal
─────────────────────────────────────────────────────────────────────────────
"""

import os
import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, TimerAction


def generate_launch_description():

    pkg_path   = get_package_share_directory('obstacle_avoidance')
    world_file = os.path.join(pkg_path, 'worlds', 'maze_world.world')
    xacro_file = os.path.join(pkg_path, 'urdf',   'TARS.xacro')
    robot_desc = xacro.process_file(xacro_file).toxml()

    # ── Spawn position — maze ENTRY gap ──────────────────────────────────────
    #    Left outer wall gap is at x=-9, y=4.5→9.
    #    Place the robot just inside the entry: x=-8.5, y=7.0
    SPAWN_X =  '-8.5'
    SPAWN_Y =   '7.0'
    SPAWN_Z =   '0.3'

    # ── Goal position — maze EXIT gap ─────────────────────────────────────────
    #    Right outer wall gap is at x=+9, y=-9→-4.5.
    GOAL_X  =   8.5
    GOAL_Y  =  -7.0

    # ── 1. Gazebo ─────────────────────────────────────────────────────────────
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

    # ── 4. Spawn TARS at maze ENTRY (3 s delay for Gazebo to stabilise) ──────
    spawn_robot = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-topic', 'robot_description',
            '-entity', 'tars_robot',
            '-x', SPAWN_X,
            '-y', SPAWN_Y,
            '-z', SPAWN_Z,
        ],
        output='screen'
    )

    # ── 5. TARS Maze Navigator (5 s delay — after robot spawns) ──────────────
    #
    # Parameter reference
    # ───────────────────
    #   linear_speed      Forward cruise speed [m/s]
    #   angular_speed     Max rotation speed   [rad/s]
    #   goal_tolerance    Stop radius around goal [m]
    #   heading_tolerance Alignment dead-band  [rad]
    #
    tars_navigator = Node(
        package='obstacle_avoidance',
        executable='tars_maze',
        name='tars_maze',
        output='screen',
        parameters=[{
            'linear_speed':      0.40,
            'angular_speed':     0.80,
            'goal_tolerance':    0.80,
            'heading_tolerance': 0.10,
        }]
    )

    # ── 6. Goal UI — publishes maze exit to /goal_xy (7 s delay) ─────────────
    #    NOTE: executable name must match the installed filename (maze_ui.py),
    #    not the stem without extension. ROS 2 installs Python scripts with
    #    their original filename, so 'maze_ui.py' is the correct executable.
    goal_ui = Node(
        package='obstacle_avoidance',
        executable='maze_ui.py',          # ← FIX: was 'maze_ui' (caused "not found" error)
        name='maze_ui',
        output='screen',
        respawn=True,
        parameters=[{
            'goal_x':    GOAL_X,
            'goal_y':    GOAL_Y,
            'delay_sec': 1.0,
        }]
    )

    # ── 7. RViz2 (5 s delay) ─────────────────────────────────────────────────
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
        TimerAction(period=7.0, actions=[goal_ui]),
    ])
