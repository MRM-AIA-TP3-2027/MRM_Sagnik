TARS Autonomous Rover Navigation

Overview

TARS is a ROS 2 based autonomous rover designed for goal driven navigation in both open environments and maze like structures. It combines LiDAR based obstacle avoidance, odometry based localization, and goal tracking using world coordinates or GPS.

The system uses a lightweight Vector Field Histogram method for local planning and a Bug2 style wall following strategy to escape local minima.

Features

LiDAR based obstacle avoidance using VFH
Goal navigation using odometry and GPS
Cross Track Error monitoring
Local minima detection and recovery
Wall following for maze escape
Smooth steering with oscillation control
Works in both open world and maze environments

Sensors Used

LiDAR, 360 degree scan for obstacle detection
Odometry, position and orientation
GPS, converted to ENU frame
Stereo Camera, used for perception extension

Navigation Pipeline

Receive goal from /goal_xy or /goal_gps
Convert GPS to ENU if needed
Align robot to goal direction
Drive using VFH steering
Detect local minima
Trigger wall following escape
Resume goal seeking after progress
Stop at goal

Project Structure

navigator.cpp or maze.cpp for core navigation logic
maze_launch.py for simulation setup
maze_ui.py for goal publishing
worlds folder for Gazebo environments

Running the Project

Build

colcon build --packages-select obstacle_avoidance
source install/setup.bash

Run Maze World

ros2 launch obstacle_avoidance maze_launch.py

This will:

Launch Gazebo
Spawn robot at maze entry
Start navigator
Publish goal automatically

Manual Goal

ros2 topic pub /goal_xy geometry_msgs/msg/Point "{x: 8.5, y: -7.0, z: 0.0}"

Core Algorithm

VFH Based Steering

The rover divides LiDAR into sectors and selects the safest direction closest to goal.

Pseudo logic:

for each sector
ignore blocked sectors
ignore large deviation from goal
score = distance minus goal bias
choose best sector

Steering Logic

if front is clear and heading error small
steer toward goal
else
steer using VFH best angle

Steering smoothing:

steer = 0.7 previous + 0.3 current

Speed Control

speed depends on:

obstacle distance
goal distance

Ensures:

slow near obstacles
slow near goal
minimum forward motion maintained

Local Minima Detection

Uses sliding window of position
Detects low movement over time
Disabled during alignment phase
Activated only after 2 seconds of driving

Escape Strategy

Rotate to find wall
Follow left wall
Maintain distance using proportional control
Exit when progress toward goal is detected

Cross Track Error

Measures deviation from straight path:

cte = perpendicular distance from path

Published on:

/tars/cross_track_error

Key Parameters

STOP_D for emergency stop distance
SLOW_D for slowdown threshold
WALL_FOLLOW_DIST for wall distance
ESC_PROGRESS_THR for escape completion

Improvements Implemented

Steering smoothing to remove oscillation
Forward bias to prevent spinning
Delayed minima detection to avoid false triggers
Stable wall following
Cross Track Error monitoring added

Limitations

VFH struggles in dense clutter
No global path planner
Depends on LiDAR quality

Future Work

Replace VFH with gap following
Add global planner like A star
Integrate stereo depth properly
Add SLAM for unknown environments

Summary

This project implements a complete local navigation system using reactive planning and recovery strategies. It works in both open environments and structured maze scenarios using a unified approach.

Youtube Video Link : 
