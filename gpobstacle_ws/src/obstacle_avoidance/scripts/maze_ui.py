#!/usr/bin/env python3
"""
maze_ui.py — TARS Maze Goal Publisher
======================================
Publishes a navigation goal to the TARS maze navigator via the /goal_xy topic.
Coordinates are Gazebo world XY — no GPS conversion needed.

Maze reference:
  Arena   : 18 m x 18 m  (x: -9 -> +9,  y: -9 -> +9)
  ENTRY   : x ~ -8.5,  y ~  7.0   (left wall gap, top)
  EXIT    : x ~  8.5,  y ~ -7.0   (right wall gap, bottom)  <- default goal

Usage
-----
  # Send the default maze-exit goal:
  ros2 run obstacle_avoidance maze_ui

  # Send a custom goal:
  ros2 run obstacle_avoidance maze_ui --ros-args -p goal_x:=8.5 -p goal_y:=-7.0

  # Or publish manually from terminal (no script needed):
  ros2 topic pub --once /goal_xy geometry_msgs/msg/Point "{x: 8.5, y: -7.0, z: 0.0}"
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point

# ── Change these two values to set a different goal ──────────────────────────
#   GOAL_X : Gazebo world X  (positive = east / right)
#   GOAL_Y : Gazebo world Y  (positive = north / up)
#   Maze exit  ->  GOAL_X =  8.5,  GOAL_Y = -7.0
#   Maze entry ->  GOAL_X = -8.5,  GOAL_Y =  7.0  (return trip)
DEFAULT_GOAL_X =  8.5   # <- maze EXIT
DEFAULT_GOAL_Y = -7.0   # <- maze EXIT
# ─────────────────────────────────────────────────────────────────────────────


class GoalUI(Node):
    def __init__(self):
        super().__init__('goal_ui')
        self.goal_sent = False

        self.declare_parameter('goal_x',    DEFAULT_GOAL_X)
        self.declare_parameter('goal_y',    DEFAULT_GOAL_Y)
        self.declare_parameter('delay_sec', 1.0)

        self.gx    = self.get_parameter('goal_x').get_parameter_value().double_value
        self.gy    = self.get_parameter('goal_y').get_parameter_value().double_value
        delay      = self.get_parameter('delay_sec').get_parameter_value().double_value

        self.pub   = self.create_publisher(Point, '/goal_xy', 10)

        # keep-alive timer so the node stays up after the goal is sent
        self.keep_alive_timer = self.create_timer(1.0, lambda: None)

        # one-shot timer that fires after `delay` seconds to publish the goal
        self.timer = self.create_timer(delay, self.publish_goal)

        self.get_logger().info(
            f'GoalUI ready — will publish goal ({self.gx:.2f}, {self.gy:.2f}) '
            f'in {delay:.1f}s on /goal_xy'
        )

    def publish_goal(self):
        if self.goal_sent:
            return

        msg = Point()
        msg.x = self.gx
        msg.y = self.gy
        msg.z = 0.0

        self.pub.publish(msg)
        self.get_logger().info(
            f'Goal published: world_x={self.gx:.2f}, world_y={self.gy:.2f}'
        )
        self.goal_sent = True
        self.get_logger().info('Goal sent — node staying alive for re-goals')


def main(args=None):
    rclpy.init(args=args)
    node = GoalUI()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
