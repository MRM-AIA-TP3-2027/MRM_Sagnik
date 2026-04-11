#!/usr/bin/env python3
"""
goal_ui.py — Interactive terminal goal sender for TARS
Run in a second terminal:
    python3 ~/gp_ws/src/global_planner_pkg/scripts/goal_ui.py
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
import time

class GoalUI(Node):
    def __init__(self):
        super().__init__('goal_ui')
        self.pub = self.create_publisher(Point, '/goal_gps', 10)
        time.sleep(1.0)  # wait for connection

    def send_goal(self, lat: float, lon: float):
        msg = Point()
        msg.x = lat   # latitude
        msg.y = lon   # longitude
        msg.z = 0.0
        self.pub.publish(msg)
        print(f"Goal sent → lat={lat:.6f}  lon={lon:.6f}")
        print("Watch Gazebo — red sphere will appear at goal\n")

def main():
    rclpy.init()
    node = GoalUI()

    print("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━")
    print("       TARS Goal Sender     ")
    print("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━")
    print("  GPS reference (Gazebo origin):")
    print("    lat = 12.9716,  lon = 77.5946")
    print("  Type 'q' to quit\n")

    while rclpy.ok():
        try:
            lat_str = input("  Goal latitude  : ").strip()
            if lat_str.lower() == 'q':
                break

            lon_str = input("  Goal longitude : ").strip()
            if lon_str.lower() == 'q':
                break

            lat = float(lat_str)
            lon = float(lon_str)

            if not (-90 <= lat <= 90):
                print("Latitude must be between -90 and 90\n")
                continue
            if not (-180 <= lon <= 180):
                print("Longitude must be between -180 and 180\n")
                continue

            node.send_goal(lat, lon)
            print("Robot is navigating... send next goal only after it stops.\n")

        except ValueError:
            print("Enter a valid number\n")
        except KeyboardInterrupt:
            pass

    print("\n  Shutting down.")
    if rclpy.ok():
    	rclpy.shutdown()

if __name__ == '__main__':
    main()
