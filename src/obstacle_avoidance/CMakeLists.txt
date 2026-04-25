cmake_minimum_required(VERSION 3.8)
project(obstacle_avoidance)

add_compile_options(-std=c++17)

find_package(ament_cmake REQUIRED)
find_package(rclcpp        REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs   REQUIRED)   # LaserScan, NavSatFix, Imu, Image
find_package(std_msgs      REQUIRED)
find_package(gazebo_msgs   REQUIRED)
find_package(nav_msgs      REQUIRED)   # kept for odom fallback / RViz

include_directories(include)

add_executable(tars_navigator src/navigator.cpp)

ament_target_dependencies(tars_navigator
  rclcpp
  geometry_msgs
  nav_msgs
  sensor_msgs
  std_msgs
  gazebo_msgs
)

# ── Maze solver (LiDAR + Odometry only, no GPS/IMU) ──────────────────────────
add_executable(tars_maze src/maze.cpp)

ament_target_dependencies(tars_maze
  rclcpp
  geometry_msgs
  nav_msgs
  sensor_msgs
  std_msgs
  gazebo_msgs
)


install(TARGETS
  tars_navigator
  tars_maze
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch urdf rviz
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/goal_ui.py
  scripts/maze_ui.py
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY worlds
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
