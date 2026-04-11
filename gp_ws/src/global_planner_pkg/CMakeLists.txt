cmake_minimum_required(VERSION 3.8)
project(global_planner_pkg)

add_compile_options(-std=c++17)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(gazebo_msgs REQUIRED)
find_package(nav_msgs REQUIRED)

include_directories(include)

add_executable(tars_navigator src/navigator.cpp)

ament_target_dependencies(tars_navigator
  rclcpp
  geometry_msgs
  nav_msgs
  std_msgs
  gazebo_msgs
)

install(TARGETS tars_navigator
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch urdf rviz
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/goal_ui.py
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY worlds
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
