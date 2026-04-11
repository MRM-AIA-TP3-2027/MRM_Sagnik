from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_path = get_package_share_directory('robomechtars_description')
    xacro_file = os.path.join(pkg_path, 'urdf', 'TARS.xacro')

    import xacro
    robot_description = xacro.process_file(xacro_file).toxml()

    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{
                'robot_description': robot_description
            }]
        ),
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=['-topic', 'robot_description', '-entity', 'TARS'],
            output='screen'
        ),
    ])
