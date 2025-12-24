import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # Point to the config file in the project root
    config = '/home/windiff/Code/Simulation/config/identification.yaml'

    return LaunchDescription([
        Node(
            package='identification',
            executable='identify_node',
            name='identification_node',
            output='screen',
            parameters=[config]
        )
    ])
