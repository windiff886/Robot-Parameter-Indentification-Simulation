from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'data_file',
            default_value='',
            description='Path to the CSV identification data file'
        ),
        Node(
            package='identification',
            executable='identify_node',
            name='identification_node',
            output='screen',
            parameters=[{'data_file': LaunchConfiguration('data_file')}]
        )
    ])
