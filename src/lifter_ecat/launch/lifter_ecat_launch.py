import os
import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch_ros.actions import Node

def generate_launch_description():
    config_file = os.path.join(os.path.dirname(__file__), '../config/lifter_config.yaml')

    return LaunchDescription([
        LogInfo(msg=f"Loading config from {config_file}"),

        Node(
            package='lifter_ecat',
            executable='lifter_node',
            name='lifter_node',
            output='screen',
            parameters=[config_file],
        ),
    ])
