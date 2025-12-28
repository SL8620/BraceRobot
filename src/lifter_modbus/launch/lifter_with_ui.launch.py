from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription([
        Node(
            package='lifter_modbus',
            executable='lifter_node',
            name='lifter_node',
            output='screen',
        ),
        Node(
            package='lifter_modbus',
            executable='lifter_ui.py',
            name='lifter_ui',
            output='screen',
        ),
    ])
