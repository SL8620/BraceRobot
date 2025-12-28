from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription([
        Node(
            package='lifter_modbus',
            executable='lifter_test',
            name='lifter_test',
            output='screen',
        ),
    ])
