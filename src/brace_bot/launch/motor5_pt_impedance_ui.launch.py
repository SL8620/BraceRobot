from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='brace_bot',
            executable='motor5_pt_impedance_ui.py',
            output='screen',
            parameters=[
                {'target_node': 'motor5_pt_impedance_node'},
            ],
        )
    ])
