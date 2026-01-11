from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='kvaser_motor_control',
            executable='motor_keyboard_node',
            output='screen'
        )
    ])
