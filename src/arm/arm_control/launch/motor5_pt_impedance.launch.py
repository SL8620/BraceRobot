import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('kvaser_motor_control')
    params_file = os.path.join(pkg_share, 'config', 'motor5_pt_impedance.yaml')

    return LaunchDescription([
        Node(
            package='kvaser_motor_control',
            executable='motor5_pt_impedance_node',
            name='motor5_pt_impedance_node',
            output='screen',
            parameters=[params_file],
        )
    ])
