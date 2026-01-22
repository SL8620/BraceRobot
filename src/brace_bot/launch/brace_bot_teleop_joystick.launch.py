import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    joystick_config = os.path.join(
        get_package_share_directory('andino_bringup'),
        'config',
        'joystick.yaml',
    )

    cmd_vel_topic_arg = DeclareLaunchArgument(
        'cmd_vel_topic',
        default_value='/cmd_vel',
        description='cmd_vel output topic for joystick teleop.',
    )
    cmd_vel_topic = LaunchConfiguration('cmd_vel_topic')

    joy_node = Node(
        package='joy_linux',
        executable='joy_linux_node',
        output='screen',
        parameters=[joystick_config],
    )

    teleop_node = Node(
        package='teleop_twist_joy',
        executable='teleop_node',
        name='teleop_node',
        output='screen',
        parameters=[joystick_config],
        remappings=[('/cmd_vel', cmd_vel_topic)],
    )

    return LaunchDescription([
        cmd_vel_topic_arg,
        joy_node,
        teleop_node,
    ])
