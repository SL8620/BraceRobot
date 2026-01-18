from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    teleop_node = Node(
        package='teleop_twist_keyboard',
        executable='teleop_twist_keyboard',
        name='teleop_twist_keyboard_node',
        output='screen',
        # 设置线速度和角速度的基准值
        # 对应按键持续按下时发布 /cmd_vel
        parameters=[
            {'speed': 0.1},   # 前后：0.1 m/s
            {'turn': 0.3},    # 左右：0.3 rad/s
        ],
    )

    return LaunchDescription([
        teleop_node,
    ])
