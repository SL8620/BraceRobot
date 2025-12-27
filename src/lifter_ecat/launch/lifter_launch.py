from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    ld = LaunchDescription()

    lifter_node = Node(
        package='lifter_ecat',
        executable='lifter_node',
        name='lifter_node',
        output='screen',
        parameters=[{}],
        remappings=[]
    )

    ld.add_action(lifter_node)
    return ld
