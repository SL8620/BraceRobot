from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

from ament_index_python.packages import get_package_share_directory

import os
import xacro


def generate_launch_description():
    andino_desc_launch = os.path.join(
        get_package_share_directory('andino_description'),
        'launch',
        'andino_description.launch.py',
    )

    # === 1) 机器人模型与 robot_state_publisher ===
    # 保持 andino_description 的逻辑，由它来启动 robot_state_publisher
    desc_ld = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(andino_desc_launch)
    )

    # === 2) 底盘控制：等效于 andino_control.launch.py，但不再依赖 ros2 param get ===
    andino_control_share = get_package_share_directory('andino_control')
    controller_params_file = os.path.join(andino_control_share, 'config', 'andino_controllers.yaml')

    # 直接用 xacro 生成 robot_description
    andino_desc_share = get_package_share_directory('andino_description')
    xacro_file = os.path.join(andino_desc_share, 'urdf', 'andino.urdf.xacro')
    xacro_args = {'yaml_config_dir': os.path.join(andino_desc_share, 'config', 'andino')}
    doc = xacro.process_file(xacro_file, mappings=xacro_args)
    robot_desc = doc.toprettyxml(indent='  ')
    robot_description = ParameterValue(robot_desc, value_type=str)

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[{'robot_description': robot_description}, controller_params_file],
        remappings=[
            ('/diff_controller/cmd_vel', '/cmd_vel'),
            ('/diff_controller/cmd_vel_unstamped', '/cmd_vel'),
            ('/diff_controller/cmd_vel_out', '/cmd_vel_out'),
            ('/diff_controller/odom', '/odom'),
        ],
        output="both",
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )

    diff_drive_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["diff_controller", "--controller-manager", "/controller_manager"],
    )

    delay_diff_drive_controller_spawner_after_joint_state_broadcaster_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[diff_drive_controller_spawner],
        )
    )

    return LaunchDescription([
        # 1) 机器人模型与 robot_state_publisher（保留模型与 TF）
        desc_ld,

        # 底盘控制节点暂时不启动（仅使用机械臂 CAN）
        # control_node,
        # joint_state_broadcaster_spawner,
        # delay_diff_drive_controller_spawner_after_joint_state_broadcaster_spawner,

        # 电机控制节点（CAN）——使用 can0
        # 可执行名来自 kvaser_motor_control/CMakeLists.txt 中的 add_executable(motor_keyboard_node ...)
        Node(
            package='kvaser_motor_control',
            executable='motor_keyboard_node',
            name='motor_control_node',
            output='screen',
            parameters=[{'can_channel': 0}]
        ),
        # 电缸 Modbus 节点
        Node(
            package='lifter_modbus',
            executable='lifter_node',
            name='lifter_node',
            output='screen'
        ),
        # 集成状态机节点
        Node(
            package='brace_bot',
            executable='brace_state_node',
            name='brace_state_node',
            output='screen'
        ),
        # Python UI 总控台
        Node(
            package='brace_bot',
            executable='brace_ui_node.py',
            name='brace_ui',
            output='screen'
        ),
    ])
