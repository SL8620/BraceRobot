from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, RegisterEventHandler, DeclareLaunchArgument, TimerAction, ExecuteProcess
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import UnlessCondition
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

from ament_index_python.packages import get_package_share_directory, get_package_prefix

import os
import xacro


def generate_launch_description():
    ui_only = LaunchConfiguration('ui_only')

    ui_only_arg = DeclareLaunchArgument(
        'ui_only',
        default_value='false',
        description='If true, only launch UI without hardware/action nodes.',
    )
    # 默认关掉，由导航 launch 启动自己的 robot_state_publisher，避免重复 TF/robot_description
    rsp_arg = DeclareLaunchArgument(
        'rsp',
        default_value='false',
        description='Run robot_state_publisher from andino_description. Set true only when running bringup without nav.',
    )
    rsp = LaunchConfiguration('rsp')

    andino_desc_launch = os.path.join(
        get_package_share_directory('andino_description'),
        'launch',
        'andino_description.launch.py',
    )

    # === 1) 机器人模型（默认不启 robot_state_publisher，由导航 launch 负责）===
    desc_ld = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(andino_desc_launch),
        launch_arguments={'rsp': rsp}.items(),
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

    # 将 /cmd_vel 转发到 /cmd_vel_to_base 供底盘使用（Nav2 前进 = 底盘前进，不取负）
    _pkg_prefix = get_package_prefix("brace_bot")
    _cmd_vel_negate_script = os.path.join(_pkg_prefix, "lib", "brace_bot", "cmd_vel_linear_negate_node.py")
    cmd_vel_negate_node = ExecuteProcess(
        cmd=["python3", _cmd_vel_negate_script],
        name="cmd_vel_linear_negate",
        output="screen",
        shell=False,
        condition=UnlessCondition(ui_only),
    )

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[{'robot_description': robot_description}, controller_params_file],
        remappings=[
            ('/diff_controller/cmd_vel', '/cmd_vel_to_base'),
            ('/diff_controller/cmd_vel_unstamped', '/cmd_vel_to_base'),
            ('/diff_controller/cmd_vel_out', '/cmd_vel_out'),
            # 不把底盘 odom 映射到 /odom：原则全部使用视觉里程计（轮子打滑严重），轮式 odom 仅保留在 /diff_controller/odom
        ],
        output="both",
        condition=UnlessCondition(ui_only),
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
        condition=UnlessCondition(ui_only),
    )

    diff_drive_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["diff_controller", "--controller-manager", "/controller_manager"],
        condition=UnlessCondition(ui_only),
    )

    delay_diff_drive_controller_spawner_after_joint_state_broadcaster_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[diff_drive_controller_spawner],
        )
    )

    # 底盘与手臂共用 can0 时，若手臂节点与底盘同时启动，会导致底盘电机 8 先使能后掉使能。
    # 与 andino_robot.launch 一致：先让底盘独占 CAN 完成配置（约 5–6s），再启动使用 CAN 的节点。
    chassis_first_delay_s = 6.0
    delayed_can_nodes = TimerAction(
        period=chassis_first_delay_s,
        actions=[
            Node(
                package='kvaser_motor_control',
                executable='motor_keyboard_node',
                name='motor_control_node',
                output='screen',
                parameters=[{'can_channel': 0}],
                condition=UnlessCondition(ui_only),
            ),
            Node(
                package='lifter_modbus',
                executable='lifter_node',
                name='lifter_node',
                output='screen',
                condition=UnlessCondition(ui_only),
            ),
            Node(
                package='brace_bot',
                executable='brace_state_node',
                name='brace_state_node',
                output='screen',
                condition=UnlessCondition(ui_only),
            ),
        ],
    )

    return LaunchDescription([
        ui_only_arg,
        rsp_arg,

        # 1) 机器人模型与 robot_state_publisher（rsp:=false 时不启，避免与导航冲突）
        desc_ld,

        # cmd_vel 转发节点（/cmd_vel -> /cmd_vel_to_base）
        cmd_vel_negate_node,
        # 底盘控制节点（ros2_control + diff_drive，订阅 /cmd_vel_to_base）
        control_node,
        joint_state_broadcaster_spawner,
        delay_diff_drive_controller_spawner_after_joint_state_broadcaster_spawner,

        # 延迟启动：电机/电缸/状态机（使用 CAN 或可能与底盘争用），避免底盘电机 8 掉使能
        delayed_can_nodes,

        # Python UI 总控台（不占 CAN，可与底盘同时启）
        Node(
            package='brace_bot',
            executable='brace_ui_node.py',
            name='brace_ui',
            output='screen'
        ),
    ])
