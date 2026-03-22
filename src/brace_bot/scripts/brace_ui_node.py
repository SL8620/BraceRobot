#!/usr/bin/env python3
import math
import threading

import rclpy
from rclpy.node import Node

from std_msgs.msg import Float32, Int32, Int32MultiArray
from sensor_msgs.msg import JointState

try:
    import tkinter as tk
    from tkinter import ttk
except ImportError:
    tk = None
    ttk = None


class BraceUINode(Node):
    """整体操控台 UI：电缸 / 电机 / 底盘 / 状态机。"""

    def __init__(self):
        super().__init__('brace_ui_node')

        # ---- ROS 通信 ----
        # 电缸
        self.lifter_cmd_pub = self.create_publisher(Int32MultiArray, '/lifter_cmd', 10)
        self.lifter_pos_sub = self.create_subscription(
            Float32, '/lifter_pos', self._lifter_pos_callback, 10
        )

        # 电机：控制 + 状态订阅（用于 UI 显示）
        self.motor_cmd_pub = self.create_publisher(JointState, '/motor/position_cmd', 10)
        # 订阅机械臂节点发布的实时关节状态（实际位置），而不是只看指令值
        self.motor_state_sub = self.create_subscription(
            JointState, '/motor/joint_states', self._motor_state_callback, 10
        )
        # 电机全局使能/失能命令
        self.motor_enable_pub = self.create_publisher(Int32, '/motor/enable_cmd', 10)
        # 单个电机零点设置：/motor/set_zero, data=1-6
        self.motor_zero_pub = self.create_publisher(Int32, '/motor/set_zero', 10)
        # 6 个电机当前目标角度（deg），用于 UI 显示
        self._motor_current_deg = [0.0] * 6

        # 底盘：发布 cmd_vel 到 andino 控制
        from geometry_msgs.msg import Twist
        self._twist_type = Twist
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # 状态机
        self.state_cmd_pub = self.create_publisher(Int32, '/brace_bot/state_cmd', 10)

        # 本地状态
        self._lifter_pos = 0.0

        if tk is None:
            self.get_logger().error('tkinter not available, UI will not start.')
            return

        # 在单独线程中运行 Tk 主循环，避免阻塞 rclpy spin
        self.ui_thread = threading.Thread(target=self._init_ui, daemon=True)
        self.ui_thread.start()

    # ----------------- 电缸回调 -----------------
    def _lifter_pos_callback(self, msg: Float32):
        self._lifter_pos = msg.data
        if hasattr(self, 'lifter_pos_var'):
            # 在 UI 线程安全更新
            def update():
                self.lifter_pos_var.set(f'{self._lifter_pos:.1f}')
                if hasattr(self, 'lifter_pos_bar'):
                    try:
                        self.lifter_pos_bar['value'] = self._lifter_pos
                    except tk.TclError:
                        pass
            if self.root:
                self.root.after(0, update)

    def _motor_state_callback(self, msg: JointState):
        """接收 /motor/position_cmd，更新 6 个电机在 UI 中显示的角度（deg）。"""
        if not msg.position:
            return

        rad2deg = 180.0 / math.pi
        n = min(len(msg.position), 6)
        for i in range(n):
            try:
                self._motor_current_deg[i] = msg.position[i] * rad2deg
            except (TypeError, IndexError):
                continue

        # 在 Tk 线程中更新文本
        if hasattr(self, 'root') and self.root and hasattr(self, 'motor_deg_vars'):
            def update():
                for i in range(6):
                    try:
                        self.motor_deg_vars[i].set(f"{self._motor_current_deg[i]:.1f}")
                    except tk.TclError:
                        pass

            try:
                self.root.after(0, update)
            except tk.TclError:
                pass

    # ----------------- UI 初始化 -----------------
    def _init_ui(self):
        self.root = tk.Tk()
        self.root.title('Brace Robot Control Panel')

        notebook = ttk.Notebook(self.root)
        notebook.pack(fill=tk.BOTH, expand=True)

        # 电缸 tab
        lifter_frame = ttk.Frame(notebook)
        notebook.add(lifter_frame, text='Lifter')
        self._build_lifter_ui(lifter_frame)

        # 电机 tab
        motor_frame = ttk.Frame(notebook)
        notebook.add(motor_frame, text='Motors')
        self._build_motor_ui(motor_frame)

        # 底盘 tab
        base_frame = ttk.Frame(notebook)
        notebook.add(base_frame, text='Base')
        self._build_base_ui(base_frame)

        # 整机状态机 tab
        sm_frame = ttk.Frame(notebook)
        notebook.add(sm_frame, text='State Machine')
        self._build_state_machine_ui(sm_frame)

        self.root.mainloop()

    # ----------------- 电缸 UI -----------------
    def _build_lifter_ui(self, frame):
        # 当前高度
        ttk.Label(frame, text='Current height (mm):').grid(row=0, column=0, sticky='w', padx=5, pady=5)
        self.lifter_pos_var = tk.StringVar(value='0.0')
        ttk.Label(frame, textvariable=self.lifter_pos_var, width=8).grid(row=0, column=1, sticky='w')
        self.lifter_pos_bar = ttk.Progressbar(frame, orient='horizontal', length=200, mode='determinate')
        self.lifter_pos_bar.grid(row=0, column=2, columnspan=2, padx=5, pady=5, sticky='we')
        self.lifter_pos_bar['maximum'] = 200.0  # 预估最大 200mm，可按需调整

        # 目标高度
        ttk.Label(frame, text='Target height (mm):').grid(row=1, column=0, sticky='w', padx=5, pady=5)
        self.lifter_target_var = tk.StringVar(value='0')
        ttk.Entry(frame, textvariable=self.lifter_target_var, width=8).grid(row=1, column=1, sticky='w')

        # 按钮：使能 / 失能 / 执行
        ttk.Button(frame, text='Enable', command=self._on_lifter_enable).grid(row=2, column=0, padx=5, pady=5)
        ttk.Button(frame, text='Disable', command=self._on_lifter_disable).grid(row=2, column=1, padx=5, pady=5)
        ttk.Button(frame, text='Move', command=self._on_lifter_move).grid(row=2, column=2, padx=5, pady=5)

    def _on_lifter_enable(self):
        msg = Int32MultiArray()
        msg.data = [1]  # type 1 = enable
        self.lifter_cmd_pub.publish(msg)
        self.get_logger().info('Lifter enable command sent')

    def _on_lifter_disable(self):
        msg = Int32MultiArray()
        msg.data = [2]  # type 2 = disable
        self.lifter_cmd_pub.publish(msg)
        self.get_logger().info('Lifter disable command sent')

    def _on_lifter_move(self):
        try:
            target = float(self.lifter_target_var.get())
        except ValueError:
            self.get_logger().warn('Invalid lifter target height')
            return
        msg = Int32MultiArray()
        msg.data = [0, int(round(target))]  # type 0 = position mode
        self.lifter_cmd_pub.publish(msg)
        self.get_logger().info(f'Lifter move command: {target} mm')

    # ----------------- 电机 UI -----------------
    def _build_motor_ui(self, frame):
        # 右臂：电机 1,3,5 ； 左臂：电机 2,4,6
        ttk.Label(frame, text='电机编号 1-6（右臂: 1,3,5；左臂: 2,4,6）').grid(
            row=0, column=0, columnspan=4, sticky='w', padx=5, pady=5
        )

        self.motor_deg_vars = [tk.StringVar(value='0.0') for _ in range(6)]
        self.motor_target_deg_vars = [tk.StringVar(value='0.0') for _ in range(6)]

        for i in range(6):
            motor_id = i + 1
            side = '右臂' if motor_id in (1, 3, 5) else '左臂'

            row = i + 1
            ttk.Label(frame, text=f'M{motor_id} ({side}) 当前(deg):').grid(
                row=row, column=0, sticky='w', padx=5, pady=2
            )
            ttk.Label(frame, textvariable=self.motor_deg_vars[i], width=8).grid(
                row=row, column=1, sticky='w', padx=5, pady=2
            )

            ttk.Label(frame, text='目标(deg):').grid(
                row=row, column=2, sticky='e', padx=5, pady=2
            )
            ttk.Entry(frame, textvariable=self.motor_target_deg_vars[i], width=8).grid(
                row=row, column=3, sticky='w', padx=5, pady=2
            )

        ttk.Button(frame, text='执行所有电机', command=self._on_motors_execute).grid(
            row=7, column=0, columnspan=2, padx=5, pady=10
        )

        # 使能 / 失能 按钮
        ttk.Button(frame, text='使能全部电机', command=self._on_motors_enable).grid(
            row=7, column=2, padx=5, pady=10
        )
        ttk.Button(frame, text='失能全部电机', command=self._on_motors_disable).grid(
            row=7, column=3, padx=5, pady=10
        )

        # 只为 M6 提供一个“将当前角度设为 0”按钮，避免误操作其他关节
        ttk.Button(frame, text='将 M6 当前设为 0°', command=self._on_motor6_zero).grid(
            row=8, column=0, columnspan=4, padx=5, pady=5, sticky='we'
        )

    def _on_motors_execute(self):
        # 读取 6 个电机的目标角度（deg），并按照电机编号施加限位：
        # 1,2: [-15, 0]；3,4: [0, 20]；5,6: [-20, 20]
        limits = [(-15.0, 0.0), (-15.0, 0.0), (0.0, 20.0), (0.0, 20.0), (-20.0, 20.0), (-20.0, 20.0)]

        targets_deg = []
        clamped = False
        for i in range(6):
            txt = self.motor_target_deg_vars[i].get()
            try:
                val = float(txt)
            except ValueError:
                self.get_logger().warn(f'Invalid target for motor {i+1}: "{txt}"')
                return

            low, high = limits[i]
            if val < low:
                val = low
                clamped = True
            elif val > high:
                val = high
                clamped = True

            # 更新输入框为限幅后的值
            self.motor_target_deg_vars[i].set(f"{val:.1f}")
            targets_deg.append(val)

        if clamped:
            self.get_logger().info('Some motor targets were clamped to their limits.')

        rad = math.pi / 180.0
        js = JointState()
        js.position = [d * rad for d in targets_deg]
        self.motor_cmd_pub.publish(js)
        self.get_logger().info('Motor command (deg): ' + ', '.join(f'{d:.1f}' for d in targets_deg))

    def _on_motors_enable(self):
        msg = Int32()
        msg.data = 1
        self.motor_enable_pub.publish(msg)
        self.get_logger().info('Enable all motors command sent')

    def _on_motors_disable(self):
        msg = Int32()
        msg.data = 0
        self.motor_enable_pub.publish(msg)
        self.get_logger().info('Disable all motors command sent')

    def _on_motor6_zero(self):
        """将当前物理位置作为 M6 的逻辑 0°。"""
        msg = Int32()
        msg.data = 6  # 电机 6
        self.motor_zero_pub.publish(msg)
        self.get_logger().info('Set current position of motor 6 as zero (via /motor/set_zero)')

    # ----------------- 底盘 UI -----------------
    def _build_base_ui(self, frame):
        ttk.Label(frame, text='Base control (cmd_vel):').grid(row=0, column=0, columnspan=3, sticky='w', padx=5, pady=5)
        ttk.Label(frame, text='按住按钮持续运动，松开自动停止').grid(row=4, column=0, columnspan=3, sticky='w', padx=5, pady=2)

        self._cmd_vel_active = None  # (linear_x, angular_z) or None
        self._cmd_vel_timer_id = None

        def make_btn(text, row, col, vx, wz):
            btn = ttk.Button(frame, text=text)
            btn.grid(row=row, column=col, padx=5, pady=5)
            btn.bind('<ButtonPress-1>', lambda e: self._start_cmd_vel(vx, wz))
            btn.bind('<ButtonRelease-1>', lambda e: self._stop_cmd_vel())
            return btn

        make_btn('Forward',  1, 1,  0.2,  0.0)
        make_btn('Backward', 3, 1, -0.2,  0.0)
        make_btn('Left',     2, 0,  0.0,  0.5)
        make_btn('Right',    2, 2,  0.0, -0.5)
        ttk.Button(frame, text='Stop', command=lambda: self._stop_cmd_vel()).grid(row=2, column=1, padx=5, pady=5)

    def _start_cmd_vel(self, linear_x: float, angular_z: float):
        self._cmd_vel_active = (linear_x, angular_z)
        self._publish_cmd_vel_tick()

    def _stop_cmd_vel(self):
        self._cmd_vel_active = None
        if self._cmd_vel_timer_id is not None:
            self.root.after_cancel(self._cmd_vel_timer_id)
            self._cmd_vel_timer_id = None
        # 发送零速确保停车
        twist = self._twist_type()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        self.cmd_vel_pub.publish(twist)

    def _publish_cmd_vel_tick(self):
        if self._cmd_vel_active is None:
            return
        vx, wz = self._cmd_vel_active
        twist = self._twist_type()
        twist.linear.x = vx
        twist.angular.z = wz
        self.cmd_vel_pub.publish(twist)
        # 每 100ms 重复发布（10Hz），保持 diff_drive_controller 不超时
        self._cmd_vel_timer_id = self.root.after(100, self._publish_cmd_vel_tick)

    def _send_cmd_vel(self, linear_x: float, angular_z: float):
        twist = self._twist_type()
        twist.linear.x = linear_x
        twist.angular.z = angular_z
        self.cmd_vel_pub.publish(twist)
        self.get_logger().info(f'cmd_vel: vx={linear_x}, wz={angular_z}')

    # ----------------- 状态机 UI -----------------
    def _build_state_machine_ui(self, frame):
        ttk.Label(frame, text='High-level state:').grid(row=0, column=0, sticky='w', padx=5, pady=5)

        self.state_var = tk.StringVar(value='IDLE')
        ttk.Label(frame, textvariable=self.state_var, width=10).grid(row=0, column=1, sticky='w')

        # 四个高层状态：idle -> low -> pick -> high -> pick -> low -> idle
        ttk.Button(frame, text='To IDLE', command=lambda: self._set_state(0)).grid(row=1, column=0, padx=5, pady=5)
        ttk.Button(frame, text='To LOW', command=lambda: self._set_state(1)).grid(row=1, column=1, padx=5, pady=5)
        ttk.Button(frame, text='To PICK', command=lambda: self._set_state(2)).grid(row=1, column=2, padx=5, pady=5)
        ttk.Button(frame, text='To HIGH', command=lambda: self._set_state(3)).grid(row=1, column=3, padx=5, pady=5)

        # 简单的本地记录当前状态，约束切换逻辑
        # 0=idle,1=low,2=pick,3=high
        self._state_int = 0

    def _set_state(self, target: int):
        # 约束仅允许按照 idle->low->pick->high->pick->low->idle 的路径切换
        cur = self._state_int
        valid = False

        if cur == target:
            valid = True
        elif cur == 0 and target == 1:  # idle -> low
            valid = True
        elif cur == 1 and target == 2:  # low -> pick
            valid = True
        elif cur == 2 and target == 3:  # pick -> high
            valid = True
        elif cur == 3 and target == 2:  # high -> pick
            valid = True
        elif cur == 2 and target == 1:  # pick -> low
            valid = True
        elif cur == 1 and target == 0:  # low -> idle
            valid = True

        if not valid:
            self.get_logger().warn('Invalid transition, must follow IDLE->LOW->PICK->HIGH->PICK->LOW->IDLE')
            return

        msg = Int32()
        msg.data = target
        self.state_cmd_pub.publish(msg)

        self._state_int = target
        names = {0: 'IDLE', 1: 'LOW', 2: 'PICK', 3: 'HIGH'}
        self.state_var.set(names.get(target, '?'))
        self.get_logger().info(f'State command: {self.state_var.get()}')


def main(args=None):
    rclpy.init(args=args)
    node = BraceUINode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
