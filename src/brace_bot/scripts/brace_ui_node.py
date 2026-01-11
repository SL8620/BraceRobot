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

        # 电机
        self.motor_cmd_pub = self.create_publisher(JointState, '/motor/position_cmd', 10)

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
        ttk.Label(frame, text='Motor1 (deg):').grid(row=0, column=0, sticky='w', padx=5, pady=5)
        self.motor1_deg_var = tk.StringVar(value='0.0')
        ttk.Entry(frame, textvariable=self.motor1_deg_var, width=8).grid(row=0, column=1, sticky='w')

        ttk.Label(frame, text='Motor2 (deg):').grid(row=1, column=0, sticky='w', padx=5, pady=5)
        self.motor2_deg_var = tk.StringVar(value='0.0')
        ttk.Entry(frame, textvariable=self.motor2_deg_var, width=8).grid(row=1, column=1, sticky='w')

        ttk.Button(frame, text='Execute', command=self._on_motors_execute).grid(row=2, column=0, columnspan=2, padx=5, pady=10)

    def _on_motors_execute(self):
        try:
            m1 = float(self.motor1_deg_var.get())
            m2 = float(self.motor2_deg_var.get())
        except ValueError:
            self.get_logger().warn('Invalid motor target degrees')
            return
        rad = math.pi / 180.0
        js = JointState()
        js.position = [m1 * rad, m2 * rad]
        self.motor_cmd_pub.publish(js)
        self.get_logger().info(f'Motor command: [{m1}, {m2}] deg')

    # ----------------- 底盘 UI -----------------
    def _build_base_ui(self, frame):
        ttk.Label(frame, text='Base control (cmd_vel):').grid(row=0, column=0, columnspan=3, sticky='w', padx=5, pady=5)

        ttk.Button(frame, text='Forward', command=lambda: self._send_cmd_vel(0.2, 0.0)).grid(row=1, column=1, padx=5, pady=5)
        ttk.Button(frame, text='Backward', command=lambda: self._send_cmd_vel(-0.2, 0.0)).grid(row=3, column=1, padx=5, pady=5)
        ttk.Button(frame, text='Left', command=lambda: self._send_cmd_vel(0.0, 0.5)).grid(row=2, column=0, padx=5, pady=5)
        ttk.Button(frame, text='Right', command=lambda: self._send_cmd_vel(0.0, -0.5)).grid(row=2, column=2, padx=5, pady=5)
        ttk.Button(frame, text='Stop', command=lambda: self._send_cmd_vel(0.0, 0.0)).grid(row=2, column=1, padx=5, pady=5)

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

        # 这里简单使用按钮按照 idle -> lift -> down -> idle 的顺序切换
        ttk.Button(frame, text='To IDLE', command=lambda: self._set_state(0)).grid(row=1, column=0, padx=5, pady=5)
        ttk.Button(frame, text='To LIFT', command=lambda: self._set_state(1)).grid(row=1, column=1, padx=5, pady=5)
        ttk.Button(frame, text='To DOWN', command=lambda: self._set_state(2)).grid(row=1, column=2, padx=5, pady=5)

        # 简单的本地记录当前状态，约束切换逻辑
        self._state_int = 0  # 0=idle,1=lift,2=down

    def _set_state(self, target: int):
        # 约束仅允许 idle->lift->down->idle 循环
        if self._state_int == 0 and target == 1:
            pass
        elif self._state_int == 1 and target == 2:
            pass
        elif self._state_int == 2 and target == 0:
            pass
        elif target == self._state_int:
            pass
        else:
            self.get_logger().warn('Invalid transition, must follow IDLE->LIFT->DOWN->IDLE')
            return

        msg = Int32()
        msg.data = target
        self.state_cmd_pub.publish(msg)

        self._state_int = target
        names = {0: 'IDLE', 1: 'LIFT', 2: 'DOWN'}
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
