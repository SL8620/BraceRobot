#!/usr/bin/env python3

import threading
import tkinter as tk
from tkinter import ttk

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from std_msgs.msg import Int32MultiArray


class LifterUI(Node):
    def __init__(self):
        super().__init__('lifter_ui')

        self.cmd_pub = self.create_publisher(Int32MultiArray, '/lifter_cmd', 10)
        self.create_subscription(Float32, '/lifter_pos', self.pos_callback, 10)

        # 当前高度（mm）和期望高度（mm）
        self.current_height_mm = 0.0
        self.step_mm = 10.0  # 每次上升/下降的步长

        # Tkinter UI
        self.root = tk.Tk()
        self.root.title('Lifter UI')

        main_frame = ttk.Frame(self.root, padding='10')
        main_frame.grid(row=0, column=0, sticky=(tk.N, tk.S, tk.E, tk.W))

        # 当前高度显示
        ttk.Label(main_frame, text='Current height (mm):').grid(row=0, column=0, sticky=tk.W)
        self.height_var = tk.StringVar(value='0.0')
        self.height_label = ttk.Label(main_frame, textvariable=self.height_var, font=('Arial', 16))
        self.height_label.grid(row=0, column=1, sticky=tk.W)

        # 期望位置显示/输入
        ttk.Label(main_frame, text='Target height (mm):').grid(row=1, column=0, sticky=tk.W, pady=(10, 0))
        self.target_var = tk.StringVar(value='0.0')
        self.target_entry = ttk.Entry(main_frame, textvariable=self.target_var, width=10)
        self.target_entry.grid(row=1, column=1, sticky=tk.W, pady=(10, 0))

        # 按钮行
        btn_frame = ttk.Frame(main_frame, padding='10 20 0 0')
        btn_frame.grid(row=2, column=0, columnspan=2, sticky=(tk.W, tk.E))

        enable_btn = ttk.Button(btn_frame, text='Enable', command=self.send_enable)
        enable_btn.grid(row=0, column=0, padx=5)

        disable_btn = ttk.Button(btn_frame, text='Disable', command=self.send_disable)
        disable_btn.grid(row=0, column=1, padx=5)

        up_btn = ttk.Button(btn_frame, text='Up', command=self.send_up)
        up_btn.grid(row=0, column=2, padx=5)

        down_btn = ttk.Button(btn_frame, text='Down', command=self.send_down)
        down_btn.grid(row=0, column=3, padx=5)

        publish_btn = ttk.Button(btn_frame, text='Publish', command=self.send_publish)
        publish_btn.grid(row=0, column=4, padx=5)

        # 关闭窗口时，优雅关闭 ROS2
        self.root.protocol('WM_DELETE_WINDOW', self.on_close)

        # 使用 Tk 的定时器周期性调用 ROS 事件循环
        self.root.after(50, self.ros_spin_once)

    # ROS 回调：更新当前位置显示
    def pos_callback(self, msg: Float32):
        self.current_height_mm = float(msg.data)
        self.height_var.set(f'{self.current_height_mm:.1f}')

    # 发送命令辅助函数
    def publish_cmd(self, data):
        msg = Int32MultiArray()
        msg.data = [int(v) for v in data]
        self.cmd_pub.publish(msg)

    def send_enable(self):
        # type=1: enable
        self.get_logger().info('Send ENABLE command')
        self.publish_cmd([1])

    def send_disable(self):
        # type=2: disable
        self.get_logger().info('Send DISABLE command')
        self.publish_cmd([2])

    def send_up(self):
        """增加期望位置，但不立即发布，只更新期望位置框"""
        try:
            base = float(self.target_var.get())
        except ValueError:
            base = self.current_height_mm
        target = base + self.step_mm
        self.target_var.set(f'{target:.1f}')

    def send_down(self):
        """减小期望位置，但不立即发布，只更新期望位置框"""
        try:
            base = float(self.target_var.get())
        except ValueError:
            base = self.current_height_mm
        target = base - self.step_mm
        self.target_var.set(f'{target:.1f}')

    def send_publish(self):
        """根据期望位置框的值发布一次位置指令"""
        try:
            target = int(float(self.target_var.get()))
        except ValueError:
            self.get_logger().warn('Invalid target height, please input a number')
            return

        self.get_logger().info(f'Send POSITION command to {target} mm')
        # type=0: position, data[1] = 目标高度(mm)
        self.publish_cmd([0, target])

    def ros_spin_once(self):
        # 非阻塞处理一次 ROS 事件，然后重新安排下一次
        rclpy.spin_once(self, timeout_sec=0.01)
        self.root.after(50, self.ros_spin_once)

    def on_close(self):
        # 关闭窗口时停止节点并退出
        self.get_logger().info('Lifter UI shutting down')
        self.destroy_node()
        rclpy.shutdown()
        self.root.destroy()

    def run(self):
        self.root.mainloop()


def main():
    rclpy.init()
    node = LifterUI()
    node.run()


if __name__ == '__main__':
    main()
