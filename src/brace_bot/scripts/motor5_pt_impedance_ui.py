#!/usr/bin/env python3
import math
import threading

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter as RclpyParameter

from std_msgs.msg import Float64, Float64MultiArray, Int32

from rcl_interfaces.srv import GetParameters, SetParameters

try:
    import tkinter as tk
    from tkinter import ttk
except ImportError:
    tk = None
    ttk = None


class Motor5PtImpedanceUINode(Node):
    def __init__(self):
        super().__init__('motor5_pt_impedance_ui')

        self.target_node_name = self.declare_parameter('target_node', 'motor5_pt_impedance_node').value

        # Publishers for desired setpoints
        self.qd_pub = self.create_publisher(Float64, '/motor5/impedance_position_cmd', 10)
        self.qd_dot_pub = self.create_publisher(Float64, '/motor5/impedance_velocity_cmd', 10)
        self.enable_pub = self.create_publisher(Int32, '/motor5/enable_cmd', 10)

        # Subscriber for live state
        self.state_sub = self.create_subscription(
            Float64MultiArray, '/motor5/pt_impedance_state', self._state_cb, 10
        )

        # Remote parameter service clients (kp/kd)
        self._get_params_client = self.create_client(GetParameters, self._param_service_name('get_parameters'))
        self._set_params_client = self.create_client(SetParameters, self._param_service_name('set_parameters'))

        # Latest state (SI units: rad, rad/s, Nm)
        self._p = 0.0
        self._v = 0.0
        self._t = 0.0
        self._pd = 0.0
        self._vd = 0.0
        self._kp = 0.0
        self._kd = 0.0

        if tk is None:
            self.get_logger().error('tkinter not available, UI will not start.')
            return

        self.ui_thread = threading.Thread(target=self._init_ui, daemon=True)
        self.ui_thread.start()

        # Periodically refresh kp/kd from remote node (in case changed elsewhere)
        self.create_timer(0.5, self._refresh_params)

    def _param_service_name(self, service: str) -> str:
        name = str(self.target_node_name)
        if not name.startswith('/'):
            name = '/' + name
        return f'{name}/{service}'

    def _state_cb(self, msg: Float64MultiArray):
        if not msg.data or len(msg.data) < 3:
            return
        data = msg.data
        # [p, v, t, pd, vd, kp, kd]
        self._p = float(data[0])
        self._v = float(data[1])
        self._t = float(data[2])
        if len(data) >= 5:
            self._pd = float(data[3])
            self._vd = float(data[4])
        if len(data) >= 7:
            self._kp = float(data[5])
            self._kd = float(data[6])
        if len(data) >= 8:
            self._enabled = (float(data[7]) >= 0.5)

        if hasattr(self, 'root') and self.root:
            try:
                self.root.after(0, self._update_state_vars)
            except tk.TclError:
                pass

    def _refresh_params(self):
        if not self._get_params_client.service_is_ready():
            return

        req = GetParameters.Request()
        req.names = ['kp', 'kd']
        future = self._get_params_client.call_async(req)

        def _done(fut):
            try:
                resp = fut.result()
                values = resp.values if resp else None
                if values and len(values) >= 2:
                    self._kp = float(values[0].double_value)
                    self._kd = float(values[1].double_value)
                    if hasattr(self, 'root') and self.root:
                        try:
                            self.root.after(0, self._update_gain_vars)
                        except tk.TclError:
                            pass
            except Exception:
                return

        future.add_done_callback(_done)

    # ---------------- UI ----------------
    def _init_ui(self):
        self.root = tk.Tk()
        self.root.title('Motor5 PT Impedance Tuner')

        frm = ttk.Frame(self.root, padding=10)
        frm.grid(row=0, column=0, sticky='nsew')

        # Live state
        ttk.Label(frm, text='Live state (Motor 5)').grid(row=0, column=0, columnspan=4, sticky='w', pady=(0, 6))

        self.p_var = tk.StringVar(value='0.0')
        self.v_var = tk.StringVar(value='0.0')
        self.t_var = tk.StringVar(value='0.0')
        self.pd_var = tk.StringVar(value='0.0')
        self.vd_var = tk.StringVar(value='0.0')
        self.kp_var = tk.StringVar(value='0.0')
        self.kd_var = tk.StringVar(value='0.0')
        self.en_var = tk.StringVar(value='UNKNOWN')

        ttk.Label(frm, text='P (deg):').grid(row=1, column=0, sticky='e')
        ttk.Label(frm, textvariable=self.p_var, width=10).grid(row=1, column=1, sticky='w')
        ttk.Label(frm, text='Pd (deg):').grid(row=1, column=2, sticky='e')
        ttk.Label(frm, textvariable=self.pd_var, width=10).grid(row=1, column=3, sticky='w')

        ttk.Label(frm, text='V (deg/s):').grid(row=2, column=0, sticky='e')
        ttk.Label(frm, textvariable=self.v_var, width=10).grid(row=2, column=1, sticky='w')
        ttk.Label(frm, text='Vd (deg/s):').grid(row=2, column=2, sticky='e')
        ttk.Label(frm, textvariable=self.vd_var, width=10).grid(row=2, column=3, sticky='w')

        ttk.Label(frm, text='T (Nm):').grid(row=3, column=0, sticky='e')
        ttk.Label(frm, textvariable=self.t_var, width=10).grid(row=3, column=1, sticky='w')

        ttk.Label(frm, text='Enabled:').grid(row=3, column=2, sticky='e')
        ttk.Label(frm, textvariable=self.en_var, width=10).grid(row=3, column=3, sticky='w')

        ttk.Separator(frm, orient='horizontal').grid(row=4, column=0, columnspan=4, sticky='we', pady=8)

        # Commands
        ttk.Label(frm, text='Commands').grid(row=5, column=0, columnspan=4, sticky='w', pady=(0, 6))

        ttk.Button(frm, text='Enable', command=self._on_enable).grid(row=6, column=0, sticky='we')
        ttk.Button(frm, text='Disable', command=self._on_disable).grid(row=6, column=1, sticky='we')

        self.qd_entry_var = tk.StringVar(value='0.0')
        self.vd_entry_var = tk.StringVar(value='0.0')
        self.kp_entry_var = tk.StringVar(value='0.0')
        self.kd_entry_var = tk.StringVar(value='0.0')

        ttk.Label(frm, text='Desired P (deg):').grid(row=7, column=0, sticky='e')
        ttk.Entry(frm, textvariable=self.qd_entry_var, width=12).grid(row=7, column=1, sticky='w')

        ttk.Label(frm, text='Desired V (deg/s):').grid(row=7, column=2, sticky='e')
        ttk.Entry(frm, textvariable=self.vd_entry_var, width=12).grid(row=7, column=3, sticky='w')

        ttk.Label(frm, text='kp:').grid(row=8, column=0, sticky='e')
        ttk.Entry(frm, textvariable=self.kp_entry_var, width=12).grid(row=8, column=1, sticky='w')

        ttk.Label(frm, text='kd:').grid(row=8, column=2, sticky='e')
        ttk.Entry(frm, textvariable=self.kd_entry_var, width=12).grid(row=8, column=3, sticky='w')

        ttk.Button(frm, text='Apply All', command=self._on_apply_all).grid(
            row=9, column=0, columnspan=4, sticky='we', pady=(8, 0)
        )

        self.status_var = tk.StringVar(value='')
        ttk.Label(frm, textvariable=self.status_var).grid(row=10, column=0, columnspan=4, sticky='w', pady=(6, 0))

        self._update_state_vars()
        self._update_gain_vars()

        self.root.mainloop()

    def _update_state_vars(self):
        rad2deg = 180.0 / math.pi
        self.p_var.set(f'{self._p * rad2deg:.2f}')
        self.v_var.set(f'{self._v * rad2deg:.2f}')
        self.t_var.set(f'{self._t:.3f}')
        self.pd_var.set(f'{self._pd * rad2deg:.2f}')
        self.vd_var.set(f'{self._vd * rad2deg:.2f}')
        enabled = getattr(self, '_enabled', None)
        if enabled is True:
            self.en_var.set('YES')
        elif enabled is False:
            self.en_var.set('NO')
        else:
            self.en_var.set('UNKNOWN')

    def _update_gain_vars(self):
        self.kp_var.set(f'{self._kp:.3f}')
        self.kd_var.set(f'{self._kd:.3f}')

    def _on_apply_all(self):
        # Desired setpoints
        try:
            qd_deg = float(self.qd_entry_var.get())
            vd_deg = float(self.vd_entry_var.get())
        except ValueError:
            self.status_var.set('Invalid P/V input')
            return

        qd = qd_deg * math.pi / 180.0
        vd = vd_deg * math.pi / 180.0

        self.qd_pub.publish(Float64(data=qd))
        self.qd_dot_pub.publish(Float64(data=vd))

        # Gains via parameter service
        try:
            kp = float(self.kp_entry_var.get())
            kd = float(self.kd_entry_var.get())
        except ValueError:
            self.status_var.set('Invalid kp/kd input')
            return

        if not self._set_params_client.service_is_ready():
            self.status_var.set('Param service not ready')
            return

        p_kp = RclpyParameter('kp', RclpyParameter.Type.DOUBLE, kp).to_parameter_msg()
        p_kd = RclpyParameter('kd', RclpyParameter.Type.DOUBLE, kd).to_parameter_msg()
        req = SetParameters.Request()
        req.parameters = [p_kp, p_kd]
        future = self._set_params_client.call_async(req)

        def _done(_):
            try:
                self.status_var.set('Applied')
            except Exception:
                pass

        future.add_done_callback(_done)

    def _on_enable(self):
        self.enable_pub.publish(Int32(data=1))
        try:
            self.status_var.set('Enable sent')
        except Exception:
            pass

    def _on_disable(self):
        self.enable_pub.publish(Int32(data=0))
        try:
            self.status_var.set('Disable sent')
        except Exception:
            pass


def main(argv=None):
    rclpy.init(args=argv)
    node = Motor5PtImpedanceUINode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
