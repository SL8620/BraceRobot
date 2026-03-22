#!/usr/bin/env python3
import math
import threading
from typing import List

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter as RclpyParameter

from rcl_interfaces.srv import GetParameters, SetParameters

from sensor_msgs.msg import JointState
from std_msgs.msg import Int32

try:
    import tkinter as tk
    from tkinter import ttk
except ImportError:
    tk = None
    ttk = None


class ArmPtImpedanceUINode(Node):
    NUM_MOTORS = 6

    def __init__(self):
        super().__init__('arm_pt_impedance_ui')

        self.target_node_name = self.declare_parameter('target_node', 'arm_pt_impedance_node').value

        # Pub/Sub
        self.cmd_pub = self.create_publisher(JointState, '/motor/impedance_position_cmd', 10)
        self.enable_all_pub = self.create_publisher(Int32, '/motor/enable_cmd', 10)
        self.enable_one_pub = self.create_publisher(Int32, '/motor/enable_one_cmd', 10)
        self.motor6_hw_zero_pub = self.create_publisher(Int32, '/motor6/set_zero_hw', 10)

        self.state_sub = self.create_subscription(JointState, '/motor/pt_impedance_state', self._state_cb, 10)

        # Remote parameter service clients
        self._get_params_client = self.create_client(GetParameters, self._param_service_name('get_parameters'))
        self._set_params_client = self.create_client(SetParameters, self._param_service_name('set_parameters'))

        # State
        self._p: List[float] = [0.0] * self.NUM_MOTORS
        self._v: List[float] = [0.0] * self.NUM_MOTORS
        self._t: List[float] = [0.0] * self.NUM_MOTORS

        self._kp: List[float] = [0.0] * self.NUM_MOTORS
        self._kd: List[float] = [0.0] * self.NUM_MOTORS
        self._enabled: List[bool] = [True] * self.NUM_MOTORS

        # Desired command buffers (rad, rad/s). Per-motor Apply updates these and publishes all 6.
        self._pd_cmd: List[float] = [0.0] * self.NUM_MOTORS
        self._vd_cmd: List[float] = [0.0] * self.NUM_MOTORS

        # UI editing flags (to avoid overwriting Entry while user types)
        self._editing_kp: List[bool] = [False] * self.NUM_MOTORS
        self._editing_kd: List[bool] = [False] * self.NUM_MOTORS

        if tk is None:
            self.get_logger().error('tkinter not available, UI will not start.')
            return

        # NOTE: Tkinter must run on the main thread. The UI is started from main().
        self.root = None

        # Periodically refresh arrays (kp/kd/enabled) from remote node
        self.create_timer(0.5, self._refresh_params)

    def start_ui(self):
        if tk is None:
            return
        self._init_ui()

    def _param_service_name(self, service: str) -> str:
        name = str(self.target_node_name)
        if not name.startswith('/'):
            name = '/' + name
        return f'{name}/{service}'

    def _state_cb(self, msg: JointState):
        # position/velocity/effort expected length >= 6
        if msg.position and len(msg.position) >= self.NUM_MOTORS:
            for i in range(self.NUM_MOTORS):
                self._p[i] = float(msg.position[i])
        if msg.velocity and len(msg.velocity) >= self.NUM_MOTORS:
            for i in range(self.NUM_MOTORS):
                self._v[i] = float(msg.velocity[i])
        if msg.effort and len(msg.effort) >= self.NUM_MOTORS:
            for i in range(self.NUM_MOTORS):
                self._t[i] = float(msg.effort[i])

        if hasattr(self, 'root') and self.root:
            try:
                self.root.after(0, self._update_state_vars)
            except tk.TclError:
                pass

    def _refresh_params(self):
        if not self._get_params_client.service_is_ready():
            return

        req = GetParameters.Request()
        req.names = ['kp', 'kd', 'enabled']
        fut = self._get_params_client.call_async(req)

        def _done(f):
            try:
                resp = f.result()
                if not resp or len(resp.values) < 3:
                    return

                # kp
                kp_vals = list(resp.values[0].double_array_value)
                kd_vals = list(resp.values[1].double_array_value)
                en_vals = list(resp.values[2].bool_array_value)

                if len(kp_vals) >= self.NUM_MOTORS:
                    self._kp = kp_vals[: self.NUM_MOTORS]
                if len(kd_vals) >= self.NUM_MOTORS:
                    self._kd = kd_vals[: self.NUM_MOTORS]
                if len(en_vals) >= self.NUM_MOTORS:
                    self._enabled = en_vals[: self.NUM_MOTORS]

                if hasattr(self, 'root') and self.root:
                    try:
                        self.root.after(0, self._update_gain_vars)
                        self.root.after(0, self._update_enabled_vars)
                    except tk.TclError:
                        pass
            except Exception:
                return

        fut.add_done_callback(_done)

    # ---------------- UI ----------------
    def _init_ui(self):
        self.root = tk.Tk()
        self.root.title('Arm PT Impedance Tuner (6 motors)')

        frm = ttk.Frame(self.root, padding=10)
        frm.grid(row=0, column=0, sticky='nsew')

        # Top controls
        ttk.Label(frm, text='Arm PT Impedance (6 motors)').grid(row=0, column=0, columnspan=12, sticky='w', pady=(0, 6))

        ttk.Button(frm, text='Enable ALL', command=self._on_enable_all).grid(row=1, column=0, sticky='we')
        ttk.Button(frm, text='Disable ALL', command=self._on_disable_all).grid(row=1, column=1, sticky='we')
        ttk.Button(frm, text='Send Setpoints', command=self._on_send_setpoints).grid(row=1, column=2, sticky='we')
        ttk.Button(frm, text='Apply Gains', command=self._on_apply_gains).grid(row=1, column=3, sticky='we')

        ttk.Separator(frm, orient='horizontal').grid(row=2, column=0, columnspan=12, sticky='we', pady=8)

        # Table headers
        headers = [
            'Motor',
            'P (deg)', 'V (deg/s)', 'T (Nm)',
            'Pd (deg)', 'Vd (deg/s)',
            'kp (cur)', 'kd (cur)',
            'kp (edit)', 'kd (edit)',
            'Enabled',
            'Actions'
        ]
        for c, h in enumerate(headers):
            ttk.Label(frm, text=h).grid(row=3, column=c, sticky='w', padx=(0, 8))

        # Row vars
        self.p_vars = [tk.StringVar(value='0.0') for _ in range(self.NUM_MOTORS)]
        self.v_vars = [tk.StringVar(value='0.0') for _ in range(self.NUM_MOTORS)]
        self.t_vars = [tk.StringVar(value='0.0') for _ in range(self.NUM_MOTORS)]

        self.pd_entry_vars = [tk.StringVar(value='0.0') for _ in range(self.NUM_MOTORS)]
        self.vd_entry_vars = [tk.StringVar(value='0.0') for _ in range(self.NUM_MOTORS)]
        self.kp_cur_vars = [tk.StringVar(value='0.000') for _ in range(self.NUM_MOTORS)]
        self.kd_cur_vars = [tk.StringVar(value='0.000') for _ in range(self.NUM_MOTORS)]

        self.kp_entry_vars = [tk.StringVar(value='0.000') for _ in range(self.NUM_MOTORS)]
        self.kd_entry_vars = [tk.StringVar(value='0.000') for _ in range(self.NUM_MOTORS)]

        self.en_vars = [tk.StringVar(value='YES') for _ in range(self.NUM_MOTORS)]

        start_row = 4

        self.kp_entries = [None] * self.NUM_MOTORS
        self.kd_entries = [None] * self.NUM_MOTORS

        for i in range(self.NUM_MOTORS):
            r = start_row + i
            ttk.Label(frm, text=str(i + 1)).grid(row=r, column=0, sticky='w')

            ttk.Label(frm, textvariable=self.p_vars[i], width=10).grid(row=r, column=1, sticky='w')
            ttk.Label(frm, textvariable=self.v_vars[i], width=10).grid(row=r, column=2, sticky='w')
            ttk.Label(frm, textvariable=self.t_vars[i], width=10).grid(row=r, column=3, sticky='w')

            ttk.Entry(frm, textvariable=self.pd_entry_vars[i], width=10).grid(row=r, column=4, sticky='w')
            ttk.Entry(frm, textvariable=self.vd_entry_vars[i], width=10).grid(row=r, column=5, sticky='w')

            ttk.Label(frm, textvariable=self.kp_cur_vars[i], width=8).grid(row=r, column=6, sticky='w')
            ttk.Label(frm, textvariable=self.kd_cur_vars[i], width=8).grid(row=r, column=7, sticky='w')

            self.kp_entries[i] = ttk.Entry(frm, textvariable=self.kp_entry_vars[i], width=8)
            self.kp_entries[i].grid(row=r, column=8, sticky='w')
            self.kp_entries[i].bind('<FocusIn>', lambda _e, idx=i: self._set_editing('kp', idx, True))
            self.kp_entries[i].bind('<FocusOut>', lambda _e, idx=i: self._set_editing('kp', idx, False))

            self.kd_entries[i] = ttk.Entry(frm, textvariable=self.kd_entry_vars[i], width=8)
            self.kd_entries[i].grid(row=r, column=9, sticky='w')
            self.kd_entries[i].bind('<FocusIn>', lambda _e, idx=i: self._set_editing('kd', idx, True))
            self.kd_entries[i].bind('<FocusOut>', lambda _e, idx=i: self._set_editing('kd', idx, False))

            ttk.Label(frm, textvariable=self.en_vars[i], width=8).grid(row=r, column=10, sticky='w')

            # Actions
            btn_frame = ttk.Frame(frm)
            btn_frame.grid(row=r, column=10, sticky='w')

            ttk.Button(btn_frame, text='Apply PV', command=lambda idx=i: self._apply_pv_one(idx)).grid(row=0, column=0)
            ttk.Button(btn_frame, text='Enable', command=lambda idx=i: self._enable_one(idx)).grid(row=0, column=1)
            ttk.Button(btn_frame, text='Disable', command=lambda idx=i: self._disable_one(idx)).grid(row=0, column=2)
            ttk.Button(btn_frame, text='Hold Here', command=lambda idx=i: self._hold_here(idx)).grid(row=0, column=3)

            if i == 5:
                ttk.Button(btn_frame, text='HW Zero', command=self._motor6_hw_zero).grid(row=0, column=4)

        ttk.Separator(frm, orient='horizontal').grid(row=start_row + self.NUM_MOTORS, column=0, columnspan=12, sticky='we', pady=8)

        self.status_var = tk.StringVar(value='')
        ttk.Label(frm, textvariable=self.status_var).grid(row=start_row + self.NUM_MOTORS + 1, column=0, columnspan=12, sticky='w')

        self._update_state_vars()
        self._update_gain_vars()
        self._update_enabled_vars()

        self.root.mainloop()

    def _update_state_vars(self):
        rad2deg = 180.0 / math.pi
        for i in range(self.NUM_MOTORS):
            self.p_vars[i].set(f'{self._p[i] * rad2deg:.2f}')
            self.v_vars[i].set(f'{self._v[i] * rad2deg:.2f}')
            self.t_vars[i].set(f'{self._t[i]:.3f}')

    def _update_gain_vars(self):
        for i in range(self.NUM_MOTORS):
            self.kp_cur_vars[i].set(f'{self._kp[i]:.3f}')
            self.kd_cur_vars[i].set(f'{self._kd[i]:.3f}')

    def _set_editing(self, field: str, idx: int, editing: bool):
        if idx < 0 or idx >= self.NUM_MOTORS:
            return
        if field == 'kp':
            self._editing_kp[idx] = bool(editing)
        elif field == 'kd':
            self._editing_kd[idx] = bool(editing)

    def _update_enabled_vars(self):
        for i in range(self.NUM_MOTORS):
            self.en_vars[i].set('YES' if bool(self._enabled[i]) else 'NO')

    # --------- Actions ---------
    def _on_enable_all(self):
        self.enable_all_pub.publish(Int32(data=1))
        self._set_status('Enable ALL sent')

    def _on_disable_all(self):
        self.enable_all_pub.publish(Int32(data=0))
        self._set_status('Disable ALL sent')

    def _on_send_setpoints(self):
        # Publish JointState with desired arrays (rad, rad/s)
        pd = [0.0] * self.NUM_MOTORS
        vd = [0.0] * self.NUM_MOTORS
        try:
            for i in range(self.NUM_MOTORS):
                pd_deg = float(self.pd_entry_vars[i].get())
                vd_deg = float(self.vd_entry_vars[i].get())
                pd[i] = pd_deg * math.pi / 180.0
                vd[i] = vd_deg * math.pi / 180.0
        except ValueError:
            self._set_status('Invalid Pd/Vd input')
            return

        self._pd_cmd = list(pd)
        self._vd_cmd = list(vd)

        msg = JointState()
        msg.position = pd
        msg.velocity = vd
        self.cmd_pub.publish(msg)
        self._set_status('Setpoints published')

    def _apply_pv_one(self, idx: int):
        if idx < 0 or idx >= self.NUM_MOTORS:
            return

        try:
            pd_deg = float(self.pd_entry_vars[idx].get())
            vd_deg = float(self.vd_entry_vars[idx].get())
        except ValueError:
            self._set_status(f'Invalid Pd/Vd for motor {idx + 1}')
            return

        self._pd_cmd[idx] = pd_deg * math.pi / 180.0
        self._vd_cmd[idx] = vd_deg * math.pi / 180.0

        msg = JointState()
        msg.position = list(self._pd_cmd)
        msg.velocity = list(self._vd_cmd)
        self.cmd_pub.publish(msg)
        self._set_status(f'Motor {idx + 1} PV applied')

    def _on_apply_gains(self):
        kp = [0.0] * self.NUM_MOTORS
        kd = [0.0] * self.NUM_MOTORS
        try:
            for i in range(self.NUM_MOTORS):
                kp[i] = float(self.kp_entry_vars[i].get())
                kd[i] = float(self.kd_entry_vars[i].get())
        except ValueError:
            self._set_status('Invalid kp/kd input')
            return

        if not self._set_params_client.service_is_ready():
            self._set_status('Param service not ready')
            return

        p_kp = RclpyParameter('kp', RclpyParameter.Type.DOUBLE_ARRAY, kp).to_parameter_msg()
        p_kd = RclpyParameter('kd', RclpyParameter.Type.DOUBLE_ARRAY, kd).to_parameter_msg()

        req = SetParameters.Request()
        req.parameters = [p_kp, p_kd]
        fut = self._set_params_client.call_async(req)

        def _done(_):
            try:
                resp = fut.result()
                if not resp or not resp.results:
                    self._set_status('Gains apply: no response')
                    return
                ok = all(r.successful for r in resp.results)
                if ok:
                    self._set_status('Gains applied')
                else:
                    reasons = [r.reason for r in resp.results if not r.successful and r.reason]
                    msg = 'Gains apply failed'
                    if reasons:
                        msg += ': ' + '; '.join(reasons[:2])
                    self._set_status(msg)
            except Exception:
                pass

        fut.add_done_callback(_done)

    def _set_enabled(self, idx: int, value: bool):
        if idx < 0 or idx >= self.NUM_MOTORS:
            return
        if not self._set_params_client.service_is_ready():
            self._set_status('Param service not ready')
            return

        enabled = list(self._enabled)
        enabled[idx] = bool(value)

        p_en = RclpyParameter('enabled', RclpyParameter.Type.BOOL_ARRAY, enabled).to_parameter_msg()
        req = SetParameters.Request()
        req.parameters = [p_en]
        fut = self._set_params_client.call_async(req)

        def _done(_):
            try:
                self._enabled[idx] = bool(value)
                if hasattr(self, 'root') and self.root:
                    self.root.after(0, self._update_enabled_vars)
                self._set_status(f'Motor {idx + 1} enabled={value}')
            except Exception:
                pass

        fut.add_done_callback(_done)

    def _enable_one(self, idx: int):
        motor_id = idx + 1
        # Drive-level enable + parameter-level torque gating
        self.enable_one_pub.publish(Int32(data=motor_id))
        self._set_enabled(idx, True)

    def _disable_one(self, idx: int):
        motor_id = idx + 1
        self.enable_one_pub.publish(Int32(data=-motor_id))
        self._set_enabled(idx, False)

    def _hold_here(self, idx: int):
        # Convenience: set Pd entry to current P, and Vd to 0, then publish.
        if idx < 0 or idx >= self.NUM_MOTORS:
            return

        rad2deg = 180.0 / math.pi
        self.pd_entry_vars[idx].set(f'{self._p[idx] * rad2deg:.2f}')
        self.vd_entry_vars[idx].set('0.0')
        self._apply_pv_one(idx)

    def _motor6_hw_zero(self):
        self.motor6_hw_zero_pub.publish(Int32(data=1))
        self._set_status('Motor 6 HW Zero sent')

    def _set_status(self, text: str):
        if hasattr(self, 'status_var'):
            try:
                self.status_var.set(text)
            except Exception:
                pass


def main():
    rclpy.init()
    node = ArmPtImpedanceUINode()

    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    try:
        node.start_ui()
    except KeyboardInterrupt:
        pass
    finally:
        try:
            if hasattr(node, 'root') and node.root:
                node.root.quit()
        except Exception:
            pass

        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
