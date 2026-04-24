import tkinter as tk
from tkinter import ttk, messagebox
import struct
import time
import threading
from collections import deque

import can

import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.figure import Figure


# -----------------------------
# RMD-X Protocol Commands
# -----------------------------
CMD_READ_STATUS_2   = 0x9C   # temp, iq, speed, angle(int16, deg/LSB)
CMD_READ_MT_ANGLE   = 0x92   # (your firmware) multi-turn absolute angle(int32, 0.01deg/LSB) in data[1..4]
CMD_SET_ZERO        = 0x19
CMD_BRAKE_RELEASE   = 0x77
CMD_BRAKE_LOCK      = 0x78
CMD_STOP            = 0x81
CMD_SHUTDOWN        = 0x80
CMD_TORQUE_CTRL     = 0xA1   # iqControl int16, 0.01A/LSB in data[4..5]
CMD_POS_CTRL_A4     = 0xA4   # multi-turn position int32, 0.01deg/LSB in data[4..7], speed limit uint16 in data[2..3]


# -----------------------------
# CAN Driver
# -----------------------------
class MotorDriver:
    def __init__(self, channel='can0', bitrate=1000000):
        self.lock = threading.Lock()
        self.bus = None
        try:
            self.bus = can.interface.Bus(channel=channel, interface='socketcan', bitrate=bitrate)
            print(f"[OK] CAN interface opened: {channel} @ {bitrate}")
        except Exception as e:
            print(f"[ERR] CAN init failed: {e}")

    @staticmethod
    def tx_id(mid: int) -> int:
        return 0x140 + mid

    @staticmethod
    def rx_id(mid: int) -> int:
        # Many RMD variants reply with 0x240 + id
        return 0x240 + mid

    def _flush(self):
        if self.bus is None:
            return
        while True:
            m = self.bus.recv(timeout=0)
            if m is None:
                break

    def _send(self, mid: int, data8: bytes) -> bool:
        if self.bus is None:
            return False
        msg = can.Message(arbitration_id=self.tx_id(mid), data=data8, is_extended_id=False)
        try:
            self.bus.send(msg)
            return True
        except can.CanError as e:
            print(f"[ERR] send failed: {e}")
            return False

    def _recv_expected(self, mid: int, timeout=0.02):
        """Wait one frame that matches rx_id(mid) OR tx_id(mid) (some adapters echo)."""
        if self.bus is None:
            return None
        t0 = time.time()
        while time.time() - t0 < timeout:
            m = self.bus.recv(timeout=timeout)
            if m is None:
                return None
            if m.arbitration_id in (self.rx_id(mid), self.tx_id(mid)):
                return m
        return None

    # ------------ Parse helpers ------------
    @staticmethod
    def parse_status_like(msg):
        """
        For replies of 0x9C / 0xA1 / 0xA4:
          data[1]  : temp int8 (1C/LSB)
          data[2:4]: iq int16 (0.01A/LSB)
          data[4:6]: speed int16 (1 dps/LSB)
          data[6:8]: angle int16 (1 deg/LSB)  <-- NOT multi-turn
        """
        if msg is None or len(msg.data) != 8:
            return None
        temp = struct.unpack('<b', bytes([msg.data[1]]))[0]
        iq_raw = struct.unpack('<h', bytes(msg.data[2:4]))[0]
        speed = struct.unpack('<h', bytes(msg.data[4:6]))[0]
        ang_i16 = struct.unpack('<h', bytes(msg.data[6:8]))[0]
        return {
            "temp_c": temp,
            "iq_a": iq_raw * 0.01,
            "speed_dps": float(speed),
            "angle_deg_i16": float(ang_i16),   # fast, coarse; not multi-turn
        }

    @staticmethod
    def parse_multiturn_92(msg):
        """
        For reply of 0x92 (YOUR observed layout):
          data[1:5] int32 motorAngle, 0.01deg/LSB (absolute multi-turn)
        NOTE: keep [1:5] as requested (firmware/protocol variance).
        """
        if msg is None or len(msg.data) != 8:
            return None
        if msg.data[0] != CMD_READ_MT_ANGLE:
            return None
        ang_i32 = struct.unpack('<i', bytes(msg.data[1:5]))[0]
        return ang_i32 * 0.01

    # ------------ Basic commands ------------
    def brake_release(self, mid: int):
        with self.lock:
            self._flush()
            self._send(mid, bytes([CMD_BRAKE_RELEASE, 0,0,0,0,0,0,0]))
            return self._recv_expected(mid, timeout=0.05)

    def brake_lock(self, mid: int):
        with self.lock:
            self._flush()
            self._send(mid, bytes([CMD_BRAKE_LOCK, 0,0,0,0,0,0,0]))
            return self._recv_expected(mid, timeout=0.05)

    def set_zero(self, mid: int):
        with self.lock:
            self._flush()
            self._send(mid, bytes([CMD_SET_ZERO, 0,0,0,0,0,0,0]))
            return self._recv_expected(mid, timeout=0.05)

    def stop(self, mid: int):
        with self.lock:
            self._flush()
            self._send(mid, bytes([CMD_STOP, 0,0,0,0,0,0,0]))
            return self._recv_expected(mid, timeout=0.05)

    def shutdown(self, mid: int):
        with self.lock:
            self._flush()
            self._send(mid, bytes([CMD_SHUTDOWN, 0,0,0,0,0,0,0]))
            return self._recv_expected(mid, timeout=0.05)

    def read_status_9c(self, mid: int):
        with self.lock:
            self._flush()
            self._send(mid, bytes([CMD_READ_STATUS_2, 0,0,0,0,0,0,0]))
            msg = self._recv_expected(mid, timeout=0.05)
            return msg

    def read_multiturn_angle_92(self, mid: int):
        with self.lock:
            self._flush()
            self._send(mid, bytes([CMD_READ_MT_ANGLE, 0,0,0,0,0,0,0]))
            msg = self._recv_expected(mid, timeout=0.05)
            return msg

    # ------------ Control commands ------------
    def torque_a1(self, mid: int, current_a: float, want_reply=True):
        """
        current_a -> iqControl (int16, 0.01A/LSB) in data[4..5]
        """
        with self.lock:
            iq = int(round(current_a / 0.01))
            iq = max(-32000, min(32000, iq))
            payload = bytearray([CMD_TORQUE_CTRL, 0,0,0])
            payload.extend(struct.pack('<h', iq))
            payload.extend([0,0])
            self._flush()
            self._send(mid, bytes(payload))
            if not want_reply:
                return None
            msg = self._recv_expected(mid, timeout=0.02)
            return msg

    def position_a4(self, mid: int, target_deg: float, speed_limit_dps: int = 360):
        """
        0xA4: DATA[2..3]=maxSpeed (uint16, 1 dps/LSB)
              DATA[4..7]=angleControl (int32, 0.01deg/LSB)
        """
        with self.lock:
            spd = int(max(0, min(30000, speed_limit_dps)))
            pos = int(round(target_deg * 100.0))
            payload = bytearray([CMD_POS_CTRL_A4, 0x00])
            payload.extend(struct.pack('<H', spd))
            payload.extend(struct.pack('<i', pos))
            self._flush()
            self._send(mid, bytes(payload))
            msg = self._recv_expected(mid, timeout=0.02)
            return msg

    # ------------ ID Scan ------------
    def scan_ids(self, start=1, end=32):
        """
        Send 0x9C to each ID and list responders.
        """
        found = []
        with self.lock:
            self._flush()
            for mid in range(start, end+1):
                self._send(mid, bytes([CMD_READ_STATUS_2,0,0,0,0,0,0,0]))
                msg = self._recv_expected(mid, timeout=0.01)
                if msg and msg.arbitration_id in (self.rx_id(mid), self.tx_id(mid)):
                    found.append(mid)
            self._flush()
        return found


# -----------------------------
# GUI App
# -----------------------------
class App(tk.Tk):
    def __init__(self, driver: MotorDriver):
        super().__init__()
        self.driver = driver

        self.title("RMD Motor Lab (MT Angle Priority + Safe E-Stop + Impedance)")
        self.geometry("980x920")

        # realtime data buffers
        self.data_len = 250
        self.angle_data = deque([0.0]*self.data_len, maxlen=self.data_len)
        self.current_data = deque([0.0]*self.data_len, maxlen=self.data_len)
        self.speed_data = deque([0.0]*self.data_len, maxlen=self.data_len)

        # state
        self.monitoring = True
        self.sending_torque = False
        self.sending_impedance = False

        # last known values
        self.last_fast_angle_deg = 0.0
        self.last_mt_angle_deg = 0.0

        # ---------------- UI ----------------
        top = tk.Frame(self)
        top.pack(fill="x", padx=10, pady=5)

        tk.Label(top, text="Motor ID:").pack(side="left")
        self.ent_mid = tk.Entry(top, width=6)
        self.ent_mid.insert(0, "1")
        self.ent_mid.pack(side="left", padx=6)

        tk.Button(top, text="SCAN (1~32)", command=self.scan_ids).pack(side="left", padx=4)
        self.lbl_scan = tk.Label(top, text="Found: []")
        self.lbl_scan.pack(side="left", padx=10)

        tk.Button(top, text="BRAKE RELEASE (0x77)", bg="lightgreen", command=self.brake_release).pack(side="left", padx=4)
        tk.Button(top, text="SET ZERO (0x19)", bg="orange", command=self.set_zero).pack(side="left", padx=4)

        # Status
        st = tk.LabelFrame(self, text="Real-time Status (MT priority)")
        st.pack(fill="x", padx=10, pady=6)
        self.lbl_status = tk.Label(st, text="--", font=("Consolas", 12, "bold"))
        self.lbl_status.pack(pady=6)

        # Plot
        plotf = tk.LabelFrame(self, text="Graph (AngleMT, Current)")
        plotf.pack(fill="both", expand=True, padx=10, pady=6)

        self.fig = Figure(figsize=(6,4), dpi=100)
        self.ax1 = self.fig.add_subplot(111)
        self.line_ang, = self.ax1.plot([], [], label="AngleMT (deg)")
        self.ax2 = self.ax1.twinx()
        self.line_iq, = self.ax2.plot([], [], label="Current (A)", alpha=0.6)
        self.ax1.set_xlabel("t")
        self.ax1.set_ylabel("AngleMT (deg)")
        self.ax2.set_ylabel("Current (A)")
        self.ax1.grid(True)

        self.canvas = FigureCanvasTkAgg(self.fig, master=plotf)
        self.canvas.draw()
        self.canvas.get_tk_widget().pack(fill="both", expand=True)

        # Torque control
        tc = tk.LabelFrame(self, text="Torque Control (0xA1) + Safety (MT angle)")
        tc.pack(fill="x", padx=10, pady=8)

        limf = tk.Frame(tc)
        limf.pack(fill="x", pady=4)
        tk.Label(limf, text="Safety |AngleMT| limit (deg):").pack(side="left")
        self.ent_limit = tk.Entry(limf, width=8)
        self.ent_limit.insert(0, "45")
        self.ent_limit.pack(side="left", padx=6)

        self.var_torque = tk.DoubleVar(value=0.0)
        self.scale_torque = tk.Scale(tc, variable=self.var_torque, from_=-5.0, to=5.0,
                                     resolution=0.01, orient="horizontal", length=520,
                                     label="Torque Current (A)")
        self.scale_torque.pack(pady=4)

        self.btn_torque = tk.Button(tc, text="Torque Loop OFF", bg="red", fg="white",
                                    font=("Arial", 11, "bold"), command=self.toggle_torque)
        self.btn_torque.pack(pady=6)

        # "torque-position" (impedance) control
        imp = tk.LabelFrame(self, text="Torque-Position (Impedance PD via 0xA1, MT priority)")
        imp.pack(fill="x", padx=10, pady=8)

        row = tk.Frame(imp); row.pack(fill="x", pady=3)
        tk.Label(row, text="Target angle (deg):").pack(side="left")
        self.ent_imp_target = tk.Entry(row, width=10)
        self.ent_imp_target.insert(0, "0.0")
        self.ent_imp_target.pack(side="left", padx=6)

        tk.Label(row, text="Kp (A/deg):").pack(side="left")
        self.ent_kp = tk.Entry(row, width=8); self.ent_kp.insert(0, "0.05"); self.ent_kp.pack(side="left", padx=6)

        tk.Label(row, text="Kd (A/(deg/s)) :").pack(side="left")
        self.ent_kd = tk.Entry(row, width=8); self.ent_kd.insert(0, "0.001"); self.ent_kd.pack(side="left", padx=6)

        tk.Label(row, text="I limit (A):").pack(side="left")
        self.ent_ilim = tk.Entry(row, width=8); self.ent_ilim.insert(0, "3.0"); self.ent_ilim.pack(side="left", padx=6)

        self.btn_imp = tk.Button(imp, text="Impedance OFF", bg="gray20", fg="white",
                                 font=("Arial", 11, "bold"), command=self.toggle_impedance)
        self.btn_imp.pack(pady=6)

        # Position A4
        pf = tk.LabelFrame(self, text="Position Control (0xA4, multi-turn, 0.01deg)")
        pf.pack(fill="x", padx=10, pady=8)

        self.scale_pos = tk.Scale(pf, from_=-6480, to=6480, orient="horizontal",
                                  length=520, resolution=1.0, label="Target (deg)")
        self.scale_pos.pack(pady=4)

        row2 = tk.Frame(pf); row2.pack(pady=3)
        tk.Label(row2, text="Speed limit (dps):").pack(side="left")
        self.ent_spd = tk.Entry(row2, width=8); self.ent_spd.insert(0, "200"); self.ent_spd.pack(side="left", padx=6)
        tk.Button(row2, text="GO (0xA4)", command=self.send_position_a4).pack(side="left", padx=8)

        # Emergency stop
        self.btn_stop = tk.Button(self, text="🚨 EMERGENCY STOP (SPACE) 🚨",
                                  bg="darkred", fg="white", font=("Arial", 16, "bold"),
                                  command=self.emergency_stop)
        self.btn_stop.pack(side="bottom", fill="x", padx=20, pady=10)
        self.bind('<space>', lambda e: self.emergency_stop())

        # start threads
        threading.Thread(target=self.monitor_thread, daemon=True).start()
        self.after(100, self.update_plot)

    def mid(self) -> int:
        try:
            return int(self.ent_mid.get())
        except:
            return 1

    # ---------- Buttons ----------
    def scan_ids(self):
        found = self.driver.scan_ids(1, 32)
        self.lbl_scan.config(text=f"Found: {found}")

    def brake_release(self):
        self.driver.brake_release(self.mid())

    def set_zero(self):
        if self.sending_torque or self.sending_impedance:
            messagebox.showwarning("Warning", "Disable control loops first!")
            return
        if messagebox.askyesno("Set Zero", "Really set zero?"):
            m = self.mid()
            self.driver.stop(m)
            time.sleep(0.05)
            self.driver.set_zero(m)

    def send_position_a4(self):
        if self.sending_torque:
            self.toggle_torque()
            time.sleep(0.1)
        if self.sending_impedance:
            self.toggle_impedance()
            time.sleep(0.1)

        m = self.mid()
        target = float(self.scale_pos.get())
        try:
            spd = int(self.ent_spd.get())
        except:
            spd = 200

        # recommended sequence
        self.driver.stop(m)
        time.sleep(0.02)
        self.driver.brake_release(m)
        time.sleep(0.02)
        self.driver.position_a4(m, target, speed_limit_dps=spd)

    def toggle_torque(self):
        if not self.sending_torque:
            try:
                self.safety_limit = float(self.ent_limit.get())
            except:
                messagebox.showerror("Error", "Invalid safety limit")
                return

            m = self.mid()
            self.driver.stop(m)
            time.sleep(0.02)
            self.driver.brake_release(m)
            time.sleep(0.02)

            self.sending_torque = True
            self.btn_torque.config(text=f"Torque Loop ON (MT limit {self.safety_limit}deg)", bg="green")
            threading.Thread(target=self.torque_loop, daemon=True).start()
        else:
            self.sending_torque = False
            self.btn_torque.config(text="Torque Loop OFF", bg="red")
            m = self.mid()
            self.driver.torque_a1(m, 0.0, want_reply=False)
            time.sleep(0.01)
            self.driver.stop(m)

    def toggle_impedance(self):
        if not self.sending_impedance:
            try:
                self.safety_limit = float(self.ent_limit.get())
                self.imp_target = float(self.ent_imp_target.get())
                self.kp = float(self.ent_kp.get())
                self.kd = float(self.ent_kd.get())
                self.i_lim = float(self.ent_ilim.get())
            except:
                messagebox.showerror("Error", "Invalid impedance params")
                return

            m = self.mid()
            self.driver.stop(m)
            time.sleep(0.02)
            self.driver.brake_release(m)
            time.sleep(0.02)

            self.sending_impedance = True
            self.btn_imp.config(text="Impedance ON", bg="purple")
            threading.Thread(target=self.impedance_loop, daemon=True).start()
        else:
            self.sending_impedance = False
            self.btn_imp.config(text="Impedance OFF", bg="gray20")
            m = self.mid()
            self.driver.torque_a1(m, 0.0, want_reply=False)
            time.sleep(0.01)
            self.driver.stop(m)

    # ---------- Core loops ----------
    def torque_loop(self):
        """
        Pure torque loop.
        Safety angle priority:
          - AngleMT from 0x92 (primary)
          - fallback hold last_mt_angle_deg (or fast angle if never got MT)
          - if 0x92 lost too long -> FAIL-SAFE STOP
        """
        m = self.mid()

        mt_fail = 0
        mt_fail_limit = 15  # adjust to your CAN timing/loads

        while self.sending_torque:
            iq_cmd = float(self.var_torque.get())

            # 1) torque command + fast feedback
            msg = self.driver.torque_a1(m, iq_cmd, want_reply=True)
            st = self.driver.parse_status_like(msg) if msg else None

            if st:
                temp = st["temp_c"]
                iq = st["iq_a"]
                speed = float(st["speed_dps"])
                ang_fast = float(st["angle_deg_i16"])
                self.last_fast_angle_deg = ang_fast
            else:
                temp = 0
                iq = 0.0
                speed = 0.0
                ang_fast = self.last_fast_angle_deg

            # 2) multiturn angle every cycle (primary)
            mt_msg = self.driver.read_multiturn_angle_92(m)
            mt = self.driver.parse_multiturn_92(mt_msg)

            if mt is not None:
                self.last_mt_angle_deg = float(mt)
                angle_mt = self.last_mt_angle_deg
                mt_fail = 0
            else:
                angle_mt = self.last_mt_angle_deg if self.last_mt_angle_deg != 0.0 else ang_fast
                mt_fail += 1

            # update UI + buffers (plot uses MT)
            self.push_status(temp, iq, speed, angle_mt, angle_fast=ang_fast)

            # FAIL-SAFE if multiturn read lost too long
            if mt_fail >= mt_fail_limit:
                self.after(0, lambda: messagebox.showwarning("Safety", "0x92 multiturn lost too long -> FAIL-SAFE STOP"))
                self.emergency_stop()
                break

            # safety limit uses MT
            if abs(angle_mt) > self.safety_limit:
                self.after(0, lambda: messagebox.showwarning("Safety", f"Angle limit exceeded (MT): {angle_mt:.1f} deg"))
                self.emergency_stop()
                break

    def impedance_loop(self):
        """
        "torque-position mode" implemented as PD (impedance) using 0xA1 current command.
        torque(A) = clamp( Kp*(target-angle) - Kd*speed, +/- I_limit )
        angle: use 0x92 (multiturn) every cycle if possible, fallback to fast angle.
        """
        m = self.mid()

        mt_fail = 0
        mt_fail_limit = 30  # impedance loop runs slower (sleep=0.01)

        while self.sending_impedance:
            # status: 0x9C
            st_msg = self.driver.read_status_9c(m)
            st = self.driver.parse_status_like(st_msg) if st_msg else None

            if st:
                temp = st["temp_c"]
                speed = float(st["speed_dps"])
                iq_meas = float(st["iq_a"])
                ang_fast = float(st["angle_deg_i16"])
                self.last_fast_angle_deg = ang_fast
            else:
                temp = 0
                speed = 0.0
                iq_meas = 0.0
                ang_fast = self.last_fast_angle_deg

            # multiturn: 0x92
            mt_msg = self.driver.read_multiturn_angle_92(m)
            mt = self.driver.parse_multiturn_92(mt_msg)

            if mt is not None:
                self.last_mt_angle_deg = float(mt)
                angle = self.last_mt_angle_deg
                mt_fail = 0
            else:
                angle = self.last_mt_angle_deg if self.last_mt_angle_deg != 0.0 else ang_fast
                mt_fail += 1

            # PD current command
            err = (self.imp_target - angle)
            iq_cmd = (self.kp * err) - (self.kd * speed)
            iq_cmd = max(-self.i_lim, min(self.i_lim, iq_cmd))

            # send torque
            self.driver.torque_a1(m, iq_cmd, want_reply=False)

            # update UI (MT priority)
            self.push_status(
                temp, iq_meas, speed, angle,
                angle_fast=ang_fast,
                extra=f" | ImpTgt:{self.imp_target:.1f} Kp:{self.kp} Kd:{self.kd}"
            )

            # FAIL-SAFE if multiturn lost too long
            if mt_fail >= mt_fail_limit:
                self.after(0, lambda: messagebox.showwarning("Safety", "0x92 multiturn lost too long -> FAIL-SAFE STOP"))
                self.emergency_stop()
                break

            # safety
            if abs(angle) > self.safety_limit:
                self.after(0, lambda: messagebox.showwarning("Safety", f"Angle limit exceeded (MT): {angle:.1f} deg"))
                self.emergency_stop()
                break

            time.sleep(0.01)  # 100 Hz-ish

    def monitor_thread(self):
        """
        When no control loops are running, monitor.
        Prefer multiturn(0x92) for displayed angle/plot.
        """
        while True:
            if self.monitoring and (not self.sending_torque) and (not self.sending_impedance):
                m = self.mid()

                # status: 0x9C
                msg = self.driver.read_status_9c(m)
                st = self.driver.parse_status_like(msg) if msg else None

                if st:
                    temp = st["temp_c"]
                    iq = st["iq_a"]
                    spd = float(st["speed_dps"])
                    ang_fast = float(st["angle_deg_i16"])
                    self.last_fast_angle_deg = ang_fast
                else:
                    temp = 0
                    iq = 0.0
                    spd = 0.0
                    ang_fast = self.last_fast_angle_deg

                # multiturn: 0x92 every cycle (monitor is slow anyway)
                mt_msg = self.driver.read_multiturn_angle_92(m)
                mt = self.driver.parse_multiturn_92(mt_msg)
                if mt is not None:
                    self.last_mt_angle_deg = float(mt)

                angle_mt = self.last_mt_angle_deg if self.last_mt_angle_deg != 0.0 else ang_fast
                self.push_status(temp, iq, spd, angle_mt, angle_fast=ang_fast)

                time.sleep(0.05)  # 20 Hz
            else:
                time.sleep(0.1)

    # ---------- UI update helpers ----------
    def push_status(self, temp, iq, speed, angle_mt, angle_fast=None, extra=""):
        if angle_fast is None:
            status = f"Temp:{temp}C | Iq:{iq:+.2f}A | Spd:{speed:+.1f}dps | AngleMT:{angle_mt:+.2f}deg{extra}"
        else:
            status = f"Temp:{temp}C | Iq:{iq:+.2f}A | Spd:{speed:+.1f}dps | AngleMT:{angle_mt:+.2f}deg | AngleFast:{angle_fast:+.1f}deg{extra}"

        self.after(0, lambda s=status: self.lbl_status.config(text=s))

        # plot uses multiturn angle
        self.angle_data.append(float(angle_mt))
        self.current_data.append(float(iq))
        self.speed_data.append(float(speed))

    def update_plot(self):
        self.line_ang.set_data(range(len(self.angle_data)), list(self.angle_data))
        self.line_iq.set_data(range(len(self.current_data)), list(self.current_data))

        self.ax1.set_xlim(0, len(self.angle_data))
        self.ax1.relim()
        self.ax1.autoscale_view(scalex=False, scaley=True)

        self.ax2.relim()
        self.ax2.autoscale_view(scalex=False, scaley=True)

        self.canvas.draw()
        self.after(100, self.update_plot)

    # ---------- Emergency Stop ----------
    def emergency_stop(self):
        """
        Stronger e-stop:
          1) stop loops
          2) torque 0
          3) shutdown (0x80)
          4) brake lock (0x78)
        """
        self.sending_torque = False
        self.sending_impedance = False
        self.btn_torque.config(text="Torque Loop OFF", bg="red")
        self.btn_imp.config(text="Impedance OFF", bg="gray20")

        m = self.mid()

        # 0 torque first
        self.driver.torque_a1(m, 0.0, want_reply=False)
        time.sleep(0.01)

        # shutdown + brake lock
        self.driver.shutdown(m)
        time.sleep(0.01)
        self.driver.brake_lock(m)

        print("[E-STOP] executed")


if __name__ == "__main__":
    driver = MotorDriver(channel="can0", bitrate=1000000)
    app = App(driver)
    app.mainloop()
