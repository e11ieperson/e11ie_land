import tkinter as tk
from tkinter import ttk, messagebox
import json
import os
import serial
import serial.tools.list_ports
import time
import threading
import re
import math
import queue
from collections import deque
from tkinter import scrolledtext
from matplotlib.figure import Figure
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg

CONFIG_FILE = "robot_config.json"
BAUD_RATE = 115200

DARK_BG = "#0f131a"
DARK_PANEL = "#161b22"
DARK_ELEV = "#1b2430"
ACCENT = "#58a6ff"
TEXT = "#e6edf3"
MUTED = "#9fb0c3"

# Default configs
DEFAULT_CONFIG = {
    "motors": {
        "1": {"type": "M3508", "gear_ratio": 94.23076923076923, "kp": 5.0, "ki": 1.0, "kd": 0.0, "aol": 500.0, "skp": 0.3, "ski": 0.0, "skd": 0.0, "sol": 20.0, "min_angle": 0.0, "max_angle": 360.0, "zero_offset": 0.0, "abs_zero_offset": 0.0, "abs_encoder_index": None, "abs_encoder_reversed": True, "abs_pid_enabled": False},
        "2": {"type": "M2006", "gear_ratio": 94.23076923076923, "kp": 5.0, "ki": 1.0, "kd": 0.0, "aol": 500.0, "skp": 0.3, "ski": 0.0, "skd": 0.0, "sol": 20.0, "min_angle": 0.0, "max_angle": 360.0, "zero_offset": 0.0, "abs_zero_offset": 0.0, "abs_encoder_index": None, "abs_encoder_reversed": True, "abs_pid_enabled": False},
        "3": {"type": "M2006", "gear_ratio": 94.23076923076923, "kp": 5.0, "ki": 1.0, "kd": 0.0, "aol": 500.0, "skp": 0.3, "ski": 0.0, "skd": 0.0, "sol": 20.0, "min_angle": 0.0, "max_angle": 360.0, "zero_offset": 0.0, "abs_zero_offset": 0.0, "abs_encoder_index": None, "abs_encoder_reversed": True, "abs_pid_enabled": False},
        "4": {"type": "M2006", "gear_ratio": 1.0, "kp": 5.0, "ki": 1.0, "kd": 0.0, "aol": 500.0, "skp": 0.3, "ski": 0.0, "skd": 0.0, "sol": 20.0, "min_angle": 0.0, "max_angle": 360.0, "zero_offset": 0.0, "abs_zero_offset": 0.0, "abs_encoder_index": None, "abs_encoder_reversed": True, "abs_pid_enabled": False},
        "5": {"type": "M2006", "gear_ratio": 1.0, "kp": 5.0, "ki": 1.0, "kd": 0.0, "aol": 500.0, "skp": 0.3, "ski": 0.0, "skd": 0.0, "sol": 20.0, "min_angle": 0.0, "max_angle": 360.0, "zero_offset": 0.0, "abs_zero_offset": 0.0, "abs_encoder_index": None, "abs_encoder_reversed": True, "abs_pid_enabled": False},
        "6": {"type": "M2006", "gear_ratio": 1.0, "kp": 5.0, "ki": 1.0, "kd": 0.0, "aol": 500.0, "skp": 0.3, "ski": 0.0, "skd": 0.0, "sol": 20.0, "min_angle": 0.0, "max_angle": 360.0, "zero_offset": 0.0, "abs_zero_offset": 0.0, "abs_encoder_index": None, "abs_encoder_reversed": True, "abs_pid_enabled": False}
    },
    "wrist_differential": {
        "enabled": True,
        "pitch_joint_id": 5,
        "roll_joint_id": 6,
        "pitch_encoder_index": None,
        "roll_encoder_index": None,
        "pitch_encoder_motor_id": 5,
        "roll_encoder_motor_id": 6,
        "motor_a_id": 5,
        "motor_b_id": 6,
        "mix": {
            "motor_a_pitch_sign": 1.0,
            "motor_a_roll_sign": 1.0,
            "motor_b_pitch_sign": 1.0,
            "motor_b_roll_sign": -1.0
        },
        "pid": {
            "pitch_kp": 4.0,
            "pitch_ki": 0.0,
            "pitch_kd": 0.0,
            "roll_kp": 4.0,
            "roll_ki": 0.0,
            "roll_kd": 0.0,
            "max_axis_speed": 80.0,
            "max_motor_speed": 120.0,
            "deadband_deg": 0.4,
            "integral_limit": 30.0
        }
    }
}

SAFE_PID_PRESETS = {
    "Conservative": {
        "M2006": {"kp": 3.0, "ki": 0.2, "kd": 0.0, "aol": 140.0, "skp": 0.18, "ski": 0.0, "skd": 0.0, "sol": 8.0},
        "M3508": {"kp": 4.0, "ki": 0.2, "kd": 0.0, "aol": 120.0, "skp": 0.12, "ski": 0.0, "skd": 0.0, "sol": 8.0},
    },
    "Balanced": {
        "M2006": {"kp": 5.0, "ki": 0.5, "kd": 0.0, "aol": 220.0, "skp": 0.25, "ski": 0.0, "skd": 0.0, "sol": 12.0},
        "M3508": {"kp": 8.0, "ki": 0.5, "kd": 0.0, "aol": 180.0, "skp": 0.18, "ski": 0.0, "skd": 0.0, "sol": 12.0},
    },
    "Aggressive": {
        "M2006": {"kp": 8.0, "ki": 0.8, "kd": 0.0, "aol": 320.0, "skp": 0.35, "ski": 0.0, "skd": 0.0, "sol": 16.0},
        "M3508": {"kp": 12.0, "ki": 0.8, "kd": 0.0, "aol": 260.0, "skp": 0.28, "ski": 0.0, "skd": 0.0, "sol": 16.0},
    },
}


def invert_wrist_mix(motor_a_val, motor_b_val, mix_cfg):
    a11 = float(mix_cfg.get("motor_a_pitch_sign", 1.0))
    a12 = float(mix_cfg.get("motor_a_roll_sign", 1.0))
    a21 = float(mix_cfg.get("motor_b_pitch_sign", 1.0))
    a22 = float(mix_cfg.get("motor_b_roll_sign", -1.0))
    det = (a11 * a22) - (a12 * a21)
    if abs(det) < 1e-6:
        return None, None
    pitch = ((float(motor_a_val) * a22) - (a12 * float(motor_b_val))) / det
    roll = ((a11 * float(motor_b_val)) - (float(motor_a_val) * a21)) / det
    return float(pitch), float(roll)


def clampf(value, min_value, max_value):
    return max(min_value, min(max_value, value))

class MotorTab:
    def __init__(self, parent, motor_id, config_data, send_cmd_func):
        self.motor_id = motor_id
        self.config = config_data

        # Ensure new defaults exist
        if "skp" not in self.config: self.config["skp"] = 0.3
        if "ski" not in self.config: self.config["ski"] = 0.0
        if "skd" not in self.config: self.config["skd"] = 0.0
        if "aol" not in self.config: self.config["aol"] = 500.0
        if "sol" not in self.config: self.config["sol"] = 20.0
        if "min_angle"   not in self.config: self.config["min_angle"]   = 0.0
        if "max_angle"   not in self.config: self.config["max_angle"]   = 360.0
        if "zero_offset" not in self.config: self.config["zero_offset"] = 0.0
        if "abs_zero_offset" not in self.config: self.config["abs_zero_offset"] = 0.0
        if "abs_encoder_index" not in self.config: self.config["abs_encoder_index"] = None
        if "abs_encoder_reversed" not in self.config: self.config["abs_encoder_reversed"] = True
        if "abs_pid_enabled" not in self.config: self.config["abs_pid_enabled"] = False

        self.send_cmd = send_cmd_func

        self.frame = ttk.Frame(parent, padding=10)

        # --- Real-Time Telemetry ---
        lf_telemetry = ttk.LabelFrame(self.frame, text="Real-Time Telemetry", padding=10)
        lf_telemetry.pack(fill="x", pady=5)

        ttk.Label(lf_telemetry, text="Current Angle:", font=("Arial", 10, "bold")).grid(row=0, column=0, sticky="e", padx=5)
        self.lbl_current_angle = ttk.Label(lf_telemetry, text="---", foreground="blue", font=("Arial", 10, "bold"))
        self.lbl_current_angle.grid(row=0, column=1, sticky="w")

        ttk.Label(lf_telemetry, text="Target Angle:", font=("Arial", 10, "bold")).grid(row=0, column=2, sticky="e", padx=(20, 5))
        self.lbl_target_angle = ttk.Label(lf_telemetry, text="---", font=("Arial", 10))
        self.lbl_target_angle.grid(row=0, column=3, sticky="w")
        self.target_deg = None

        ttk.Label(lf_telemetry, text="Velocity:", font=("Arial", 10, "bold")).grid(row=1, column=0, sticky="e", padx=5)
        self.lbl_current_rpm = ttk.Label(lf_telemetry, text="--- RPM", foreground="green", font=("Arial", 10, "bold"))
        self.lbl_current_rpm.grid(row=1, column=1, sticky="w")

        ttk.Label(lf_telemetry, text="Abs Pos (Enc):", font=("Arial", 10, "bold")).grid(row=1, column=2, sticky="e", padx=(20, 5))
        self.lbl_abs_pos = ttk.Label(lf_telemetry, text="---", foreground="orange", font=("Arial", 10, "bold"))
        self.lbl_abs_pos.grid(row=1, column=3, sticky="w")

        btn_zero = ttk.Button(lf_telemetry, text="Set Zero Position (CAL)", command=self.set_zero)
        btn_zero.grid(row=0, column=4, padx=(20, 0))

        # --- Motor Specs ---
        lf_specs = ttk.LabelFrame(self.frame, text="Motor Specifications", padding=10)
        lf_specs.pack(fill="x", pady=5)

        ttk.Label(lf_specs, text="Motor Type:").grid(row=0, column=0, sticky="w", pady=2)
        self.var_type = tk.StringVar(value=self.config["type"])
        cb_type = ttk.Combobox(lf_specs, textvariable=self.var_type, values=["M2006", "M3508"], state="readonly", width=10)
        cb_type.grid(row=0, column=1, sticky="w", pady=2, padx=5)
        cb_type.bind("<<ComboboxSelected>>", self.on_change)

        ttk.Label(lf_specs, text="External Gear Ratio:").grid(row=1, column=0, sticky="w", pady=2)
        self.var_gear = tk.StringVar(value=str(self.config["gear_ratio"]))
        en_gear = ttk.Entry(lf_specs, textvariable=self.var_gear, width=20)
        en_gear.grid(row=1, column=1, sticky="w", pady=2, padx=5)
        en_gear.bind("<KeyRelease>", self.on_change)

        btn_apply_gear = ttk.Button(lf_specs, text="Apply Gear to Arm", command=self.apply_gear)
        btn_apply_gear.grid(row=1, column=2, padx=10)

        # --- Absolute Encoder Mapping ---
        lf_encoder = ttk.LabelFrame(self.frame, text="Absolute Encoder Mapping", padding=10)
        lf_encoder.pack(fill="x", pady=5)

        ttk.Label(lf_encoder, text="Encoder Index").grid(row=0, column=0, sticky="w")
        self.var_abs_enc = tk.StringVar(value=("None" if self.config.get("abs_encoder_index") is None else str(int(self.config.get("abs_encoder_index")))))
        cb_abs = ttk.Combobox(lf_encoder, textvariable=self.var_abs_enc, values=["None"] + [str(i) for i in range(8)], state="readonly", width=8)
        cb_abs.grid(row=0, column=1, sticky="w", padx=6)
        cb_abs.bind("<<ComboboxSelected>>", self.on_change)

        self.var_abs_rev = tk.BooleanVar(value=bool(self.config.get("abs_encoder_reversed", True)))
        ttk.Checkbutton(lf_encoder, text="Reversed", variable=self.var_abs_rev, command=self.on_change).grid(row=0, column=2, sticky="w", padx=(12, 6))

        self.var_abs_pid = tk.BooleanVar(value=bool(self.config.get("abs_pid_enabled", False)))
        ttk.Checkbutton(lf_encoder, text="Use ABS for PID", variable=self.var_abs_pid, command=self.on_change).grid(row=0, column=3, sticky="w", padx=(12, 6))

        ttk.Button(lf_encoder, text="Apply Encoder Mapping", command=self.apply_encoder_mapping).grid(row=0, column=4, sticky="e", padx=6)
        ttk.Button(lf_encoder, text="Apply ABS PID", command=self.apply_abs_pid).grid(row=0, column=5, sticky="e", padx=6)
        lf_encoder.columnconfigure(6, weight=1)

        # --- PID Frames Container ---
        pid_container = ttk.Frame(self.frame)
        pid_container.pack(fill="x", pady=5)

        # --- Angle PID ---
        lf_pid = ttk.LabelFrame(pid_container, text="Angle PID Tuning", padding=10)
        lf_pid.pack(side="left", fill="x", expand=True, padx=(0, 5))

        self.var_kp = tk.DoubleVar(value=self.config["kp"])
        self.var_ki = tk.DoubleVar(value=self.config["ki"])
        self.var_kd = tk.DoubleVar(value=self.config["kd"])
        self.var_aol = tk.DoubleVar(value=self.config["aol"])

        self.create_entry(lf_pid, "Kp:", self.var_kp, row=0)
        self.create_entry(lf_pid, "Ki:", self.var_ki, row=1)
        self.create_entry(lf_pid, "Kd:", self.var_kd, row=2)
        self.create_entry(lf_pid, "Max Speed (RPM):", self.var_aol, row=3)

        ttk.Button(lf_pid, text="Apply Angle PID", command=self.apply_angle_pid).grid(row=4, column=0, columnspan=2, pady=10)

        # --- Speed PID ---
        lf_spid = ttk.LabelFrame(pid_container, text="Speed PID Tuning", padding=10)
        lf_spid.pack(side="right", fill="x", expand=True, padx=(5, 0))

        self.var_skp = tk.DoubleVar(value=self.config["skp"])
        self.var_ski = tk.DoubleVar(value=self.config["ski"])
        self.var_skd = tk.DoubleVar(value=self.config["skd"])
        self.var_sol = tk.DoubleVar(value=self.config["sol"])

        self.create_entry(lf_spid, "Kp:", self.var_skp, row=0)
        self.create_entry(lf_spid, "Ki:", self.var_ski, row=1)
        self.create_entry(lf_spid, "Kd:", self.var_skd, row=2)
        self.create_entry(lf_spid, "Max Current (A):", self.var_sol, row=3)

        ttk.Button(lf_spid, text="Apply Speed PID", command=self.apply_speed_pid).grid(row=4, column=0, columnspan=2, pady=10)

        # --- Quick Presets ---
        lf_presets = ttk.LabelFrame(self.frame, text="Quick Presets", padding=10)
        lf_presets.pack(fill="x", pady=5)
        self.var_preset = tk.StringVar(value="Conservative")
        ttk.Label(lf_presets, text="Preset:").pack(side="left")
        ttk.Combobox(
            lf_presets,
            textvariable=self.var_preset,
            values=list(SAFE_PID_PRESETS.keys()),
            state="readonly",
            width=14,
        ).pack(side="left", padx=6)
        ttk.Button(lf_presets, text="Load Preset", command=self.load_selected_preset).pack(side="left", padx=4)
        ttk.Button(lf_presets, text="Apply Motor All", command=self.apply_motor_all).pack(side="left", padx=8)

        # --- Test Controls ---
        lf_test = ttk.LabelFrame(self.frame, text="Testing Controls", padding=10)
        lf_test.pack(fill="x", pady=5)

        ttk.Button(lf_test, text="Go 0 (ABS)",   command=lambda: self.test_move(0)).pack(side="left", padx=5)
        ttk.Button(lf_test, text="Go 90 (ABS)",  command=lambda: self.test_move(90)).pack(side="left", padx=5)
        ttk.Button(lf_test, text="Go 180 (ABS)", command=lambda: self.test_move(180)).pack(side="left", padx=5)
        ttk.Button(lf_test, text="Go 270 (ABS)", command=lambda: self.test_move(270)).pack(side="left", padx=5)

        self.var_custom_abs = tk.StringVar(value="0")
        self.var_custom_delta = tk.StringVar(value="5")
        ttk.Separator(lf_test, orient="vertical").pack(side="left", fill="y", padx=8)
        ttk.Label(lf_test, text="Abs:").pack(side="left", padx=(2, 2))
        ttk.Entry(lf_test, textvariable=self.var_custom_abs, width=7).pack(side="left", padx=(0, 4))
        ttk.Button(lf_test, text="Go", command=self.test_move_custom).pack(side="left", padx=(0, 6))
        ttk.Label(lf_test, text="Rel:").pack(side="left", padx=(2, 2))
        ttk.Entry(lf_test, textvariable=self.var_custom_delta, width=7).pack(side="left", padx=(0, 4))
        ttk.Button(lf_test, text="Jog", command=self.jog_custom).pack(side="left", padx=(0, 6))

        ttk.Button(lf_test, text="STOP", command=lambda: self.send_cmd("S")).pack(side="right", padx=5)

        # --- Jog Controls ---
        lf_jog = ttk.LabelFrame(self.frame, text="Jog & Zero", padding=10)
        lf_jog.pack(fill="x", pady=5)

        ttk.Button(lf_jog, text="-10°", command=lambda: self.jog(-10)).pack(side="left", padx=4)
        ttk.Button(lf_jog, text="-1°",  command=lambda: self.jog(-1)).pack(side="left", padx=4)
        ttk.Button(lf_jog, text="+1°",  command=lambda: self.jog(+1)).pack(side="left", padx=4)
        ttk.Button(lf_jog, text="+10°", command=lambda: self.jog(+10)).pack(side="left", padx=4)
        ttk.Separator(lf_jog, orient="vertical").pack(side="left", fill="y", padx=8)
        ttk.Button(lf_jog, text="Set Zero Here (CAL)", command=self.set_zero).pack(side="left", padx=4)

        # --- Joint Limits & Zero Offset ---
        lf_limits = ttk.LabelFrame(self.frame, text="Joint Limits & Zero Offset", padding=10)
        lf_limits.pack(fill="x", pady=5)

        self.var_min   = tk.DoubleVar(value=self.config["min_angle"])
        self.var_max   = tk.DoubleVar(value=self.config["max_angle"])
        self.var_zoff  = tk.DoubleVar(value=self.config["zero_offset"])

        ttk.Label(lf_limits, text="Min Angle (°):").grid(row=0, column=0, sticky="e", padx=5, pady=3)
        ttk.Entry(lf_limits, textvariable=self.var_min,  width=10).grid(row=0, column=1, sticky="w", padx=5, pady=3)
        ttk.Label(lf_limits, text="Max Angle (°):").grid(row=0, column=2, sticky="e", padx=5, pady=3)
        ttk.Entry(lf_limits, textvariable=self.var_max,  width=10).grid(row=0, column=3, sticky="w", padx=5, pady=3)
        ttk.Label(lf_limits, text="Zero Offset (°):", font=("Arial", 9)).grid(row=1, column=0, sticky="e", padx=5, pady=3)
        ttk.Entry(lf_limits, textvariable=self.var_zoff, width=10).grid(row=1, column=1, sticky="w", padx=5, pady=3)
        ttk.Label(lf_limits, text="Motor deg when IK=0°", font=("Arial", 8), foreground="gray").grid(row=1, column=2, columnspan=2, sticky="w")
        ttk.Button(lf_limits, text="Apply Limits", command=self.apply_limits).grid(row=0, column=4, rowspan=2, padx=10)

        # --- Visual Angle Gauge ---
        lf_vis = ttk.LabelFrame(self.frame, text="Joint Range Visualization", padding=5)
        lf_vis.pack(fill="x", pady=5)

        self._gauge_size = 130
        self._gauge_canvas = tk.Canvas(lf_vis, width=self._gauge_size, height=self._gauge_size,
                                        bg="white", highlightthickness=0)
        self._gauge_canvas.pack(side="left", padx=10)
        self._gauge_current_deg = None  # updated by telemetry

        # Legend next to gauge
        legend = ttk.Frame(lf_vis)
        legend.pack(side="left", padx=10, anchor="n")
        ttk.Label(legend, text="● Gray = full 360°",   foreground="gray",   font=("Arial", 8)).pack(anchor="w")
        ttk.Label(legend, text="● Green = allowed",     foreground="#2d8a2d", font=("Arial", 8)).pack(anchor="w")
        ttk.Label(legend, text="● Blue = logic pos",  foreground="blue",   font=("Arial", 8)).pack(anchor="w")
        ttk.Label(legend, text="● Red = zero (CAL 0°)", foreground="red",    font=("Arial", 8)).pack(anchor="w")
        ttk.Label(legend, text="● Orange = abs enc pos", foreground="orange",   font=("Arial", 8)).pack(anchor="w")

        self._gauge_abs_deg = None  # updated by telemetry (if APOS exists)

        self._last_telemetry_time = 0.0

        self._draw_gauge()  # initial draw
        # Redraw periodically
        self._gauge_canvas.after(200, self._gauge_tick)


    def create_entry(self, parent, label, variable, row):
        ttk.Label(parent, text=label).grid(row=row, column=0, sticky="e", padx=5, pady=5)

        def entry_changed(*args):
            try:
                variable.get()  # will throw TclError if empty
                self.on_change()
            except Exception:
                pass  # ignore empty field / invalid input during typing

        # To avoid infinite loop, use trace sparingly or bind
        entry = ttk.Entry(parent, textvariable=variable, width=15)
        entry.grid(row=row, column=1, padx=5, pady=5, sticky="w")
        entry.bind("<KeyRelease>", entry_changed)

    def on_change(self, *args):
        self.config["type"] = self.var_type.get()
        try:
            self.config["gear_ratio"]  = float(self.var_gear.get())
            self.config["kp"]          = self.var_kp.get()
            self.config["ki"]          = self.var_ki.get()
            self.config["kd"]          = self.var_kd.get()
            self.config["aol"]         = self.var_aol.get()
            self.config["skp"]         = self.var_skp.get()
            self.config["ski"]         = self.var_ski.get()
            self.config["skd"]         = self.var_skd.get()
            self.config["sol"]         = self.var_sol.get()
            self.config["min_angle"]   = self.var_min.get()
            self.config["max_angle"]   = self.var_max.get()
            self.config["zero_offset"] = self.var_zoff.get()
        except ValueError:
            pass

        try:
            v = self.var_abs_enc.get().strip()
            if v.lower() == "none" or v == "":
                self.config["abs_encoder_index"] = None
            else:
                self.config["abs_encoder_index"] = int(v)
        except Exception:
            self.config["abs_encoder_index"] = None

        try:
            self.config["abs_encoder_reversed"] = bool(self.var_abs_rev.get())
        except Exception:
            self.config["abs_encoder_reversed"] = True

        try:
            self.config["abs_pid_enabled"] = bool(self.var_abs_pid.get())
        except Exception:
            self.config["abs_pid_enabled"] = False

    def apply_gear(self):
        try:
            gr = float(self.var_gear.get())
            self.send_cmd(f"GEAR {self.motor_id} {gr}")
        except ValueError:
            messagebox.showerror("Error", "Invalid Gear Ratio")

    def apply_encoder_mapping(self):
        self.on_change()
        enc_idx = self.config.get("abs_encoder_index")
        mid = int(self.motor_id)

        if enc_idx is None:
            self.send_cmd(f"ENCMAP 0 0")
            return

        try:
            enc_i = int(enc_idx)
        except Exception:
            messagebox.showerror("Error", "Invalid encoder index")
            return

        if enc_i < 0 or enc_i > 7:
            messagebox.showerror("Error", "Encoder index must be 0-7")
            return

        self.send_cmd(f"ENCMAP {enc_i} {mid}")
        rev_i = 1 if bool(self.config.get("abs_encoder_reversed", True)) else 0
        self.send_cmd(f"ENCREVSET {enc_i} {rev_i} {mid}")

    def apply_abs_pid(self):
        self.on_change()
        mid = int(self.motor_id)
        enable = 1 if bool(self.config.get("abs_pid_enabled", False)) else 0
        enc_idx = self.config.get("abs_encoder_index")
        if enable == 1 and enc_idx is None:
            self.send_cmd(f"ABSPID {mid} 0")
            return
        if enc_idx is not None:
            try:
                enc_i = int(enc_idx)
                rev_i = 1 if bool(self.config.get("abs_encoder_reversed", True)) else 0
                self.send_cmd(f"ENCMAP {enc_i} {mid}")
                time.sleep(0.02)
                self.send_cmd(f"ENCREVSET {enc_i} {rev_i} {mid}")
                # Firmware marks abs encoders as detected asynchronously.
                # Let mapping/detection settle before enabling ABSPID.
                time.sleep(0.20)
            except Exception:
                pass
        self.send_cmd(f"ABSPID {mid} {enable}")
        if enable == 1:
            for _ in range(3):
                time.sleep(0.10)
                self.send_cmd(f"ABSPID {mid} 1")

    def apply_angle_pid(self):
        kp = self.var_kp.get()
        ki = self.var_ki.get()
        kd = self.var_kd.get()
        aol = self.var_aol.get()
        mid = self.motor_id
        batch = f"KP {mid} {kp:.4f}; KI {mid} {ki:.4f}; KD {mid} {kd:.4f}; AOL {mid} {aol:.2f}"
        self.send_cmd(batch)

    def apply_speed_pid(self):
        skp = self.var_skp.get()
        ski = self.var_ski.get()
        skd = self.var_skd.get()
        sol = self.var_sol.get()
        mid = self.motor_id
        batch = f"SKP {mid} {skp:.4f}; SKI {mid} {ski:.4f}; SKD {mid} {skd:.4f}; SOL {mid} {sol:.2f}"
        self.send_cmd(batch)

    def load_selected_preset(self):
        preset_name = self.var_preset.get()
        motor_type = self.var_type.get()
        preset = SAFE_PID_PRESETS.get(preset_name, {}).get(motor_type)
        if not preset:
            messagebox.showerror("Preset Error", f"No preset found for {preset_name}/{motor_type}")
            return

        self.var_kp.set(preset["kp"])
        self.var_ki.set(preset["ki"])
        self.var_kd.set(preset["kd"])
        self.var_aol.set(preset["aol"])
        self.var_skp.set(preset["skp"])
        self.var_ski.set(preset["ski"])
        self.var_skd.set(preset["skd"])
        self.var_sol.set(preset["sol"])
        self.on_change()

    def apply_motor_all(self):
        self.on_change()
        mid = self.motor_id
        gr = self.var_gear.get()
        mn = self.var_min.get()
        mx = self.var_max.get()
        if mx <= mn:
            messagebox.showerror("Error", "Max angle must be greater than Min angle")
            return
        kp = self.var_kp.get()
        ki = self.var_ki.get()
        kd = self.var_kd.get()
        aol = self.var_aol.get()
        skp = self.var_skp.get()
        ski = self.var_ski.get()
        skd = self.var_skd.get()
        sol = self.var_sol.get()
        batch = (
            f"GEAR {mid} {float(gr):.6f}"
            f"; JLIM {mid} {mn:.1f} {mx:.1f}"
            f"; KP {mid} {kp:.4f}; KI {mid} {ki:.4f}; KD {mid} {kd:.4f}; AOL {mid} {aol:.2f}"
            f"; SKP {mid} {skp:.4f}; SKI {mid} {ski:.4f}; SKD {mid} {skd:.4f}; SOL {mid} {sol:.2f}"
        )
        self.send_cmd(batch)
        enc_idx = self.config.get("abs_encoder_index")
        if enc_idx is not None:
            try:
                enc_i = int(enc_idx)
                rev_i = 1 if bool(self.config.get("abs_encoder_reversed", True)) else 0
                self.send_cmd(f"ENCREVSET {enc_i} {rev_i} {int(mid)}")
            except Exception:
                pass

    def test_move(self, angle):
        angle = float(angle)
        self.send_cmd(f"P {self.motor_id} {angle}")
        self._set_target_display(angle)

    def test_move_custom(self):
        try:
            angle = float(self.var_custom_abs.get())
        except ValueError:
            messagebox.showerror("Error", "Invalid absolute angle value")
            return
        self.send_cmd(f"P {self.motor_id} {angle}")
        self._set_target_display(angle)

    def jog(self, delta):
        """Jog motor by delta degrees using a bounded absolute target."""
        try:
            delta_f = float(delta)
        except Exception:
            return

        reference = self._gauge_current_deg
        if reference is None:
            reference = self.target_deg
        if reference is None:
            messagebox.showwarning("Jog Unavailable", "No telemetry yet for this motor. Wait for live POS telemetry, then jog.")
            return

        target = self._project_target_to_limits(float(reference) + delta_f, reference=reference)
        self.send_cmd(f"P {self.motor_id} {target:.3f}")
        self._set_target_display(target, reference=reference)

    def jog_custom(self):
        try:
            delta = float(self.var_custom_delta.get())
        except ValueError:
            messagebox.showerror("Error", "Invalid relative delta value")
            return
        self.jog(delta)

    def set_zero(self):
        self.send_cmd(f"CAL {self.motor_id} 0")
        self._set_target_display(0.0)

    def apply_limits(self):
        mid = self.motor_id
        mn  = self.var_min.get()
        mx  = self.var_max.get()
        if mx <= mn:
            messagebox.showerror("Error", "Max angle must be greater than Min angle")
            return
        self.on_change()  # persist to config
        self.send_cmd(f"JLIM {mid} {mn:.1f} {mx:.1f}")
        if self.target_deg is not None:
            self._set_target_display(self.target_deg, reference=self._gauge_current_deg)

    def _clamp_target_to_limits(self, angle):
        mn = self.var_min.get()
        mx = self.var_max.get()
        if mx <= mn:
            mn, mx = 0.0, 360.0
        width = mx - mn
        a = float(angle)
        if width >= 359.5:
            while a >= 360.0:
                a -= 360.0
            while a < 0.0:
                a += 360.0
            return a
        return max(mn, min(mx, a))

    def _project_target_to_limits(self, angle, reference=None):
        """Project a logical angle into the same frame used by firmware limits."""
        mn = self.var_min.get()
        mx = self.var_max.get()
        if mx <= mn:
            mn, mx = 0.0, 360.0
        width = mx - mn
        a = float(angle)
        if width >= 359.5:
            while a >= 360.0:
                a -= 360.0
            while a < 0.0:
                a += 360.0
            return a

        if mn <= a <= mx:
            return a

        if reference is None:
            if self._gauge_current_deg is not None:
                reference = self._gauge_current_deg
            else:
                reference = (mn + mx) * 0.5
        turns = round((float(reference) - a) / 360.0)
        a = a + (turns * 360.0)
        return max(mn, min(mx, a))

    def _set_target_display(self, angle, reference=None):
        projected = self._project_target_to_limits(angle, reference=reference)
        self.target_deg = projected
        self.lbl_target_angle.config(text=f"{projected:.1f}°")

    def _draw_gauge(self):
        import math
        c = self._gauge_canvas
        s = self._gauge_size
        c.delete("all")
        cx, cy = s / 2, s / 2
        r = s / 2 - 10

        try:
            mn = self.var_min.get()
            mx = self.var_max.get()
            zoff = self.var_zoff.get()
        except Exception:
            mn, mx, zoff = 0, 360, 0

        # Gray circle (full 360°)
        c.create_oval(cx - r, cy - r, cx + r, cy + r, outline="#ddd", width=8)

        # Green arc for allowed range (tkinter arcs go counter-clockwise from 3-o'clock)
        # Convert our 0=top, CW convention to tkinter's 0=right, CCW convention
        start_tk = 90 - mn  # min angle mapped
        extent_tk = -(mx - mn)  # negative = clockwise in tkinter
        if abs(extent_tk) > 0.1:
            c.create_arc(cx - r, cy - r, cx + r, cy + r,
                         start=start_tk, extent=extent_tk,
                         style="arc", outline="#2d8a2d", width=8)

        # Tick marks at 0, 90, 180, 270
        for deg in [0, 90, 180, 270]:
            rad = math.radians(deg - 90)  # -90 so 0=top
            x1 = cx + (r - 6) * math.cos(rad)
            y1 = cy + (r - 6) * math.sin(rad)
            x2 = cx + (r + 3) * math.cos(rad)
            y2 = cy + (r + 3) * math.sin(rad)
            c.create_line(x1, y1, x2, y2, fill="#999", width=1)
            lx = cx + (r + 9) * math.cos(rad)
            ly = cy + (r + 9) * math.sin(rad)
            c.create_text(lx, ly, text=str(deg), font=("Arial", 6), fill="#999")

        # Red tick for zero offset position
        rad_z = math.radians(zoff - 90)
        zx1 = cx + (r - 10) * math.cos(rad_z)
        zy1 = cy + (r - 10) * math.sin(rad_z)
        zx2 = cx + (r + 2) * math.cos(rad_z)
        zy2 = cy + (r + 2) * math.sin(rad_z)
        c.create_line(zx1, zy1, zx2, zy2, fill="red", width=2)

        # Blue needle for logical position
        cur = self._gauge_current_deg
        if cur is not None:
            rad_c = math.radians(cur - 90)  # -90 so 0=top
            nx = cx + (r - 14) * math.cos(rad_c)
            ny = cy + (r - 14) * math.sin(rad_c)
            c.create_line(cx, cy, nx, ny, fill="blue", width=2, arrow="last")

        # Orange needle for Absolute Encoder Position (if available)
        if self._gauge_abs_deg is not None:
            rad_a = math.radians(self._gauge_abs_deg - 90)
            ax = cx + (r - 20) * math.cos(rad_a)
            ay = cy + (r - 20) * math.sin(rad_a)
            c.create_line(cx, cy, ax, ay, fill="orange", width=2, dash=(4,2), arrow="last")

        # Center dot
        c.create_oval(cx - 3, cy - 3, cx + 3, cy + 3, fill="blue")

    def _gauge_tick(self):
        try:
            now = time.time()
            if getattr(self, "_last_telemetry_time", 0.0) > 0 and (now - self._last_telemetry_time > 2.0):
                self.lbl_current_angle.config(text="Disconnected", foreground="red")
                self.lbl_current_rpm.config(text="Disconnected", foreground="red")
                self.lbl_abs_pos.config(text="Disconnected", foreground="red")
                self._gauge_current_deg = None
                self._gauge_abs_deg = None
            self._draw_gauge()
        except Exception:
            pass
        self._gauge_canvas.after(250, self._gauge_tick)

class WristDiagnosticsTab:
    def __init__(self, parent):
        self.frame = ttk.Frame(parent, padding=10)

        lf = ttk.LabelFrame(self.frame, text="Differential Wrist Output Kinematics", padding=15)
        lf.pack(fill="x", pady=5)

        ttk.Label(lf, text="True Pitch (M5 + M6)/2 :").grid(row=0, column=0, sticky="e", padx=5, pady=5)
        self.lbl_pitch = ttk.Label(lf, text="---", foreground="purple", font=("Arial", 14, "bold"))
        self.lbl_pitch.grid(row=0, column=1, sticky="w", pady=5)

        ttk.Label(lf, text="True Roll (M5 - M6)/2 :").grid(row=1, column=0, sticky="e", padx=5, pady=5)
        self.lbl_roll = ttk.Label(lf, text="---", foreground="teal", font=("Arial", 14, "bold"))
        self.lbl_roll.grid(row=1, column=1, sticky="w", pady=5)

        ttk.Label(lf, text="* Calculated automatically from Absolute Encoders mounted to Motor 5 and Motor 6.\n* If Pitch and Roll behave inverted, swap the + and - in ik_arm_control.py", font=("Arial", 9, "italic")).grid(row=2, column=0, columnspan=2, sticky="w", pady=10)

        self.m5_apos = None
        self.m6_apos = None

    def update_values(self):
        if self.m5_apos is not None and self.m6_apos is not None:
            pitch = (self.m5_apos + self.m6_apos) / 2.0
            roll = (self.m5_apos - self.m6_apos) / 2.0
            self.lbl_pitch.config(text=f"{pitch:.1f}°")
            self.lbl_roll.config(text=f"{roll:.1f}°")
        else:
            self.lbl_pitch.config(text="N/A")
            self.lbl_roll.config(text="N/A")

class WristTuningTab:
    def __init__(self, parent, app, config_data, send_cmd_func):
        self.app = app
        self.config = config_data
        self.send_cmd = send_cmd_func
        self.frame = ttk.Frame(parent, padding=10)
        self._ensure_defaults()

        tele = ttk.LabelFrame(self.frame, text="Wrist Differential Telemetry", padding=10)
        tele.pack(fill="x", pady=5)

        ttk.Label(tele, text="Pitch:", font=("Arial", 10, "bold")).grid(row=0, column=0, sticky="e", padx=5, pady=2)
        self.lbl_pitch = ttk.Label(tele, text="---", foreground="purple", font=("Arial", 10, "bold"))
        self.lbl_pitch.grid(row=0, column=1, sticky="w")
        ttk.Label(tele, text="Target:").grid(row=0, column=2, sticky="e", padx=(18, 5))
        self.lbl_pitch_target = ttk.Label(tele, text="---")
        self.lbl_pitch_target.grid(row=0, column=3, sticky="w")
        ttk.Label(tele, text="Error:").grid(row=0, column=4, sticky="e", padx=(18, 5))
        self.lbl_pitch_error = ttk.Label(tele, text="---")
        self.lbl_pitch_error.grid(row=0, column=5, sticky="w")

        ttk.Label(tele, text="Roll:", font=("Arial", 10, "bold")).grid(row=1, column=0, sticky="e", padx=5, pady=2)
        self.lbl_roll = ttk.Label(tele, text="---", foreground="teal", font=("Arial", 10, "bold"))
        self.lbl_roll.grid(row=1, column=1, sticky="w")
        ttk.Label(tele, text="Target:").grid(row=1, column=2, sticky="e", padx=(18, 5))
        self.lbl_roll_target = ttk.Label(tele, text="---")
        self.lbl_roll_target.grid(row=1, column=3, sticky="w")
        ttk.Label(tele, text="Error:").grid(row=1, column=4, sticky="e", padx=(18, 5))
        self.lbl_roll_error = ttk.Label(tele, text="---")
        self.lbl_roll_error.grid(row=1, column=5, sticky="w")

        ttk.Label(tele, text="Motor A cmd:").grid(row=2, column=0, sticky="e", padx=5, pady=(8, 2))
        self.lbl_motor_a = ttk.Label(tele, text="---")
        self.lbl_motor_a.grid(row=2, column=1, sticky="w")
        ttk.Label(tele, text="Motor B cmd:").grid(row=2, column=2, sticky="e", padx=(18, 5), pady=(8, 2))
        self.lbl_motor_b = ttk.Label(tele, text="---")
        self.lbl_motor_b.grid(row=2, column=3, sticky="w")

        ctrl = ttk.LabelFrame(self.frame, text="Wrist Differential Target Control", padding=10)
        ctrl.pack(fill="x", pady=5)

        self.var_pitch_target = tk.DoubleVar(value=0.0)
        self.var_roll_target = tk.DoubleVar(value=0.0)
        self.var_step = tk.DoubleVar(value=5.0)

        ttk.Label(ctrl, text="Pitch Target (deg)").grid(row=0, column=0, sticky="e", padx=5, pady=3)
        ttk.Entry(ctrl, textvariable=self.var_pitch_target, width=10).grid(row=0, column=1, sticky="w", padx=5, pady=3)
        ttk.Button(ctrl, text="Set Pitch", command=self.set_pitch_target).grid(row=0, column=2, padx=4, pady=3)
        ttk.Button(ctrl, text="Pitch -", command=lambda: self.nudge_pitch(-1.0)).grid(row=0, column=3, padx=4, pady=3)
        ttk.Button(ctrl, text="Pitch +", command=lambda: self.nudge_pitch(1.0)).grid(row=0, column=4, padx=4, pady=3)

        ttk.Label(ctrl, text="Roll Target (deg)").grid(row=1, column=0, sticky="e", padx=5, pady=3)
        ttk.Entry(ctrl, textvariable=self.var_roll_target, width=10).grid(row=1, column=1, sticky="w", padx=5, pady=3)
        ttk.Button(ctrl, text="Set Roll", command=self.set_roll_target).grid(row=1, column=2, padx=4, pady=3)
        ttk.Button(ctrl, text="Roll -", command=lambda: self.nudge_roll(-1.0)).grid(row=1, column=3, padx=4, pady=3)
        ttk.Button(ctrl, text="Roll +", command=lambda: self.nudge_roll(1.0)).grid(row=1, column=4, padx=4, pady=3)

        ttk.Label(ctrl, text="Step (deg)").grid(row=2, column=0, sticky="e", padx=5, pady=(8, 3))
        ttk.Entry(ctrl, textvariable=self.var_step, width=10).grid(row=2, column=1, sticky="w", padx=5, pady=(8, 3))
        ttk.Button(ctrl, text="Set Current As Target", command=self.capture_current_targets).grid(row=2, column=2, padx=4, pady=(8, 3))
        ttk.Button(ctrl, text="Set Both", command=self.set_both_targets).grid(row=2, column=3, padx=4, pady=(8, 3))
        ttk.Button(ctrl, text="Clear Targets", command=self.clear_targets).grid(row=2, column=4, padx=4, pady=(8, 3))

        cfg = ttk.LabelFrame(self.frame, text="Wrist Differential Tuning", padding=10)
        cfg.pack(fill="x", pady=5)

        self.var_enabled = tk.BooleanVar(value=bool(self.config.get("enabled", True)))
        self.var_pitch_enc = tk.StringVar(value=("None" if self.config.get("pitch_encoder_index") is None else str(int(self.config.get("pitch_encoder_index")))))
        self.var_roll_enc = tk.StringVar(value=("None" if self.config.get("roll_encoder_index") is None else str(int(self.config.get("roll_encoder_index")))))
        self.var_pitch_kp = tk.DoubleVar(value=float(self.config["pid"].get("pitch_kp", 4.0)))
        self.var_pitch_ki = tk.DoubleVar(value=float(self.config["pid"].get("pitch_ki", 0.0)))
        self.var_pitch_kd = tk.DoubleVar(value=float(self.config["pid"].get("pitch_kd", 0.0)))
        self.var_roll_kp = tk.DoubleVar(value=float(self.config["pid"].get("roll_kp", 4.0)))
        self.var_roll_ki = tk.DoubleVar(value=float(self.config["pid"].get("roll_ki", 0.0)))
        self.var_roll_kd = tk.DoubleVar(value=float(self.config["pid"].get("roll_kd", 0.0)))
        self.var_max_axis_speed = tk.DoubleVar(value=float(self.config["pid"].get("max_axis_speed", 80.0)))
        self.var_max_motor_speed = tk.DoubleVar(value=float(self.config["pid"].get("max_motor_speed", 120.0)))
        self.var_deadband = tk.DoubleVar(value=float(self.config["pid"].get("deadband_deg", 0.4)))
        self.var_int_limit = tk.DoubleVar(value=float(self.config["pid"].get("integral_limit", 30.0)))

        ttk.Checkbutton(cfg, text="Enable wrist differential", variable=self.var_enabled, command=self.on_change).grid(row=0, column=0, columnspan=2, sticky="w", pady=(0, 6))
        ttk.Label(cfg, text="Pitch Encoder Slot").grid(row=1, column=0, sticky="e", padx=5, pady=3)
        cb_pitch = ttk.Combobox(cfg, textvariable=self.var_pitch_enc, values=["None"] + [str(i) for i in range(8)], state="readonly", width=8)
        cb_pitch.grid(row=1, column=1, sticky="w", padx=5, pady=3)
        cb_pitch.bind("<<ComboboxSelected>>", self.on_change)
        ttk.Label(cfg, text="Roll Encoder Slot").grid(row=1, column=2, sticky="e", padx=5, pady=3)
        cb_roll = ttk.Combobox(cfg, textvariable=self.var_roll_enc, values=["None"] + [str(i) for i in range(8)], state="readonly", width=8)
        cb_roll.grid(row=1, column=3, sticky="w", padx=5, pady=3)
        cb_roll.bind("<<ComboboxSelected>>", self.on_change)

        self._entry(cfg, "Pitch Kp", self.var_pitch_kp, 2, 0)
        self._entry(cfg, "Pitch Ki", self.var_pitch_ki, 2, 2)
        self._entry(cfg, "Pitch Kd", self.var_pitch_kd, 3, 0)
        self._entry(cfg, "Roll Kp", self.var_roll_kp, 3, 2)
        self._entry(cfg, "Roll Ki", self.var_roll_ki, 4, 0)
        self._entry(cfg, "Roll Kd", self.var_roll_kd, 4, 2)
        self._entry(cfg, "Max Axis Speed", self.var_max_axis_speed, 5, 0)
        self._entry(cfg, "Max Motor Speed", self.var_max_motor_speed, 5, 2)
        self._entry(cfg, "Deadband (deg)", self.var_deadband, 6, 0)
        self._entry(cfg, "Integral Limit", self.var_int_limit, 6, 2)

        btns = ttk.Frame(cfg)
        btns.grid(row=7, column=0, columnspan=4, sticky="w", pady=(8, 0))
        ttk.Button(btns, text="Apply Wrist Config", command=self.apply_config).pack(side="left", padx=(0, 6))
        ttk.Button(btns, text="Stop Wrist", command=self.stop_wrist).pack(side="left", padx=6)

    def _ensure_defaults(self):
        default = DEFAULT_CONFIG["wrist_differential"]
        for key, value in default.items():
            if key not in self.config:
                self.config[key] = json.loads(json.dumps(value)) if isinstance(value, dict) else value
        if not isinstance(self.config.get("mix"), dict):
            self.config["mix"] = dict(default["mix"])
        else:
            merged_mix = dict(default["mix"])
            merged_mix.update(self.config["mix"])
            self.config["mix"] = merged_mix
        if not isinstance(self.config.get("pid"), dict):
            self.config["pid"] = dict(default["pid"])
        else:
            merged_pid = dict(default["pid"])
            merged_pid.update(self.config["pid"])
            self.config["pid"] = merged_pid

    def _entry(self, parent, label, variable, row, col):
        ttk.Label(parent, text=label).grid(row=row, column=col, sticky="e", padx=5, pady=3)
        entry = ttk.Entry(parent, textvariable=variable, width=10)
        entry.grid(row=row, column=col + 1, sticky="w", padx=5, pady=3)
        entry.bind("<KeyRelease>", self.on_change)

    def on_change(self, *args):
        self.config["enabled"] = bool(self.var_enabled.get())
        for cfg_key, widget_val in (("pitch_encoder_index", self.var_pitch_enc.get()), ("roll_encoder_index", self.var_roll_enc.get())):
            try:
                token = str(widget_val).strip()
                self.config[cfg_key] = None if token.lower() == "none" or token == "" else int(token)
            except Exception:
                self.config[cfg_key] = None
        try:
            self.config["pid"]["pitch_kp"] = float(self.var_pitch_kp.get())
            self.config["pid"]["pitch_ki"] = float(self.var_pitch_ki.get())
            self.config["pid"]["pitch_kd"] = float(self.var_pitch_kd.get())
            self.config["pid"]["roll_kp"] = float(self.var_roll_kp.get())
            self.config["pid"]["roll_ki"] = float(self.var_roll_ki.get())
            self.config["pid"]["roll_kd"] = float(self.var_roll_kd.get())
            self.config["pid"]["max_axis_speed"] = float(self.var_max_axis_speed.get())
            self.config["pid"]["max_motor_speed"] = float(self.var_max_motor_speed.get())
            self.config["pid"]["deadband_deg"] = float(self.var_deadband.get())
            self.config["pid"]["integral_limit"] = float(self.var_int_limit.get())
        except Exception:
            pass

    def apply_config(self):
        self.on_change()
        self.app.apply_wrist_settings()

    def stop_wrist(self):
        self.app.clear_wrist_targets(send_stop=True)

    def set_pitch_target(self):
        self.app.set_wrist_targets(pitch_target_deg=self.var_pitch_target.get())

    def set_roll_target(self):
        self.app.set_wrist_targets(roll_target_deg=self.var_roll_target.get())

    def set_both_targets(self):
        self.app.set_wrist_targets(
            pitch_target_deg=self.var_pitch_target.get(),
            roll_target_deg=self.var_roll_target.get(),
        )

    def capture_current_targets(self):
        telemetry = self.app.get_wrist_telemetry_snapshot()
        pitch = telemetry.get("pitch")
        roll = telemetry.get("roll")
        if pitch is not None:
            self.var_pitch_target.set(float(pitch))
        if roll is not None:
            self.var_roll_target.set(float(roll))
        self.set_both_targets()

    def clear_targets(self):
        self.app.clear_wrist_targets(send_stop=True)

    def nudge_pitch(self, direction):
        step = abs(float(self.var_step.get()))
        base = self.app.get_wrist_target("pitch")
        if base is None:
            base = self.app.get_wrist_telemetry_snapshot().get("pitch")
        base = 0.0 if base is None else float(base)
        updated = base + (direction * step)
        self.var_pitch_target.set(updated)
        self.app.set_wrist_targets(pitch_target_deg=updated)

    def nudge_roll(self, direction):
        step = abs(float(self.var_step.get()))
        base = self.app.get_wrist_target("roll")
        if base is None:
            base = self.app.get_wrist_telemetry_snapshot().get("roll")
        base = 0.0 if base is None else float(base)
        updated = base + (direction * step)
        self.var_roll_target.set(updated)
        self.app.set_wrist_targets(roll_target_deg=updated)

    def update_values(self, telemetry):
        pitch = telemetry.get("pitch")
        roll = telemetry.get("roll")
        pitch_target = telemetry.get("pitch_target")
        roll_target = telemetry.get("roll_target")
        motor_a = telemetry.get("motor_a_cmd")
        motor_b = telemetry.get("motor_b_cmd")
        self.lbl_pitch.config(text=(f"{pitch:.1f}°" if pitch is not None else "N/A"))
        self.lbl_roll.config(text=(f"{roll:.1f}°" if roll is not None else "N/A"))
        self.lbl_pitch_target.config(text=(f"{pitch_target:.1f}°" if pitch_target is not None else "---"))
        self.lbl_roll_target.config(text=(f"{roll_target:.1f}°" if roll_target is not None else "---"))
        self.lbl_pitch_error.config(text=(f"{(pitch_target - pitch):.1f}°" if pitch is not None and pitch_target is not None else "---"))
        self.lbl_roll_error.config(text=(f"{(roll_target - roll):.1f}°" if roll is not None and roll_target is not None else "---"))
        self.lbl_motor_a.config(text=(f"{motor_a:.1f} RPM" if motor_a is not None else "---"))
        self.lbl_motor_b.config(text=(f"{motor_b:.1f} RPM" if motor_b is not None else "---"))
        if pitch_target is not None:
            self.var_pitch_target.set(float(pitch_target))
        if roll_target is not None:
            self.var_roll_target.set(float(roll_target))

class SpeedTunerTab:
    def __init__(self, parent, send_cmd):
        self.frame = ttk.Frame(parent, padding=10)
        self.send_cmd = send_cmd

        self.var_motor = tk.IntVar(value=1)
        self.var_skp = tk.DoubleVar(value=0.30)
        self.var_ski = tk.DoubleVar(value=0.00)
        self.var_skd = tk.DoubleVar(value=0.00)
        self.var_sol = tk.DoubleVar(value=12.0)
        self.var_target = tk.DoubleVar(value=120.0)
        self.var_hold = tk.DoubleVar(value=1.5)
        self.var_sweep_start = tk.DoubleVar(value=0.0)
        self.var_sweep_end = tk.DoubleVar(value=240.0)
        self.var_sweep_step = tk.DoubleVar(value=40.0)
        self.var_sweep_dwell = tk.DoubleVar(value=0.8)
        self.var_open_loop = tk.BooleanVar(value=False)

        self.telemetry = {i: deque(maxlen=1800) for i in range(1, 9)}
        self.step_active = False
        self.step_motor = 1
        self.step_target = 0.0
        self.step_t0 = 0.0
        self.stop_sweep = False
        self.sweep_thread = None

        self._build_ui()
        self.frame.after(120, self._update_plot)

    def _build_ui(self):
        gains = ttk.LabelFrame(self.frame, text="Speed PID", padding=10)
        gains.pack(fill="x", pady=(0, 8))
        ttk.Label(gains, text="Motor").grid(row=0, column=0, sticky="w")
        ttk.Spinbox(gains, from_=1, to=8, textvariable=self.var_motor, width=6).grid(row=0, column=1, padx=4, sticky="w")
        ttk.Label(gains, text="SKP").grid(row=0, column=2, padx=(10, 2), sticky="w")
        ttk.Entry(gains, textvariable=self.var_skp, width=10).grid(row=0, column=3, sticky="w")
        ttk.Label(gains, text="SKI").grid(row=0, column=4, padx=(10, 2), sticky="w")
        ttk.Entry(gains, textvariable=self.var_ski, width=10).grid(row=0, column=5, sticky="w")
        ttk.Label(gains, text="SKD").grid(row=0, column=6, padx=(10, 2), sticky="w")
        ttk.Entry(gains, textvariable=self.var_skd, width=10).grid(row=0, column=7, sticky="w")
        ttk.Label(gains, text="SOL (A)").grid(row=0, column=8, padx=(10, 2), sticky="w")
        ttk.Entry(gains, textvariable=self.var_sol, width=10).grid(row=0, column=9, sticky="w")
        ttk.Button(gains, text="Apply Speed Gains", command=self.apply_gains).grid(row=0, column=10, padx=8)
        ttk.Checkbutton(gains, text="Bypass Speed PID (MV open-loop)", variable=self.var_open_loop).grid(row=1, column=0, columnspan=6, sticky="w", pady=(6, 0))

        tests = ttk.LabelFrame(self.frame, text="Step/Sweep Tests", padding=10)
        tests.pack(fill="x", pady=(0, 8))
        ttk.Label(tests, text="Target RPM").grid(row=0, column=0, sticky="w")
        ttk.Entry(tests, textvariable=self.var_target, width=10).grid(row=0, column=1, padx=4, sticky="w")
        ttk.Button(tests, text="Step +", command=lambda: self.start_step(abs(self.var_target.get()))).grid(row=0, column=2, padx=4)
        ttk.Button(tests, text="Step -", command=lambda: self.start_step(-abs(self.var_target.get()))).grid(row=0, column=3, padx=4)
        ttk.Button(tests, text="Stop Motor", command=self.stop_motor).grid(row=0, column=4, padx=4)

        ttk.Label(tests, text="Hold (s)").grid(row=1, column=0, sticky="w", pady=(8, 0))
        ttk.Entry(tests, textvariable=self.var_hold, width=10).grid(row=1, column=1, padx=4, sticky="w", pady=(8, 0))
        ttk.Button(tests, text="Run Timed Step", command=self.run_timed_step).grid(row=1, column=2, padx=4, pady=(8, 0))

        ttk.Label(tests, text="Sweep Start").grid(row=2, column=0, sticky="w", pady=(8, 0))
        ttk.Entry(tests, textvariable=self.var_sweep_start, width=10).grid(row=2, column=1, padx=4, sticky="w", pady=(8, 0))
        ttk.Label(tests, text="Sweep End").grid(row=2, column=2, sticky="w", pady=(8, 0))
        ttk.Entry(tests, textvariable=self.var_sweep_end, width=10).grid(row=2, column=3, padx=4, sticky="w", pady=(8, 0))
        ttk.Label(tests, text="Step").grid(row=2, column=4, sticky="w", pady=(8, 0))
        ttk.Entry(tests, textvariable=self.var_sweep_step, width=10).grid(row=2, column=5, padx=4, sticky="w", pady=(8, 0))
        ttk.Label(tests, text="Dwell (s)").grid(row=2, column=6, sticky="w", pady=(8, 0))
        ttk.Entry(tests, textvariable=self.var_sweep_dwell, width=10).grid(row=2, column=7, padx=4, sticky="w", pady=(8, 0))
        ttk.Button(tests, text="Run Sweep", command=self.start_sweep).grid(row=2, column=8, padx=6, pady=(8, 0))
        ttk.Button(tests, text="Stop Sweep", command=self.stop_sweep_test).grid(row=2, column=9, padx=6, pady=(8, 0))

        self.lbl_metrics = ttk.Label(self.frame, text="Rise: --  Overshoot: --  Settling: --  SSE: --", font=("Consolas", 10))
        self.lbl_metrics.pack(anchor="w", pady=(2, 6))

        fig = Figure(figsize=(8.2, 3.6), dpi=100)
        self.ax_rpm = fig.add_subplot(211)
        self.ax_cur = fig.add_subplot(212, sharex=self.ax_rpm)
        self.ax_rpm.set_ylabel("RPM")
        self.ax_cur.set_ylabel("A")
        self.ax_cur.set_xlabel("Time (s)")
        self.ax_rpm.grid(True, alpha=0.25)
        self.ax_cur.grid(True, alpha=0.25)
        self.line_rpm, = self.ax_rpm.plot([], [], color="tab:blue", linewidth=1.7, label="Actual RPM")
        self.line_target, = self.ax_rpm.plot([], [], color="tab:orange", linewidth=1.2, linestyle="--", label="Target")
        self.line_cur, = self.ax_cur.plot([], [], color="tab:green", linewidth=1.4, label="Current")
        self.ax_rpm.legend(loc="upper left")
        self.ax_cur.legend(loc="upper left")
        self.canvas = FigureCanvasTkAgg(fig, master=self.frame)
        self.canvas.get_tk_widget().pack(fill="both", expand=True)

    def apply_gains(self):
        mid = int(self.var_motor.get())
        cmd = (
            f"SKP {mid} {self.var_skp.get():.4f}; SKI {mid} {self.var_ski.get():.4f}; "
            f"SKD {mid} {self.var_skd.get():.4f}; SOL {mid} {self.var_sol.get():.2f}"
        )
        self.send_cmd(cmd)

    def start_step(self, target_rpm):
        mid = int(self.var_motor.get())
        cmd = "MV" if self.var_open_loop.get() else "M"
        self.send_cmd(f"{cmd} {mid} {target_rpm:.2f}")
        self.step_active = True
        self.step_motor = mid
        self.step_target = float(target_rpm)
        self.step_t0 = time.time()

    def stop_motor(self):
        mid = int(self.var_motor.get())
        cmd = "MV" if self.var_open_loop.get() else "M"
        self.send_cmd(f"{cmd} {mid} 0")
        self.step_active = False

    def run_timed_step(self):
        hold = max(0.1, float(self.var_hold.get()))
        self.start_step(self.var_target.get())

        def _worker():
            time.sleep(hold)
            self.frame.after(0, self.stop_motor)

        threading.Thread(target=_worker, daemon=True).start()

    def start_sweep(self):
        if self.sweep_thread and self.sweep_thread.is_alive():
            return
        self.stop_sweep = False
        self.sweep_thread = threading.Thread(target=self._sweep_worker, daemon=True)
        self.sweep_thread.start()

    def stop_sweep_test(self):
        self.stop_sweep = True

    def _sweep_worker(self):
        start = float(self.var_sweep_start.get())
        end = float(self.var_sweep_end.get())
        step = abs(float(self.var_sweep_step.get()))
        dwell = max(0.1, float(self.var_sweep_dwell.get()))
        if step <= 0.0:
            return
        direction = 1.0 if end >= start else -1.0
        val = start
        while not self.stop_sweep:
            self.frame.after(0, lambda rpm=val: self.start_step(rpm))
            t0 = time.time()
            while time.time() - t0 < dwell:
                if self.stop_sweep:
                    break
                time.sleep(0.02)
            if self.stop_sweep:
                break
            if direction > 0:
                if val + step > end:
                    break
                val += step
            else:
                if val - step < end:
                    break
                val -= step
        self.frame.after(0, self.stop_motor)

    def update_telemetry(self, mid, rpm, amps):
        if rpm is None:
            return
        self.telemetry[mid].append((time.time(), rpm, amps if amps is not None else 0.0))

    def _compute_metrics(self, points):
        if not self.step_active or int(self.var_motor.get()) != self.step_motor or not points:
            return "Rise: --  Overshoot: --  Settling: --  SSE: --"
        if abs(self.step_target) < 1e-6:
            return "Rise: --  Overshoot: --  Settling: --  SSE: --"

        post = [(t - self.step_t0, r) for (t, r, _a) in points if t >= self.step_t0]
        if len(post) < 4:
            return "Rise: --  Overshoot: --  Settling: --  SSE: --"

        times = [p[0] for p in post]
        vals = [p[1] for p in post]
        target_abs = abs(self.step_target)
        norm = [math.copysign(v, self.step_target) for v in vals]

        y10 = 0.1 * target_abs
        y90 = 0.9 * target_abs
        t10 = next((t for t, y in zip(times, norm) if y >= y10), None)
        t90 = next((t for t, y in zip(times, norm) if y >= y90), None)
        rise = (t90 - t10) if (t10 is not None and t90 is not None and t90 >= t10) else None

        peak = max(norm)
        overshoot = max(0.0, (peak - target_abs) / target_abs * 100.0)

        band = 0.05 * target_abs
        settling = None
        for i in range(len(norm)):
            if all(abs(target_abs - y) <= band for y in norm[i:]):
                settling = times[i]
                break

        recent = [target_abs - y for (t, y) in zip(times, norm) if times[-1] - t <= 1.0]
        sse = (sum(recent) / len(recent)) if recent else None

        fmt = lambda v, u: "--" if v is None else f"{v:.3f}{u}"
        return (
            f"Rise: {fmt(rise, 's')}  Overshoot: {overshoot:.1f}%  "
            f"Settling: {fmt(settling, 's')}  SSE: {fmt(sse, ' rpm')}"
        )

    def _update_plot(self):
        mid = int(self.var_motor.get())
        points = list(self.telemetry.get(mid, []))
        if points:
            t_ref = points[-1][0]
            x = [t - t_ref for (t, _r, _a) in points]
            y_r = [r for (_t, r, _a) in points]
            y_a = [a for (_t, _r, a) in points]
            y_t = [self.step_target if self.step_active and self.step_motor == mid else 0.0 for _ in x]
            self.line_rpm.set_data(x, y_r)
            self.line_cur.set_data(x, y_a)
            self.line_target.set_data(x, y_t)
            self.ax_rpm.set_xlim(min(x), 0.0)
            y_all = y_r + y_t
            ymin = min(y_all) if y_all else -1.0
            ymax = max(y_all) if y_all else 1.0
            span = max(20.0, ymax - ymin)
            self.ax_rpm.set_ylim(ymin - 0.2 * span, ymax + 0.2 * span)
            amin = min(y_a) if y_a else -1.0
            amax = max(y_a) if y_a else 1.0
            aspan = max(1.0, amax - amin)
            self.ax_cur.set_ylim(amin - 0.2 * aspan, amax + 0.2 * aspan)
            self.lbl_metrics.config(text=self._compute_metrics(points))
        else:
            self.line_rpm.set_data([], [])
            self.line_cur.set_data([], [])
            self.line_target.set_data([], [])
            self.lbl_metrics.config(text="Rise: --  Overshoot: --  Settling: --  SSE: --")

        self.canvas.draw_idle()
        self.frame.after(120, self._update_plot)

class PIDTunerApp:
    def __init__(self, root):
        self.root = root
        self.root.title("RoboMaster PID & Configuration Tuner")
        self.root.geometry("800x650")

        self.config = self.load_config()
        self.controller_map = self.config.get("controllers", self._controller_defaults())
        self.ser = None
        self.running = True
        self.tabs = {}
        self.controller_ports = {"base": None, "joint4": None, "wrist": None}
        self.controller_serial = {"base": None, "joint4": None, "wrist": None}
        self.controller_read_threads = {"base": None, "joint4": None, "wrist": None}
        self.controller_tx_threads = {"base": None, "joint4": None, "wrist": None}
        self.controller_cmd_queues = {"base": queue.Queue(), "joint4": queue.Queue(), "wrist": queue.Queue()}
        self.controller_last_line = {"base": "", "joint4": "", "wrist": ""}
        self.latest_telemetry = {}
        self._abspid_skip_last_warn = {}
        self.wrist_diff_target_logical_deg = {"pitch": None, "roll": None}
        self.wrist_diff_pid_state = {
            "pitch": {"integral": 0.0, "prev_error": 0.0},
            "roll": {"integral": 0.0, "prev_error": 0.0},
        }
        self.wrist_diff_last_update_time = 0.0
        self.wrist_diff_last_motor_cmd = (None, None)

        self.create_widgets()
        self.root.after(20, self._wrist_control_tick)

    def get_wrist_target(self, axis_name):
        return self.wrist_diff_target_logical_deg.get(axis_name)

    def reset_wrist_diff_pid_state(self):
        for axis_name in ("pitch", "roll"):
            self.wrist_diff_pid_state[axis_name]["integral"] = 0.0
            self.wrist_diff_pid_state[axis_name]["prev_error"] = 0.0
        self.wrist_diff_last_update_time = 0.0
        self.wrist_diff_last_motor_cmd = (None, None)

    def clear_wrist_targets(self, send_stop=False):
        self.wrist_diff_target_logical_deg["pitch"] = None
        self.wrist_diff_target_logical_deg["roll"] = None
        self.reset_wrist_diff_pid_state()
        if send_stop:
            cfg = self.config.get("wrist_differential", {})
            motor_a_id = int(cfg.get("motor_a_id", 5))
            motor_b_id = int(cfg.get("motor_b_id", 6))
            self.send_command(f"M {motor_a_id} 0; M {motor_b_id} 0")

    def set_wrist_targets(self, pitch_target_deg=None, roll_target_deg=None):
        if pitch_target_deg is not None:
            self.wrist_diff_target_logical_deg["pitch"] = float(pitch_target_deg)
        if roll_target_deg is not None:
            self.wrist_diff_target_logical_deg["roll"] = float(roll_target_deg)
        self.reset_wrist_diff_pid_state()

    def _wrist_control_tick(self):
        try:
            self.update_wrist_diff_controller_once()
        except Exception:
            pass
        if self.running:
            self.root.after(20, self._wrist_control_tick)

    def update_wrist_diff_controller_once(self):
        cfg = self.config.get("wrist_differential", {})
        if not bool(cfg.get("enabled", False)):
            return False
        if not self._connected_controller_names():
            return False
        pitch_target = self.wrist_diff_target_logical_deg.get("pitch")
        roll_target = self.wrist_diff_target_logical_deg.get("roll")
        if pitch_target is None and roll_target is None:
            return False

        telemetry = self.get_wrist_telemetry_snapshot()
        pitch_actual = telemetry.get("pitch")
        roll_actual = telemetry.get("roll")
        if pitch_actual is None or roll_actual is None:
            return False

        pid_cfg = cfg.get("pid", {}) if isinstance(cfg.get("pid"), dict) else {}
        now = time.time()
        if self.wrist_diff_last_update_time > 0.0:
            dt = clampf(now - self.wrist_diff_last_update_time, 0.01, 0.10)
        else:
            dt = 0.02
        self.wrist_diff_last_update_time = now

        axis_cmd = {}
        all_in_deadband = True
        for axis_name, actual_val, target_val in (
            ("pitch", pitch_actual, pitch_target),
            ("roll", roll_actual, roll_target),
        ):
            if target_val is None:
                axis_cmd[axis_name] = 0.0
                continue
            err = float(target_val) - float(actual_val)
            deadband = abs(float(pid_cfg.get("deadband_deg", 0.4)))
            if abs(err) > deadband:
                all_in_deadband = False
            state = self.wrist_diff_pid_state[axis_name]
            kp = float(pid_cfg.get(f"{axis_name}_kp", 4.0))
            ki = float(pid_cfg.get(f"{axis_name}_ki", 0.0))
            kd = float(pid_cfg.get(f"{axis_name}_kd", 0.0))
            int_limit = abs(float(pid_cfg.get("integral_limit", 30.0)))
            state["integral"] += err * dt
            state["integral"] = clampf(state["integral"], -int_limit, int_limit)
            deriv = (err - state["prev_error"]) / dt if dt > 1e-6 else 0.0
            state["prev_error"] = err
            cmd = (kp * err) + (ki * state["integral"]) + (kd * deriv)
            max_axis_speed = abs(float(pid_cfg.get("max_axis_speed", 80.0)))
            axis_cmd[axis_name] = clampf(cmd, -max_axis_speed, max_axis_speed)

        mix = cfg.get("mix", {}) if isinstance(cfg.get("mix"), dict) else {}
        motor_a_id = int(cfg.get("motor_a_id", 5))
        motor_b_id = int(cfg.get("motor_b_id", 6))
        motor_a_cmd = (
            axis_cmd.get("pitch", 0.0) * float(mix.get("motor_a_pitch_sign", 1.0))
            + axis_cmd.get("roll", 0.0) * float(mix.get("motor_a_roll_sign", 1.0))
        )
        motor_b_cmd = (
            axis_cmd.get("pitch", 0.0) * float(mix.get("motor_b_pitch_sign", 1.0))
            + axis_cmd.get("roll", 0.0) * float(mix.get("motor_b_roll_sign", -1.0))
        )
        max_motor_speed = abs(float(pid_cfg.get("max_motor_speed", 120.0)))
        motor_a_cmd = clampf(motor_a_cmd, -max_motor_speed, max_motor_speed)
        motor_b_cmd = clampf(motor_b_cmd, -max_motor_speed, max_motor_speed)

        if all_in_deadband:
            motor_a_cmd = 0.0
            motor_b_cmd = 0.0

        prev_a, prev_b = self.wrist_diff_last_motor_cmd
        if prev_a is None or prev_b is None or abs(prev_a - motor_a_cmd) >= 0.5 or abs(prev_b - motor_b_cmd) >= 0.5:
            self.send_command(f"M {motor_a_id} {motor_a_cmd:.2f}; M {motor_b_id} {motor_b_cmd:.2f}")
            self.wrist_diff_last_motor_cmd = (motor_a_cmd, motor_b_cmd)
        return True

    def _controller_defaults(self):
        return {
            "base": {"port": None, "motors": [1, 2, 3]},
            "joint4": {"port": None, "motors": [4]},
            "wrist": {"port": None, "motors": [5, 6]},
        }

    def _sanitize_controllers(self, loaded):
        out = self._controller_defaults()
        if not isinstance(loaded, dict):
            return out
        for name in ("base", "joint4", "wrist"):
            raw = loaded.get(name, {}) if isinstance(loaded.get(name, {}), dict) else {}
            port = raw.get("port")
            out[name]["port"] = None if port in (None, "") else str(port)
            motors = raw.get("motors")
            if isinstance(motors, list):
                vals = []
                for mid in motors:
                    try:
                        vals.append(int(mid))
                    except Exception:
                        pass
                if vals:
                    out[name]["motors"] = vals
        return out

    def _controller_for_motor(self, mid):
        try:
            motor_id = int(mid)
        except Exception:
            return "base"
        for name, cfg in self.controller_map.items():
            if motor_id in set(cfg.get("motors", [])):
                return name
        return "base"

    def _connected_controller_names(self):
        return [name for name, conn in self.controller_serial.items() if conn and conn.is_open]

    def _is_any_controller_connected(self):
        return bool(self._connected_controller_names())

    def _sync_legacy_serial_ref(self):
        for name in ("base", "joint4", "wrist"):
            conn = self.controller_serial.get(name)
            if conn and conn.is_open:
                self.ser = conn
                return
        self.ser = None

    def _controller_has_fresh_telemetry(self, controller_name):
        cfg = self.controller_map.get(controller_name, {}) if isinstance(self.controller_map.get(controller_name, {}), dict) else {}
        motor_ids = cfg.get("motors", []) if isinstance(cfg.get("motors", []), list) else []
        now = time.time()
        for mid in motor_ids:
            tele = self.latest_telemetry.get(str(mid), {})
            ts = tele.get("time")
            if ts is None:
                continue
            try:
                if (now - float(ts)) <= 2.0:
                    return True
            except Exception:
                pass
        return False

    def _controller_owns_motor(self, controller_name, motor_id):
        cfg = self.controller_map.get(controller_name, {}) if isinstance(self.controller_map.get(controller_name, {}), dict) else {}
        assigned = cfg.get("motors", []) if isinstance(cfg.get("motors", []), list) else []
        if not assigned:
            return True
        assigned_ids = set()
        for mid in assigned:
            try:
                assigned_ids.add(int(mid))
            except Exception:
                pass
        if not assigned_ids:
            return True
        try:
            return int(motor_id) in assigned_ids
        except Exception:
            return False

    def refresh_controller_port_status(self):
        parts = []
        for name in ("base", "joint4", "wrist"):
            configured = self.controller_map.get(name, {}).get("port")
            opened = self.controller_ports.get(name)
            conn = self.controller_serial.get(name)
            is_open = bool(conn and conn.is_open)
            live = self._controller_has_fresh_telemetry(name)
            state = "LIVE" if live else ("OPEN" if is_open else "OFF")
            shown_port = opened or configured or "-"
            extra = ""
            if configured and opened and configured != opened:
                extra = f" cfg:{configured}"
            parts.append(f"{name}:{shown_port} {state}{extra}")
        if hasattr(self, "lbl_controller_ports"):
            self.lbl_controller_ports.config(text="Controller Ports: " + " | ".join(parts))

    def _parse_command_segments(self, cmd):
        return [part.strip() for part in str(cmd).split(";") if part.strip()]

    def _route_segment(self, segment):
        toks = segment.split()
        if not toks:
            return set()
        op = toks[0].upper()
        if op in {"S", "TON", "TOFF", "T"}:
            return {"base", "joint4", "wrist"}
        if op in {"P", "PR", "M", "MV", "KP", "KI", "KD", "AOL", "SKP", "SKI", "SKD", "SOL", "ABSPID", "CAL", "JLIM", "GEAR"} and len(toks) >= 2:
            return {self._controller_for_motor(toks[1])}
        if op == "ENCREVSET":
            if len(toks) >= 4:
                return {self._controller_for_motor(toks[3])}
            return {"base", "joint4", "wrist"}
        if op == "ENCMAP":
            if len(toks) >= 3 and toks[2] != "0":
                return {self._controller_for_motor(toks[2])}
            return {"base", "joint4", "wrist"}
        return {"base", "joint4", "wrist"}

    def _route_command_packets(self, cmd):
        routed = {"base": [], "joint4": [], "wrist": []}
        for segment in self._parse_command_segments(cmd):
            for name in self._route_segment(segment):
                routed[name].append(segment)
        return {name: "; ".join(parts) for name, parts in routed.items() if parts}

    def _clear_pending_commands_for(self, controller_name=None):
        names = [controller_name] if controller_name else ["base", "joint4", "wrist"]
        for name in names:
            q = self.controller_cmd_queues[name]
            while not q.empty():
                try:
                    q.get_nowait()
                    q.task_done()
                except Exception:
                    break

    def _sanitize_motor_config(self, raw_motor, default_motor):
        cfg = dict(default_motor)
        if isinstance(raw_motor, dict):
            cfg.update(raw_motor)

        # Numeric sanitization so bad saved values do not brick runtime control.
        for key in ("gear_ratio", "kp", "ki", "kd", "aol", "skp", "ski", "skd", "sol", "min_angle", "max_angle", "zero_offset", "abs_zero_offset"):
            try:
                cfg[key] = float(cfg.get(key, default_motor.get(key, 0.0)))
            except Exception:
                cfg[key] = float(default_motor.get(key, 0.0))

        if cfg["gear_ratio"] <= 0:
            cfg["gear_ratio"] = float(default_motor["gear_ratio"])
        if cfg["aol"] <= 0:
            cfg["aol"] = float(default_motor["aol"])
        if cfg["sol"] <= 0:
            cfg["sol"] = float(default_motor["sol"])
        if cfg["skp"] < 0.05:
            cfg["skp"] = float(default_motor["skp"])
        if cfg["max_angle"] <= cfg["min_angle"]:
            cfg["min_angle"] = 0.0
            cfg["max_angle"] = 360.0

        if cfg.get("type") not in ("M2006", "M3508"):
            cfg["type"] = default_motor["type"]

        enc_idx = cfg.get("abs_encoder_index", None)
        if enc_idx is None or enc_idx == "None":
            cfg["abs_encoder_index"] = None
        else:
            try:
                cfg["abs_encoder_index"] = int(enc_idx)
            except Exception:
                cfg["abs_encoder_index"] = None
        if cfg["abs_encoder_index"] is not None and (cfg["abs_encoder_index"] < 0 or cfg["abs_encoder_index"] > 7):
            cfg["abs_encoder_index"] = None

        try:
            cfg["abs_encoder_reversed"] = bool(cfg.get("abs_encoder_reversed", True))
        except Exception:
            cfg["abs_encoder_reversed"] = True

        try:
            cfg["abs_pid_enabled"] = bool(cfg.get("abs_pid_enabled", False))
        except Exception:
            cfg["abs_pid_enabled"] = False

        return cfg

    def _sanitize_config(self, loaded):
        out = {"controllers": self._controller_defaults(), "motors": {}, "wrist_differential": json.loads(json.dumps(DEFAULT_CONFIG["wrist_differential"]))}
        loaded_motors = loaded.get("motors", {}) if isinstance(loaded, dict) else {}
        for i in range(1, 7):
            mid = str(i)
            default_motor = DEFAULT_CONFIG["motors"][mid]
            raw_motor = loaded_motors.get(mid, {}) if isinstance(loaded_motors, dict) else {}
            out["motors"][mid] = self._sanitize_motor_config(raw_motor, default_motor)
        out["controllers"] = self._sanitize_controllers(loaded.get("controllers", {}) if isinstance(loaded, dict) else {})
        loaded_wrist = loaded.get("wrist_differential", {}) if isinstance(loaded, dict) else {}
        if isinstance(loaded_wrist, dict):
            out["wrist_differential"].update({k: v for k, v in loaded_wrist.items() if k not in {"mix", "pid"}})
            if isinstance(loaded_wrist.get("mix"), dict):
                out["wrist_differential"]["mix"].update(loaded_wrist["mix"])
            if isinstance(loaded_wrist.get("pid"), dict):
                out["wrist_differential"]["pid"].update(loaded_wrist["pid"])
        return out

    def load_config(self):
        if os.path.exists(CONFIG_FILE):
            try:
                with open(CONFIG_FILE, 'r') as f:
                    loaded = json.load(f)
                return self._sanitize_config(loaded)
            except Exception as e:
                messagebox.showwarning("Warning", f"Failed to load config: {e}\nUsing defaults.")
        return self._sanitize_config(DEFAULT_CONFIG)

    def save_config(self):
        try:
            with open(CONFIG_FILE, 'w') as f:
                json.dump(self.config, f, indent=4)
            messagebox.showinfo("Success", "Configuration saved successfully!")
        except Exception as e:
            messagebox.showerror("Error", f"Failed to save config: {e}")

    def create_widgets(self):
        sw = self.root.winfo_screenwidth()
        sh = self.root.winfo_screenheight()
        w = max(900, int(sw * 0.80))
        h = max(650, int(sh * 0.85))
        self.root.geometry(f"{w}x{h}")
        self.root.minsize(820, 400)
        self.root.resizable(True, True)

        # --- Fixed bottom area (always visible) ---
        self.status_bar = ttk.Label(self.root,
            text="Ready. Note: Close 'ik_arm_control.py' before connecting via USB.",
            relief="sunken", anchor="w")
        self.status_bar.pack(fill="x", side="bottom", pady=(2, 0))

        bottom_frame = ttk.Frame(self.root, padding=(10, 5))
        bottom_frame.pack(fill="x", side="bottom")
        ttk.Button(bottom_frame, text="Save to File",             command=self.save_config).pack(side="left",  padx=5)
        ttk.Button(bottom_frame, text="Apply ALL Settings to Arm", command=self.apply_all).pack(side="right", padx=5)

        # --- Fixed top bar (always visible) ---
        top_frame = ttk.Frame(self.root, padding=10)
        top_frame.pack(fill="x", side="top")
        ttk.Label(top_frame, text="6-DoF Arm Configuration", font=("Arial", 16, "bold")).pack(side="left")
        self.btn_connect = ttk.Button(top_frame, text="Connect to Arm", command=self.toggle_serial)
        self.btn_connect.pack(side="right")
        self.lbl_status = ttk.Label(top_frame, text="Disconnected", foreground="red")
        self.lbl_status.pack(side="right", padx=10)

        self.lbl_controller_ports = ttk.Label(self.root, text="Controller Ports: base=-, joint4=-, wrist=-", anchor="w", justify="left")
        self.lbl_controller_ports.pack(fill="x", padx=10)

        quick_frame = ttk.Frame(self.root, padding=(10, 0, 10, 6))
        quick_frame.pack(fill="x", side="top")
        ttk.Button(quick_frame, text="Safe Bring-Up", command=self.safe_bringup).pack(side="left", padx=4)
        ttk.Button(quick_frame, text="STOP ALL", command=lambda: self.send_command("S")).pack(side="left", padx=4)
        ttk.Button(quick_frame, text="Apply Limits Only", command=self.apply_limits_all).pack(side="left", padx=4)
        ttk.Button(quick_frame, text="Load Conservative All", command=lambda: self.load_preset_all("Conservative")).pack(side="left", padx=4)
        ttk.Button(quick_frame, text="Apply Encoder Mappings", command=self.apply_encoder_mappings).pack(side="left", padx=4)

        # --- Scrollable canvas fills remaining space ---
        scroll_container = ttk.Frame(self.root)
        scroll_container.pack(fill="both", expand=True)

        vsb = ttk.Scrollbar(scroll_container, orient="vertical")
        vsb.pack(side="right", fill="y")

        self._canvas = tk.Canvas(scroll_container, yscrollcommand=vsb.set,
                                 bd=0, highlightthickness=0)
        self._canvas.pack(side="left", fill="both", expand=True)
        vsb.configure(command=self._canvas.yview)

        # Content frame inside canvas
        content = ttk.Frame(self._canvas)
        content_id = self._canvas.create_window((0, 0), window=content, anchor="nw")

        def _on_frame_cfg(e):
            self._canvas.configure(scrollregion=self._canvas.bbox("all"))
        content.bind("<Configure>", _on_frame_cfg)

        def _on_canvas_cfg(e):
            self._canvas.itemconfig(content_id, width=e.width)
        self._canvas.bind("<Configure>", _on_canvas_cfg)

        def _on_mousewheel(e):
            # Don't steal scroll from inner text widgets (e.g., log box)
            if isinstance(e.widget, tk.Text):
                return
            self._canvas.yview_scroll(int(-1 * (e.delta / 120)), "units")
        self._canvas.bind_all("<MouseWheel>", _on_mousewheel)

        # --- Notebook (full natural height — no expand) ---
        self.notebook = ttk.Notebook(content)
        self.notebook.pack(fill="x", padx=10, pady=5)

        for i in range(1, 7):
            mid = str(i)
            if mid not in self.config["motors"]:
                self.config["motors"][mid] = DEFAULT_CONFIG["motors"][mid].copy()
            tab = MotorTab(self.notebook, mid, self.config["motors"][mid], self.send_command)
            self.tabs[mid] = tab
            self.notebook.add(tab.frame, text=f"Motor {mid}")

        # --- Wrist Diagnostics Tab ---
        self.wrist_tab = WristDiagnosticsTab(self.notebook)
        self.notebook.add(self.wrist_tab.frame, text="Wrist Diagnostics")

        self.wrist_tuning_tab = WristTuningTab(self.notebook, self, self.config["wrist_differential"], self.send_command)
        self.notebook.add(self.wrist_tuning_tab.frame, text="Wrist Tuning")

        # --- Integrated Speed PID Tuner Tab ---
        self.speed_tuner_tab = SpeedTunerTab(self.notebook, self.send_command)
        self.notebook.add(self.speed_tuner_tab.frame, text="Speed Tuner")

        # --- Serial log (also scrollable with the page) ---
        lf_log = ttk.LabelFrame(content, text="Serial Log", padding=5)
        lf_log.pack(fill="x", padx=10, pady=(0, 10))

        log_top = ttk.Frame(lf_log)
        log_top.pack(fill="x")
        ttk.Label(log_top, text="Raw output from XIAO:", font=("Arial", 8)).pack(side="left")
        ttk.Button(log_top, text="Clear", command=self.clear_log).pack(side="right")

        self.log_box = scrolledtext.ScrolledText(
            lf_log, height=8, state="disabled", wrap="word",
            font=("Courier", 8), bg="#1e1e1e", fg="#d4d4d4",
            insertbackground="white"
        )
        self.log_box.pack(fill="x", pady=(4, 0))
        self.log_box.tag_config("telemetry", foreground="#6db3f2")
        self.log_box.tag_config("error",     foreground="#f44747")
        self.log_box.tag_config("info",      foreground="#b5cea8")
        self.log_box.tag_config("sent",      foreground="#dca54e")
        self._log_line_count = 0
        self.refresh_controller_port_status()

    def find_serial_port(self):
        ports = self.find_serial_ports()
        return ports[0] if ports else None

    def find_serial_ports(self):
        ports = list(serial.tools.list_ports.comports())
        preferred, fallback = [], []
        for port in ports:
            desc = f"{port.description} {port.device}".lower()
            if any(token in desc for token in ("ch340", "arduino", "serial", "usb")):
                preferred.append(port.device)
            else:
                fallback.append(port.device)
        ordered = []
        for device in preferred + fallback:
            if device not in ordered:
                ordered.append(device)
        return ordered

    def _ordered_controller_ports(self):
        available = self.find_serial_ports()
        ordered = []
        used = set()
        for name in ("base", "joint4", "wrist"):
            port = self.controller_map.get(name, {}).get("port")
            if port and port in available and port not in used:
                ordered.append((name, port))
                used.add(port)
        for name in ("base", "joint4", "wrist"):
            if any(existing_name == name for existing_name, _ in ordered):
                continue
            remaining = [port for port in available if port not in used]
            if not remaining:
                break
            ordered.append((name, remaining[0]))
            used.add(remaining[0])
        return ordered

    def disconnect_serial(self):
        """Helper to cleanly update the UI when the serial connection is lost or closed."""
        for name, conn in list(self.controller_serial.items()):
            if conn and conn.is_open:
                try:
                    conn.close()
                except Exception:
                    pass
            self.controller_serial[name] = None
            self.controller_ports[name] = None
            self.controller_read_threads[name] = None
            self.controller_tx_threads[name] = None
            self.controller_last_line[name] = ""
        self._clear_pending_commands_for()
        self._sync_legacy_serial_ref()
        self.clear_wrist_targets(send_stop=False)
        self.btn_connect.config(text="Connect to Arm")
        self.lbl_status.config(text="Disconnected", foreground="red")
        self.refresh_controller_port_status()
        self.status_bar.config(text="Disconnected from serial.")

    def toggle_serial(self):
        if self._connected_controller_names():
            self.disconnect_serial()
        else:
            try:
                assignments = self._ordered_controller_ports()
            except Exception as e:
                messagebox.showerror("Connection Error", f"Failed while enumerating serial ports:\n{e}")
                return

            if not assignments:
                messagebox.showerror("No Device", "Could not find a connected Arduino device.")
                return

            connected = []
            failures = []
            for name, port in assignments:
                try:
                    conn = serial.Serial(port, BAUD_RATE, timeout=0.1)
                    time.sleep(2)
                    self.controller_ports[name] = port
                    self.controller_serial[name] = conn
                    self.controller_read_threads[name] = threading.Thread(target=self.serial_read_loop, args=(name,), daemon=True)
                    self.controller_read_threads[name].start()
                    tx = threading.Thread(target=self.command_tx_loop, args=(name,), daemon=True)
                    self.controller_tx_threads[name] = tx
                    tx.start()
                    connected.append((name, port))
                except Exception as e:
                    failures.append((name, port, str(e)))

            if not connected:
                self.disconnect_serial()
                details = "\n".join(f"- {name}:{port} -> {err}" for name, port, err in failures) if failures else "No controllers opened."
                messagebox.showerror("Connection Error", f"Could not connect to any controllers:\n{details}")
                return

            self._sync_legacy_serial_ref()
            self.btn_connect.config(text="Disconnect")
            self.lbl_status.config(text=f"Connected ({', '.join(f'{name}:{port}' for name, port in connected)})", foreground="green")
            self.refresh_controller_port_status()

            # TON is idempotent (unlike T toggle) - safe to call on every connect
            self.send_command("TON")

            if failures:
                failed_names = ", ".join(f"{name}:{port}" for name, port, _ in failures)
                self.status_bar.config(text=f"Connected to {len(connected)} controller(s). Failed: {failed_names}")
            else:
                # Do not auto-apply file settings on connect.
                # Bad saved values can make motors appear dead until power cycle.
                self.status_bar.config(text="Connected to controller set. Live control ready. Use 'Apply ALL' only after reviewing values.")

    @staticmethod
    def _safe_float(value):
        try:
            f = float(value)
        except Exception:
            return None
        if math.isnan(f) or math.isinf(f):
            return None
        return f

    @staticmethod
    def _safe_int(value, default=None):
        try:
            return int(value)
        except Exception:
            return default

    def _handle_abspid_skip_warning(self, controller_name, line):
        m = re.search(r"M(\d+)\s+ABSPID\s+skipped:\s*(.+)$", line)
        if not m:
            return
        mid = m.group(1)
        reason = m.group(2).strip()
        now = time.time()
        warn_key = f"{controller_name}:{mid}:{reason}"
        last = self._abspid_skip_last_warn.get(warn_key, 0.0)
        if (now - last) < 2.0:
            return
        self._abspid_skip_last_warn[warn_key] = now

        hint = reason
        if "no encoder mapped" in reason.lower():
            hint = (
                "no encoder mapped. Set Abs Encoder Index for this motor, "
                "then click 'Apply Encoder Mappings' before enabling ABS PID."
            )
        self.status_bar.config(text=f"[{controller_name}] M{mid} ABSPID skipped: {hint}")

    def serial_read_loop(self, controller_name):
        conn = self.controller_serial.get(controller_name)
        while self.running and conn and conn.is_open:
            try:
                if conn.in_waiting > 0:
                    line = conn.readline().decode('utf-8', errors='ignore').strip()
                    if not line:
                        continue
                    self.controller_last_line[controller_name] = line

                    # Append raw line to the log panel (thread-safe)
                    self.root.after(0, self.log_append, f"[{controller_name}] {line}")

                    if "ABSPID skipped:" in line:
                        self.root.after(0, self._handle_abspid_skip_warning, controller_name, line)
                        continue

                    # Keep target labels synced even when commands originate
                    # outside this app (e.g. IK tool / serial monitor).
                    m_pr = re.search(r"^M(\d+)\s+PR\s+[-+]?\d*\.?\d+\s+->\s+log\s+([-+]?\d*\.?\d+)", line)
                    if m_pr:
                        mid = m_pr.group(1)
                        if not self._controller_owns_motor(controller_name, mid):
                            continue
                        tgt = self._safe_float(m_pr.group(2))
                        if tgt is not None:
                            self.root.after(0, self._set_target_from_firmware, mid, tgt)
                        continue

                    m_p = re.search(r"^M(\d+)\s+->\s+log\s+([-+]?\d*\.?\d+)", line)
                    if m_p:
                        mid = m_p.group(1)
                        if not self._controller_owns_motor(controller_name, mid):
                            continue
                        tgt = self._safe_float(m_p.group(2))
                        if tgt is not None:
                            self.root.after(0, self._set_target_from_firmware, mid, tgt)
                        continue

                    if line.startswith("STOPPED ALL MOTORS"):
                        self.root.after(0, self._sync_targets_to_current)
                        continue

                    # Parse status lines like:
                    # M1 POS:0.0 APOS:45.3 Tot:0.0 RPM:0.0 A:0.00 |
                    for token in line.split("|"):
                        token = token.strip()
                        if not token.startswith("M"):
                            continue
                        if ("POS:" not in token) and ("RPM:" not in token) and ("APOS:" not in token):
                            continue

                        mid_m = re.search(r"M(\d+)", token)
                        if not mid_m:
                            continue

                        pos_m = re.search(r"(?:^|\s)POS:([^\s|]+)", token)
                        apos_m = re.search(r"(?:^|\s)APOS:([^\s|]+)", token)
                        rpm_m = re.search(r"RPM:([^\s|]+)", token)
                        amp_m = re.search(r"A:([^\s|]+)", token)

                        mid = mid_m.group(1)
                        if not self._controller_owns_motor(controller_name, mid):
                            continue
                        pos = pos_m.group(1) if pos_m else None
                        apos = apos_m.group(1) if apos_m else None
                        rpm = rpm_m.group(1) if rpm_m else None
                        amps = amp_m.group(1) if amp_m else None

                        self.root.after(0, self.update_telemetry_ui, mid, pos, rpm, apos, amps)
                time.sleep(0.01)
            except (serial.SerialException, OSError) as e:
                # Device was likely disconnected (e.g. unplugged USB)
                self.root.after(0, self.status_bar.config, {"text": f"Serial port lost: {e}"})
                self.root.after(0, self.disconnect_serial)
                break
            except Exception:
                break

    def update_telemetry_ui(self, mid, pos, rpm="---", apos=None, amps=None):
        key = str(mid)
        prev = self.latest_telemetry.get(key, {})

        pos_f = self._safe_float(pos) if pos is not None else None
        rpm_f = self._safe_float(rpm) if rpm is not None else None
        apos_f = self._safe_float(apos) if apos is not None else None
        amp_f = self._safe_float(amps) if amps is not None else None

        # Some telemetry tokens omit APOS/POS intermittently. Keep last known
        # valid values instead of replacing with None to avoid UI flicker.
        if pos_f is None:
            pos_f = prev.get("pos")
        if rpm_f is None:
            rpm_f = prev.get("rpm")
        if apos_f is None:
            apos_f = prev.get("apos")
        if amp_f is None:
            amp_f = prev.get("amps")

        # Current Angle should track firmware logical POS (CAL / P / PR frame).
        # APOS is shown separately as raw absolute encoder position.
        display_deg = pos_f
        self.latest_telemetry[key] = {"pos": pos_f, "display_deg": display_deg, "rpm": rpm_f, "apos": apos_f, "amps": amp_f, "time": time.time()}
        self.refresh_controller_port_status()

        if mid in self.tabs:
            tab = self.tabs[mid]
            tab._last_telemetry_time = time.time()

            tab.lbl_current_angle.config(text=(f"{display_deg:.1f}°" if display_deg is not None else "N/A"), foreground="blue")
            tab.lbl_current_rpm.config(text=(f"{rpm_f:.1f} RPM" if rpm_f is not None else "N/A"), foreground="green")

            if apos_f is not None:
                tab.lbl_abs_pos.config(text=f"{apos_f:.1f}°", foreground="orange")
                tab._gauge_abs_deg = apos_f
            else:
                tab.lbl_abs_pos.config(text="N/A", foreground="orange")
                tab._gauge_abs_deg = None

            if display_deg is not None:
                tab._gauge_current_deg = display_deg

            if mid == "5":
                self.wrist_tab.m5_apos = apos_f
                self.wrist_tab.update_values()
            elif mid == "6":
                self.wrist_tab.m6_apos = apos_f
                self.wrist_tab.update_values()

        if hasattr(self, "wrist_tuning_tab"):
            self.wrist_tuning_tab.update_values(self.get_wrist_telemetry_snapshot())

        if hasattr(self, "speed_tuner_tab"):
            try:
                self.speed_tuner_tab.update_telemetry(int(mid), rpm_f, amp_f)
            except Exception:
                pass

    def get_wrist_telemetry_snapshot(self):
        cfg = self.config.get("wrist_differential", {})
        pitch_mid = str(self._safe_int(cfg.get("pitch_encoder_motor_id", 5), 5))
        roll_mid = str(self._safe_int(cfg.get("roll_encoder_motor_id", 6), 6))
        pitch_joint = str(self._safe_int(cfg.get("pitch_joint_id", 5), 5))
        roll_joint = str(self._safe_int(cfg.get("roll_joint_id", 6), 6))
        pitch_raw = self.latest_telemetry.get(pitch_mid, {}).get("apos")
        roll_raw = self.latest_telemetry.get(roll_mid, {}).get("apos")
        pitch_zero = self._safe_float(self.config.get("motors", {}).get(pitch_joint, {}).get("abs_zero_offset", 0.0))
        roll_zero = self._safe_float(self.config.get("motors", {}).get(roll_joint, {}).get("abs_zero_offset", 0.0))
        if pitch_zero is None:
            pitch_zero = 0.0
        if roll_zero is None:
            roll_zero = 0.0
        motor_a_logical = None if pitch_raw is None else pitch_raw - pitch_zero
        motor_b_logical = None if roll_raw is None else roll_raw - roll_zero
        mix_cfg = cfg.get("mix", {}) if isinstance(cfg.get("mix"), dict) else {}
        if motor_a_logical is None or motor_b_logical is None:
            pitch = None
            roll = None
        else:
            pitch, roll = invert_wrist_mix(motor_a_logical, motor_b_logical, mix_cfg)
        pitch_target = self.wrist_diff_target_logical_deg.get("pitch")
        roll_target = self.wrist_diff_target_logical_deg.get("roll")
        motor_a_mid = str(self._safe_int(cfg.get("motor_a_id", 5), 5))
        motor_b_mid = str(self._safe_int(cfg.get("motor_b_id", 6), 6))
        return {
            "pitch": pitch,
            "roll": roll,
            "pitch_target": pitch_target,
            "roll_target": roll_target,
            "motor_a_cmd": self.latest_telemetry.get(motor_a_mid, {}).get("rpm"),
            "motor_b_cmd": self.latest_telemetry.get(motor_b_mid, {}).get("rpm"),
        }

    def clear_log(self):
        self.log_box.config(state="normal")
        self.log_box.delete("1.0", "end")
        self.log_box.config(state="disabled")
        self._log_line_count = 0

    def log_append(self, line):
        MAX_LINES = 200
        if line.startswith("M") and "POS:" in line:
            tag = "telemetry"
        elif "Error" in line or "error" in line or "blocked" in line:
            tag = "error"
        else:
            tag = "info"
        self.log_box.config(state="normal")
        self.log_box.insert("end", line + "\n", tag)
        self._log_line_count += 1
        if self._log_line_count > MAX_LINES:
            self.log_box.delete("1.0", "2.0")
            self._log_line_count -= 1
        self.log_box.see("end")
        self.log_box.config(state="disabled")

    def _resolve_target_motors(self, motor_tok):
        tok = motor_tok.strip()
        if tok == "*":
            return list(self.tabs.keys())
        return [tok] if tok in self.tabs else []

    def _set_target_from_firmware(self, mid, logical_deg):
        tab = self.tabs.get(str(mid))
        if not tab:
            return
        tab._set_target_display(logical_deg, reference=tab._gauge_current_deg)

    def _sync_targets_to_current(self):
        for tab in self.tabs.values():
            if tab._gauge_current_deg is not None:
                tab._set_target_display(tab._gauge_current_deg)

    def _estimate_target_base(self, tab):
        if tab._gauge_current_deg is not None:
            return tab._gauge_current_deg
        if tab.target_deg is not None:
            return tab.target_deg
        return 0.0

    @staticmethod
    def _is_motion_command(cmd):
        motion_ops = {"S", "P", "PR", "M", "MV", "CAL", "JLIM"}
        for part in [p.strip() for p in cmd.split(";") if p.strip()]:
            op = part.split()[0].upper() if part.split() else ""
            if op in motion_ops:
                return True
        return False

    def _clear_pending_commands(self):
        self._clear_pending_commands_for()

    def _update_target_labels_from_cmd(self, cmd):
        parts = [p.strip() for p in cmd.split(";") if p.strip()]
        for part in parts:
            toks = part.split()
            if len(toks) < 1:
                continue
            op = toks[0].upper()

            if op == "P" and len(toks) >= 3:
                angle = self._safe_float(toks[2])
                if angle is None:
                    continue
                for mid in self._resolve_target_motors(toks[1]):
                    tab = self.tabs[mid]
                    tab._set_target_display(angle, reference=self._estimate_target_base(tab))
            elif op == "PR" and len(toks) >= 3:
                delta = self._safe_float(toks[2])
                if delta is None:
                    continue
                for mid in self._resolve_target_motors(toks[1]):
                    tab = self.tabs[mid]
                    base = self._estimate_target_base(tab)
                    tab._set_target_display(base + delta, reference=base)
            elif op == "CAL" and len(toks) >= 3:
                angle = self._safe_float(toks[2])
                if angle is None:
                    continue
                for mid in self._resolve_target_motors(toks[1]):
                    self.tabs[mid]._set_target_display(angle, reference=angle)
            elif op == "JLIM" and len(toks) >= 4:
                for mid in self._resolve_target_motors(toks[1]):
                    tab = self.tabs[mid]
                    if tab.target_deg is not None:
                        tab._set_target_display(tab.target_deg, reference=tab._gauge_current_deg)
            elif op == "M" and len(toks) >= 3:
                speed = self._safe_float(toks[2])
                if speed is None or abs(speed) >= 1.0:
                    continue
                for mid in self._resolve_target_motors(toks[1]):
                    tab = self.tabs[mid]
                    if tab._gauge_current_deg is not None:
                        tab._set_target_display(tab._gauge_current_deg)
            elif op == "S":
                for tab in self.tabs.values():
                    if tab._gauge_current_deg is not None:
                        tab._set_target_display(tab._gauge_current_deg)

    def command_tx_loop(self, controller_name):
        while self.running:
            try:
                cmd = self.controller_cmd_queues[controller_name].get(timeout=0.1)
            except queue.Empty:
                continue
            try:
                conn = self.controller_serial.get(controller_name)
                if conn and conn.is_open:
                    conn.write((cmd + '\n').encode('utf-8'))
                time.sleep(0.015)
            except Exception:
                pass
            finally:
                self.controller_cmd_queues[controller_name].task_done()

    def send_command(self, cmd):
        if self._connected_controller_names():
            try:
                if self._is_motion_command(cmd):
                    self._clear_pending_commands()
                self._update_target_labels_from_cmd(cmd)
                routed = self._route_command_packets(cmd)
                for name, packet in routed.items():
                    conn = self.controller_serial.get(name)
                    if conn and conn.is_open:
                        self.controller_cmd_queues[name].put(packet)
                self.status_bar.config(text=f"Queued: {cmd}")
                self.root.after(0, self._log_sent, cmd)
            except Exception as e:
                messagebox.showerror("Serial Error", f"Failed to queue command:\n{e}")
                self.toggle_serial()
        else:
            self.status_bar.config(text=f"Not connected! (Would have sent: {cmd})")

    def _log_sent(self, cmd):
        self.log_box.config(state="normal")
        self.log_box.insert("end", f">>> SENT: {cmd}\n", "sent")
        self._log_line_count += 1
        if self._log_line_count > 200:
            self.log_box.delete("1.0", "2.0")
            self._log_line_count -= 1
        self.log_box.see("end")
        self.log_box.config(state="disabled")

    def apply_all(self):
        if not self._is_any_controller_connected():
            messagebox.showwarning("Not Connected", "Please connect to the arm first to apply settings alive.")
            return

        # Sync all UI values back into config before building command packets.
        for tab in self.tabs.values():
            tab.on_change()
        if hasattr(self, "wrist_tuning_tab"):
            self.wrist_tuning_tab.on_change()

        # Safe bring-up first: stop anything stale and ensure telemetry is ON.
        self.send_command("S; TON")
        time.sleep(0.05)

        for mid, tab in self.tabs.items():
            # Batch all commands for this motor into one semicolon-separated string
            gr   = tab.config.get("gear_ratio", 1.0)
            mn   = tab.var_min.get(); mx = tab.var_max.get()
            if mx <= mn:
                mn, mx = 0.0, 360.0
                tab.var_min.set(mn)
                tab.var_max.set(mx)
                tab.on_change()
            kp   = tab.var_kp.get();  ki  = tab.var_ki.get();  kd  = tab.var_kd.get();  aol = tab.var_aol.get()
            skp  = tab.var_skp.get(); ski = tab.var_ski.get(); skd = tab.var_skd.get(); sol = tab.var_sol.get()
            batch = (
                f"GEAR {mid} {gr}"
                f"; JLIM {mid} {mn:.1f} {mx:.1f}"
                f"; KP {mid} {kp:.4f}; KI {mid} {ki:.4f}; KD {mid} {kd:.4f}; AOL {mid} {aol:.2f}"
                f"; SKP {mid} {skp:.4f}; SKI {mid} {ski:.4f}; SKD {mid} {skd:.4f}; SOL {mid} {sol:.2f}"
            )
            self.send_command(batch)
            time.sleep(0.05)

        # Global encoder remap pass from motor tabs.
        self.apply_encoder_mappings(show_status=False)

        # Wrist-specific remap (only applies when wrist differential is enabled
        # and wrist controller is actually connected).
        self.apply_wrist_settings(show_status=False)

        for mid, tab in self.tabs.items():
            enc_idx = tab.config.get("abs_encoder_index")
            if enc_idx is None:
                continue
            try:
                enc_i = int(enc_idx)
                rev_i = 1 if bool(tab.config.get("abs_encoder_reversed", True)) else 0
                self.send_command(f"ENCREVSET {enc_i} {rev_i} {int(mid)}")
                time.sleep(0.02)
            except Exception:
                continue

        # Firmware detects mapped abs encoders asynchronously in its background
        # update loop. Allow a brief settle window before ABSPID enable.
        time.sleep(0.20)

        skipped_abs_pid = []
        abspid_packets = []
        for mid, tab in self.tabs.items():
            enable = 1 if bool(tab.config.get("abs_pid_enabled", False)) else 0
            if tab.config.get("abs_encoder_index") is None:
                enable = 0
                if bool(tab.config.get("abs_pid_enabled", False)):
                    skipped_abs_pid.append(int(mid))
            if self.config.get("wrist_differential", {}).get("enabled", False) and mid in {
                str(int(self.config["wrist_differential"].get("pitch_joint_id", 5))),
                str(int(self.config["wrist_differential"].get("roll_joint_id", 6)))
            }:
                enable = 0
            abspid_packets.append((int(mid), int(enable)))
            self.send_command(f"ABSPID {mid} {enable}")
            time.sleep(0.02)

        # Retry ABSPID enables to survive transient "encoder not detected yet"
        # windows right after ENCMAP updates.
        for _ in range(3):
            time.sleep(0.10)
            for mid, enable in abspid_packets:
                if enable == 1:
                    self.send_command(f"ABSPID {mid} 1")
                    time.sleep(0.01)

        if skipped_abs_pid:
            self.status_bar.config(
                text=(
                    "Applied ALL settings. ABS PID auto-disabled for motors without encoder mapping: "
                    + ", ".join(f"M{m}" for m in sorted(skipped_abs_pid))
                )
            )
        else:
            self.status_bar.config(text="Applied ALL settings: gear, limits, PID, ENCMAP, ABSPID, telemetry ON.")

    def apply_encoder_mappings(self, show_status=True):
        if not self._is_any_controller_connected():
            if show_status:
                messagebox.showwarning("Not Connected", "Please connect to the arm first.")
            return

        desired_by_controller = {"base": {}, "joint4": {}, "wrist": {}}
        duplicates = []
        for mid, tab in self.tabs.items():
            tab.on_change()
            enc_idx = tab.config.get("abs_encoder_index")
            if enc_idx is None:
                continue
            try:
                mid_i = int(mid)
                enc_i = int(enc_idx)
            except Exception:
                continue
            controller_name = self._controller_for_motor(mid_i)
            per_ctrl = desired_by_controller.setdefault(controller_name, {})
            if enc_i in per_ctrl and per_ctrl[enc_i] != mid_i:
                duplicates.append((controller_name, enc_i))
            per_ctrl[enc_i] = mid_i

        if duplicates and show_status:
            dup_text = ", ".join(f"{ctrl}:E{enc}" for ctrl, enc in sorted(set(duplicates)))
            messagebox.showwarning("Encoder Mapping", f"Conflicting encoder index on the same controller ({dup_text}). Last one wins per controller.")

        # Always clear all firmware encoder slots so stale mappings from prior
        # sessions/power cycles cannot survive.
        for e in range(8):
            self.send_command(f"ENCMAP {e} 0")
            time.sleep(0.01)

        for controller_name in ("base", "joint4", "wrist"):
            per_ctrl = desired_by_controller.get(controller_name, {})
            for enc_i in sorted(per_ctrl.keys()):
                self.send_command(f"ENCMAP {enc_i} {int(per_ctrl[enc_i])}")
                time.sleep(0.02)

        if show_status:
            self.status_bar.config(text="Applied absolute encoder mappings per controller.")

    def apply_wrist_settings(self, show_status=True):
        if hasattr(self, "wrist_tuning_tab"):
            self.wrist_tuning_tab.on_change()
        if not self._is_any_controller_connected():
            if show_status:
                messagebox.showwarning("Not Connected", "Please connect to the arm first.")
            return
        cfg = self.config.get("wrist_differential", {})
        if not bool(cfg.get("enabled", False)):
            if show_status:
                self.status_bar.config(text="Wrist differential disabled; wrist encoder remap skipped.")
            return
        wrist_conn = self.controller_serial.get("wrist")
        if not (wrist_conn and wrist_conn.is_open):
            if show_status:
                self.status_bar.config(text="Wrist controller not connected; skipped wrist encoder remap.")
            return
        pitch_enc = cfg.get("pitch_encoder_index")
        roll_enc = cfg.get("roll_encoder_index")
        pitch_mid = int(cfg.get("pitch_encoder_motor_id", 5))
        roll_mid = int(cfg.get("roll_encoder_motor_id", 6))
        if pitch_enc is not None:
            self.send_command(f"ENCMAP {int(pitch_enc)} 0")
            time.sleep(0.01)
            self.send_command(f"ENCMAP {int(pitch_enc)} {pitch_mid}")
            time.sleep(0.02)
        if roll_enc is not None:
            self.send_command(f"ENCMAP {int(roll_enc)} 0")
            time.sleep(0.01)
            self.send_command(f"ENCMAP {int(roll_enc)} {roll_mid}")
            time.sleep(0.02)
        for joint_mid in (int(cfg.get("pitch_joint_id", 5)), int(cfg.get("roll_joint_id", 6))):
            self.send_command(f"ABSPID {joint_mid} 0")
            time.sleep(0.02)
        if show_status:
            self.status_bar.config(text="Applied wrist differential config and encoder mappings.")

    def load_preset_all(self, preset_name):
        for tab in self.tabs.values():
            tab.var_preset.set(preset_name)
            tab.load_selected_preset()
        self.status_bar.config(text=f"Loaded '{preset_name}' preset into all motor tabs. Review, then Apply.")

    def apply_limits_all(self):
        if not self._is_any_controller_connected():
            messagebox.showwarning("Not Connected", "Please connect to the arm first.")
            return
        for mid, tab in self.tabs.items():
            mn = tab.var_min.get()
            mx = tab.var_max.get()
            if mx <= mn:
                continue
            self.send_command(f"JLIM {mid} {mn:.1f} {mx:.1f}")
            time.sleep(0.03)
        self.status_bar.config(text="Applied all joint limits to arm.")

    def safe_bringup(self):
        if not self._is_any_controller_connected():
            messagebox.showwarning("Not Connected", "Please connect to the arm first.")
            return
        self.send_command("S; TON")
        self.status_bar.config(text="Safe bring-up sent: STOP + telemetry ON.")

    def on_close(self):
        self.running = False
        if self.ser and self.ser.is_open:
            self.ser.close()
        self.root.destroy()

if __name__ == "__main__":
    if not os.path.exists(CONFIG_FILE):
        with open(CONFIG_FILE, 'w') as f:
            json.dump(DEFAULT_CONFIG, f, indent=4)

    root = tk.Tk()
    app = PIDTunerApp(root)
    root.protocol("WM_DELETE_WINDOW", app.on_close)

    root.lift()
    root.attributes('-topmost',True)
    root.after_idle(root.attributes,'-topmost',False)

    root.mainloop()
