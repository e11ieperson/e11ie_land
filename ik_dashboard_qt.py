import os
import sys
import time
import tempfile
import traceback
import json

import numpy as np
import matplotlib.pyplot as plt
from PySide6.QtCore import QTimer, Qt, QEvent
from PySide6.QtWidgets import (
    QApplication,
    QCheckBox,
    QComboBox,
    QDoubleSpinBox,
    QGridLayout,
    QGroupBox,
    QLabel,
    QMainWindow,
    QPushButton,
    QSplitter,
    QHeaderView,
    QScrollArea,
    QTableWidget,
    QTableWidgetItem,
    QTextEdit,
    QTabWidget,
    QVBoxLayout,
    QWidget,
    QSpinBox,
)
from matplotlib.backends.backend_qtagg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.figure import Figure
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401 unused import for 3D projection

import ik_arm_control as backend


WRIST_VIZ_URDF_FILE = os.path.join(os.path.dirname(__file__), "URDF", "wrist_visualization.urdf")


class NoScrollDoubleSpinBox(QDoubleSpinBox):
    """DoubleSpinBox that ignores mouse wheel events to prevent scroll interference."""
    def wheelEvent(self, event):
        event.ignore()


class NoScrollSpinBox(QSpinBox):
    """SpinBox that ignores mouse wheel events to prevent scroll interference."""
    def wheelEvent(self, event):
        event.ignore()


class WristVisualizationWidget(QWidget):
    """Dedicated 3D wrist visualization showing pitch/roll orientation and motor commands."""

    def __init__(self, parent=None):
        super().__init__(parent)
        self._build_ui()
        self._pitch_actual = 0.0
        self._roll_actual = 0.0
        self._pitch_target = None
        self._roll_target = None
        self._motor_a_cmd = 0.0
        self._motor_b_cmd = 0.0
        self._last_draw_payload = None
        self._last_draw_time = 0.0

    def _build_ui(self):
        layout = QVBoxLayout(self)
        layout.setContentsMargins(4, 4, 4, 4)
        layout.setSpacing(4)

        # Title
        title = QLabel("Wrist Differential Visualization")
        title.setStyleSheet("font-weight: bold; font-size: 11px;")
        layout.addWidget(title)

        # Matplotlib figure for 3D wrist view
        self.fig = Figure(figsize=(3.5, 3.0), dpi=100)
        self.fig.patch.set_facecolor('#f8f9fa')
        self.canvas = FigureCanvas(self.fig)
        layout.addWidget(self.canvas)

        # Create 3D axis for wrist orientation
        self.ax = self.fig.add_subplot(111, projection='3d')
        self.ax.set_xlim(-1.2, 1.2)
        self.ax.set_ylim(-1.2, 1.2)
        self.ax.set_zlim(-0.5, 0.5)
        self.ax.set_xlabel('X', fontsize=8)
        self.ax.set_ylabel('Y', fontsize=8)
        self.ax.set_zlabel('Z', fontsize=8)
        self.ax.set_title('Wrist Pose', fontsize=9)
        self.ax.view_init(elev=20, azim=-60)
        self.ax.set_facecolor('#f0f0f0')

        # Status labels below the plot
        status_layout = QGridLayout()
        status_layout.setSpacing(2)

        self.lbl_pitch = QLabel("Pitch: ---.-°")
        self.lbl_pitch.setStyleSheet("font-family: monospace; color: #6a0dad;")
        status_layout.addWidget(QLabel("Actual:"), 0, 0)
        status_layout.addWidget(self.lbl_pitch, 0, 1)

        self.lbl_roll = QLabel("Roll: ---.-°")
        self.lbl_roll.setStyleSheet("font-family: monospace; color: #008080;")
        status_layout.addWidget(self.lbl_roll, 0, 2, 1, 2)

        self.lbl_pitch_target = QLabel("Target: ---.-°")
        self.lbl_pitch_target.setStyleSheet("font-family: monospace; color: #ff6600;")
        status_layout.addWidget(QLabel("Target:"), 1, 0)
        status_layout.addWidget(self.lbl_pitch_target, 1, 1)

        self.lbl_roll_target = QLabel("Target: ---.-°")
        self.lbl_roll_target.setStyleSheet("font-family: monospace; color: #cc6600;")
        status_layout.addWidget(self.lbl_roll_target, 1, 2, 1, 2)

        layout.addLayout(status_layout)

        # Motor command bars
        motor_group = QGroupBox("Motor Commands (RPM)")
        motor_layout = QGridLayout(motor_group)
        motor_layout.setSpacing(2)

        self.lbl_motor_a = QLabel("Motor A: ---.-")
        self.lbl_motor_a.setStyleSheet("font-family: monospace;")
        self.lbl_motor_b = QLabel("Motor B: ---.-")
        self.lbl_motor_b.setStyleSheet("font-family: monospace;")

        motor_layout.addWidget(QLabel("A:"), 0, 0)
        motor_layout.addWidget(self.lbl_motor_a, 0, 1)
        motor_layout.addWidget(QLabel("B:"), 1, 0)
        motor_layout.addWidget(self.lbl_motor_b, 1, 1)

        layout.addWidget(motor_group)

        # Initialize plot elements
        self._init_plot()

    def _init_plot(self):
        """Initialize the 3D wrist visualization elements."""
        self.ax.cla()
        self.ax.set_xlim(-1.2, 1.2)
        self.ax.set_ylim(-1.2, 1.2)
        self.ax.set_zlim(-0.5, 0.5)
        self.ax.set_xlabel('X', fontsize=8)
        self.ax.set_ylabel('Y', fontsize=8)
        self.ax.set_zlabel('Z', fontsize=8)
        self.ax.set_title('Wrist Pose', fontsize=9)
        self.ax.set_facecolor('#f0f0f0')

        # Draw coordinate frame at origin
        self.ax.quiver(0, 0, 0, 0.8, 0, 0, color='red', arrow_length_ratio=0.15, linewidth=1.5)
        self.ax.quiver(0, 0, 0, 0, 0.8, 0, color='green', arrow_length_ratio=0.15, linewidth=1.5)
        self.ax.quiver(0, 0, 0, 0, 0, 0.4, color='blue', arrow_length_ratio=0.15, linewidth=1.5)

        # Draw wrist plate (circle representing the wrist plane)
        theta = np.linspace(0, 2 * np.pi, 50)
        radius = 0.6
        x_circle = radius * np.cos(theta)
        y_circle = radius * np.sin(theta)
        z_circle = np.zeros_like(theta)
        self.wrist_plate, = self.ax.plot(x_circle, y_circle, z_circle, 'b-', linewidth=2, alpha=0.6)

        # Pitch indicator (vertical bar)
        self.pitch_line, = self.ax.plot([0, 0], [0, 0], [-0.3, 0.3], 'purple', linewidth=3, label='Pitch')

        # Roll indicator (horizontal bar)
        self.roll_line, = self.ax.plot([0, 0], [0, 0], [0, 0], 'teal', linewidth=3, label='Roll')

        # Target crosshairs
        self.target_pitch, = self.ax.plot([0, 0], [0, 0], [-0.25, 0.25], 'orange', linewidth=1.5, linestyle='--', alpha=0.7)
        self.target_roll, = self.ax.plot([0, 0], [0, 0], [0, 0], 'darkorange', linewidth=1.5, linestyle='--', alpha=0.7)

        self.ax.legend(loc='upper left', fontsize=7)

    def update_wrist(self, pitch_actual, roll_actual, pitch_target, roll_target, motor_a_cmd, motor_b_cmd):
        """Update the wrist visualization with new data."""
        self._pitch_actual = pitch_actual if pitch_actual is not None else 0.0
        self._roll_actual = roll_actual if roll_actual is not None else 0.0
        self._pitch_target = pitch_target
        self._roll_target = roll_target
        self._motor_a_cmd = motor_a_cmd if motor_a_cmd is not None else 0.0
        self._motor_b_cmd = motor_b_cmd if motor_b_cmd is not None else 0.0
        payload = (
            round(float(self._pitch_actual), 1),
            round(float(self._roll_actual), 1),
            None if self._pitch_target is None else round(float(self._pitch_target), 1),
            None if self._roll_target is None else round(float(self._roll_target), 1),
            round(float(self._motor_a_cmd), 1),
            round(float(self._motor_b_cmd), 1),
        )
        now = time.time()
        should_redraw = (payload != self._last_draw_payload) or ((now - self._last_draw_time) >= 0.25)
        self._update_labels()
        if should_redraw:
            self._redraw()
            self._last_draw_payload = payload
            self._last_draw_time = now

    def _redraw(self):
        """Redraw the 3D visualization."""
        self.ax.cla()
        self.ax.set_xlim(-1.2, 1.2)
        self.ax.set_ylim(-1.2, 1.2)
        self.ax.set_zlim(-0.5, 0.5)
        self.ax.set_xlabel('X', fontsize=8)
        self.ax.set_ylabel('Y', fontsize=8)
        self.ax.set_zlabel('Z', fontsize=8)
        self.ax.set_title('Wrist Pose', fontsize=9)
        self.ax.set_facecolor('#f0f0f0')

        # Base coordinate frame
        self.ax.quiver(0, 0, 0, 0.8, 0, 0, color='red', arrow_length_ratio=0.15, linewidth=1.5)
        self.ax.quiver(0, 0, 0, 0, 0.8, 0, color='green', arrow_length_ratio=0.15, linewidth=1.5)
        self.ax.quiver(0, 0, 0, 0, 0, 0.4, color='blue', arrow_length_ratio=0.15, linewidth=1.5)

        # Wrist plate (tilted by pitch and roll)
        pitch_rad = np.radians(self._pitch_actual)
        roll_rad = np.radians(self._roll_actual)

        theta = np.linspace(0, 2 * np.pi, 50)
        radius = 0.6
        x_base = radius * np.cos(theta)
        y_base = radius * np.sin(theta)

        # Apply rotation: pitch rotates around Y, roll rotates around X
        x_tilted = x_base * np.cos(roll_rad) + 0 * np.sin(roll_rad)
        y_tilted = y_base * np.cos(pitch_rad)
        z_tilted = x_base * np.sin(roll_rad) + y_base * np.sin(pitch_rad) * 0.5

        self.ax.plot(x_tilted, y_tilted, z_tilted, 'b-', linewidth=2, alpha=0.6)

        # Pitch indicator (vertical line through center, tilted)
        pitch_z = 0.35 * np.sin(pitch_rad)
        self.ax.plot([0, 0], [0, 0], [-0.3 + pitch_z, 0.3 + pitch_z], 'purple', linewidth=3, label='Pitch')

        # Roll indicator (horizontal line through center, tilted)
        roll_x = 0.35 * np.sin(roll_rad)
        self.ax.plot([-0.3 + roll_x, 0.3 + roll_x], [0, 0], [0, 0], 'teal', linewidth=3, label='Roll')

        # Target crosshairs (if set)
        if self._pitch_target is not None:
            tgt_pitch_rad = np.radians(self._pitch_target)
            tgt_pitch_z = 0.25 * np.sin(tgt_pitch_rad)
            self.ax.plot([0, 0], [0, 0], [-0.25 + tgt_pitch_z, 0.25 + tgt_pitch_z],
                         'orange', linewidth=1.5, linestyle='--', alpha=0.7, label='Tgt Pitch')

        if self._roll_target is not None:
            tgt_roll_rad = np.radians(self._roll_target)
            tgt_roll_x = 0.25 * np.sin(tgt_roll_rad)
            self.ax.plot([-0.25 + tgt_roll_x, 0.25 + tgt_roll_x], [0, 0], [0, 0],
                         'darkorange', linewidth=1.5, linestyle='--', alpha=0.7, label='Tgt Roll')

        # Motor command indicators (small arrows on the side)
        motor_scale = 0.005  # Scale RPM to visual length
        if abs(self._motor_a_cmd) > 0.5:
            direction = 1 if self._motor_a_cmd > 0 else -1
            length = min(abs(self._motor_a_cmd) * motor_scale, 0.4)
            self.ax.quiver(-0.8, 0.5, 0, 0, direction * length, 0, color='darkred',
                          arrow_length_ratio=0.2, linewidth=2)
            self.ax.text(-0.8, 0.5 + direction * 0.15, 0, f'A:{int(self._motor_a_cmd)}', fontsize=7, color='darkred')

        if abs(self._motor_b_cmd) > 0.5:
            direction = 1 if self._motor_b_cmd > 0 else -1
            length = min(abs(self._motor_b_cmd) * motor_scale, 0.4)
            self.ax.quiver(-0.8, -0.5, 0, 0, direction * length, 0, color='darkblue',
                          arrow_length_ratio=0.2, linewidth=2)
            self.ax.text(-0.8, -0.5 + direction * 0.15, 0, f'B:{int(self._motor_b_cmd)}', fontsize=7, color='darkblue')

        self.ax.legend(loc='upper left', fontsize=7)
        self.canvas.draw_idle()

    def _update_labels(self):
        """Update the status labels."""
        self.lbl_pitch.setText(f"Pitch: {_fmt_deg(self._pitch_actual)}")
        self.lbl_roll.setText(f"Roll: {_fmt_deg(self._roll_actual)}")
        self.lbl_pitch_target.setText(f"Target: {_fmt_deg(self._pitch_target)}")
        self.lbl_roll_target.setText(f"Target: {_fmt_deg(self._roll_target)}")
        self.lbl_motor_a.setText(f"Motor A: {_fmt_rpm(self._motor_a_cmd)} RPM")
        self.lbl_motor_b.setText(f"Motor B: {_fmt_rpm(self._motor_b_cmd)} RPM")


KEYMAP_FILE = os.path.join(os.path.dirname(__file__), "ik_dashboard_keys.json")
KEY_OPTIONS = [
    "None",
    "W", "A", "S", "D", "Q", "E",
    "I", "J", "K", "L", "U", "O",
    "T", "F", "G", "H", "R", "Y",
    "P", "X", "C", "V", "B", "N", "M",
    "Up", "Down", "Left", "Right", "Space",
]
DEFAULT_KEYMAP = {
    "move_x_neg": "A",
    "move_x_pos": "D",
    "move_y_pos": "W",
    "move_y_neg": "S",
    "move_z_neg": "Q",
    "move_z_pos": "E",
    "pitch_pos": "I",
    "pitch_neg": "K",
    "roll_neg": "J",
    "roll_pos": "L",
    "yaw_neg": "U",
    "yaw_pos": "O",
    "cam_left": "Left",
    "cam_right": "Right",
    "cam_up": "Up",
    "cam_down": "Down",
    "execute": "Space",
    "estop": "X",
    "preview": "P",
    "sync_target": "C",
    "hand_open": "T",
    "hand_close": "G",
}
ACTION_LABELS = {
    "move_x_neg": "Move X-",
    "move_x_pos": "Move X+",
    "move_y_pos": "Move Y+",
    "move_y_neg": "Move Y-",
    "move_z_neg": "Move Z-",
    "move_z_pos": "Move Z+",
    "pitch_pos": "Pitch +",
    "pitch_neg": "Pitch -",
    "roll_neg": "Roll -",
    "roll_pos": "Roll +",
    "yaw_neg": "Yaw -",
    "yaw_pos": "Yaw +",
    "cam_left": "Cam Left",
    "cam_right": "Cam Right",
    "cam_up": "Cam Up",
    "cam_down": "Cam Down",
    "execute": "Execute",
    "estop": "E-Stop",
    "preview": "Preview",
    "sync_target": "Target Sync",
    "hand_open": "Hand Open",
    "hand_close": "Hand Close",
}


def _fmt_deg(value):
    return "---.-°" if value is None else f"{float(value):7.1f}°"


def _fmt_rpm(value):
    return "---.-" if value is None else f"{float(value):7.1f}"


def get_hand_dashboard_snapshot():
    return {
        "pwm": int(getattr(backend, "hand_pwm_command", 0)),
        "controller": "wrist",
    }


def get_wrist_dashboard_snapshot():
    if not backend.wrist_diff_enabled():
        return None
    cfg = backend.get_wrist_diff_config()
    pitch_joint_id, roll_joint_id = backend.wrist_diff_joint_ids()
    motor_a_id, motor_b_id = backend.wrist_diff_motor_ids()
    pitch_actual = backend.get_wrist_output_logical_deg("pitch")
    roll_actual = backend.get_wrist_output_logical_deg("roll")
    pitch_target = backend.wrist_diff_target_logical_deg.get("pitch")
    roll_target = backend.wrist_diff_target_logical_deg.get("roll")
    last_cmd_a, last_cmd_b = backend.wrist_diff_last_motor_cmd
    return {
        "pitch_joint_id": pitch_joint_id,
        "roll_joint_id": roll_joint_id,
        "pitch_actual": pitch_actual,
        "roll_actual": roll_actual,
        "pitch_target": pitch_target,
        "roll_target": roll_target,
        "motor_a_id": motor_a_id,
        "motor_b_id": motor_b_id,
        "motor_a_cmd": last_cmd_a,
        "motor_b_cmd": last_cmd_b,
        "pitch_encoder_motor_id": int(cfg.get("pitch_encoder_motor_id", 5)),
        "roll_encoder_motor_id": int(cfg.get("roll_encoder_motor_id", 6)),
    }


class ArmCanvas(FigureCanvas):
    def __init__(self, parent=None):
        self.figure = Figure(figsize=(9.4, 9.2))
        super().__init__(self.figure)
        self.setParent(parent)
        # Layout: top row = 3D + Side view, bottom row = status + wrist differential
        gs = self.figure.add_gridspec(2, 2, height_ratios=[2.7, 1.25], width_ratios=[1.45, 1.0], hspace=0.10, wspace=0.08)
        self.ax = self.figure.add_subplot(gs[0, 0], projection="3d")
        self.ax_side = self.figure.add_subplot(gs[0, 1], projection="3d")
        self.ax_wrist = self.figure.add_subplot(gs[1, 1], projection="3d")
        self.ax_top = None
        self.ax_status = self.figure.add_subplot(gs[1, 0])
        self.figure.subplots_adjust(left=0.03, right=0.99, top=0.98, bottom=0.04)
        self._wrist_anim_frame = 0
        self.wrist_chain = None
        if os.path.exists(WRIST_VIZ_URDF_FILE):
            try:
                self.wrist_chain = backend.Chain.from_urdf_file(WRIST_VIZ_URDF_FILE)
            except Exception:
                self.wrist_chain = None

    def draw_arm(self):
        self.ax.cla()
        self.ax_side.cla()
        self.ax_status.cla()
        self.ax_wrist.cla()
        for axis in (self.ax, self.ax_side):
            axis.set_xlim(-0.9, 0.9)
            axis.set_ylim(-0.9, 0.9)
            axis.set_zlim(-0.05, 1.0)
            axis.set_xlabel("X")
            axis.set_ylabel("Y")
            axis.set_zlabel("Z")
        self.ax.set_title("3D View")
        self.ax_side.set_title("Side View")
        self.ax.view_init(elev=backend.cam_elev, azim=backend.cam_azim)
        self.ax_side.view_init(elev=0, azim=-90)

        if backend.my_chain is None:
            self.draw_idle()
            return

        if backend.actual_joint_angles is None:
            backend.actual_joint_angles = list(backend.current_joint_angles)

        if backend.sync_model_to_telemetry:
            tele_angles = backend.get_current_real_joint_angles()
            if tele_angles is not None:
                backend.actual_joint_angles = list(tele_angles)

        ghost_pos = backend.my_chain.forward_kinematics(backend.actual_joint_angles, full_kinematics=True)
        gx, gy, gz = backend.extract_xyz(ghost_pos)
        arm_pos = backend.my_chain.forward_kinematics(backend.current_joint_angles, full_kinematics=True)
        x, y, z = backend.extract_xyz(arm_pos)
        target_x = -backend.current_target_xyz[0] if backend.DISPLAY_MIRROR_X else backend.current_target_xyz[0]

        for axis in (self.ax, self.ax_side):
            axis.plot(gx, gy, gz, "-o", color="darkorange", alpha=0.35, linewidth=4, markersize=8)
            axis.plot(x, y, z, "-o", color="tab:blue", linewidth=5, markersize=9)
            axis.plot([target_x], [backend.current_target_xyz[1]], [backend.current_target_xyz[2]], "x", color="red", markersize=12)

        for joint_idx in range(1, min(7, len(x))):
            self.ax.text(x[joint_idx], y[joint_idx], z[joint_idx] + 0.03, f"J{joint_idx}", color="darkred", fontsize=9)

        self.ax_status.axis("off")
        lines = []
        for joint_idx in range(1, 7):
            target_deg = float(np.degrees(backend.current_joint_angles[joint_idx])) if joint_idx < len(backend.current_joint_angles) else 0.0
            current_deg = backend.get_current_ik_deg_from_apos(joint_idx)
            if current_deg is None:
                lines.append(f"J{joint_idx}: tgt {target_deg:7.1f}°   cur   ---.-°")
            else:
                lines.append(f"J{joint_idx}: tgt {target_deg:7.1f}°   cur {current_deg:7.1f}°")
        wrist = get_wrist_dashboard_snapshot()
        if wrist is not None:
            lines.append("")
            lines.append(
                f"Wrist Pitch(J{wrist['pitch_joint_id']}): tgt {_fmt_deg(wrist['pitch_target']).strip()}   cur {_fmt_deg(wrist['pitch_actual']).strip()}"
            )
            lines.append(
                f"Wrist Roll (J{wrist['roll_joint_id']}): tgt {_fmt_deg(wrist['roll_target']).strip()}   cur {_fmt_deg(wrist['roll_actual']).strip()}"
            )
            lines.append(
                f"Mix Cmd M{wrist['motor_a_id']}/{wrist['motor_b_id']}: {_fmt_rpm(wrist['motor_a_cmd']).strip()} / {_fmt_rpm(wrist['motor_b_cmd']).strip()} RPM"
            )
        hand = get_hand_dashboard_snapshot()
        lines.append("")
        lines.append(f"Hand PWM ({hand['controller']}): {hand['pwm']}")
        self.ax_status.text(
            0.02,
            0.98,
            "Arm Status\n\n" + "\n".join(lines),
            va="top",
            ha="left",
            fontsize=10,
            family="monospace",
            bbox=dict(facecolor="#f4f4f4", edgecolor="black", boxstyle="round,pad=0.5"),
        )

        # Draw wrist differential visualization
        self._draw_wrist_differential(wrist)

        self.draw_idle()

    def _draw_wrist_differential(self, wrist):
        """Draw the wrist using a wrist-only URDF chain driven by pitch and roll."""
        ax = self.ax_wrist
        ax.set_xlim(-0.22, 0.22)
        ax.set_ylim(-0.22, 0.22)
        ax.set_zlim(-0.02, 0.28)
        ax.set_xlabel("X", fontsize=7)
        ax.set_ylabel("Y", fontsize=7)
        ax.set_zlabel("Z", fontsize=7)
        ax.set_title("Wrist Differential", fontsize=10, fontweight="bold")
        ax.view_init(elev=22, azim=-50)

        if wrist is None or not backend.wrist_diff_enabled() or self.wrist_chain is None:
            ax.text(
                0.0,
                0.0,
                0.12,
                "Wrist\nDisabled",
                ha="center",
                va="center",
                fontsize=12,
                color="#7a7a7a",
                fontweight="bold",
            )
            return

        pitch_actual = wrist.get("pitch_actual") if wrist.get("pitch_actual") is not None else 0.0
        roll_actual = wrist.get("roll_actual") if wrist.get("roll_actual") is not None else 0.0
        pitch_target = wrist.get("pitch_target")
        roll_target = wrist.get("roll_target")
        motor_a_cmd = wrist.get("motor_a_cmd") if wrist.get("motor_a_cmd") is not None else 0.0
        motor_b_cmd = wrist.get("motor_b_cmd") if wrist.get("motor_b_cmd") is not None else 0.0

        actual_angles = [0.0] * len(self.wrist_chain.links)
        actual_angles[1] = np.radians(float(pitch_actual))
        actual_angles[2] = np.radians(float(roll_actual))
        target_angles = [0.0] * len(self.wrist_chain.links)
        target_angles[1] = np.radians(float(pitch_actual if pitch_target is None else pitch_target))
        target_angles[2] = np.radians(float(roll_actual if roll_target is None else roll_target))

        actual_fk = self.wrist_chain.forward_kinematics(actual_angles, full_kinematics=True)
        target_fk = self.wrist_chain.forward_kinematics(target_angles, full_kinematics=True)
        actual_x = [p[0, 3] for p in actual_fk]
        actual_y = [p[1, 3] for p in actual_fk]
        actual_z = [p[2, 3] for p in actual_fk]
        target_x = [p[0, 3] for p in target_fk]
        target_y = [p[1, 3] for p in target_fk]
        target_z = [p[2, 3] for p in target_fk]

        ax.plot(target_x, target_y, target_z, "-o", color="darkorange", alpha=0.35, linewidth=4, markersize=7)
        ax.plot(actual_x, actual_y, actual_z, "-o", color="tab:blue", linewidth=4, markersize=8)

        tool = target_fk[-1]
        origin = tool[:3, 3]
        rot = tool[:3, :3]
        axis_len = 0.05
        ax.quiver(origin[0], origin[1], origin[2], rot[0, 0] * axis_len, rot[1, 0] * axis_len, rot[2, 0] * axis_len, color="red", arrow_length_ratio=0.25, linewidth=1.3)
        ax.quiver(origin[0], origin[1], origin[2], rot[0, 1] * axis_len, rot[1, 1] * axis_len, rot[2, 1] * axis_len, color="green", arrow_length_ratio=0.25, linewidth=1.3)
        ax.quiver(origin[0], origin[1], origin[2], rot[0, 2] * axis_len, rot[1, 2] * axis_len, rot[2, 2] * axis_len, color="blue", arrow_length_ratio=0.25, linewidth=1.3)

        ax.text2D(
            0.03,
            0.95,
            f"Pitch  cur {_fmt_deg(pitch_actual).strip()}   tgt {_fmt_deg(pitch_target).strip()}\n"
            f"Roll   cur {_fmt_deg(roll_actual).strip()}   tgt {_fmt_deg(roll_target).strip()}\n"
            f"Mix M{wrist['motor_a_id']}/{wrist['motor_b_id']}: {_fmt_rpm(motor_a_cmd).strip()} / {_fmt_rpm(motor_b_cmd).strip()} RPM",
            transform=ax.transAxes,
            va="top",
            ha="left",
            fontsize=8,
            family="monospace",
            bbox=dict(facecolor="#f8fafc", edgecolor="#cbd5e1", boxstyle="round,pad=0.35"),
        )


class DashboardWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("IK Dashboard (PySide6)")
        self.resize(1280, 900)
        self._log_lines = []
        self._building_controls = False
        self._last_status_snapshot = ""
        self._pressed_keys = set()
        self._key_action_fired = set()
        self._hand_key_active = None
        self._editing_inputs = set()  # Track inputs being edited
        self._last_connection_state = None
        self._last_status_text = ""
        self._last_pressed_text = ""
        self._last_indicator_snapshot = None
        self._last_telemetry_text = ""
        self._last_table_values = {}
        self._last_telemetry_live = False
        self._last_controller_state_snapshot = None
        self._last_auto_apply_time = 0.0
        self._last_runtime_apply_ok = False
        self._motion_armed = False
        self._motion_block_reason = "awaiting first verify"
        self._motion_block_logged = False
        self.keymap = self._load_keymap()
        self.keymap_inputs = {}
        self.key_indicator_labels = {}
        self._build_ui()
        self._init_backend()
        self._start_timers()
        self.refresh_all()
        self.setFocusPolicy(Qt.StrongFocus)

    def _backend_connected(self):
        return bool(getattr(backend, "is_serial_connected", lambda: False)())

    def _controller_runtime_state_snapshot(self):
        now = time.time()
        stale_sec = float(getattr(backend, "TELEMETRY_STALE_SEC", 1.0))

        groups = getattr(backend, "CONTROLLER_MOTOR_GROUPS", {}) or {}
        cfg = getattr(backend, "controller_config", {})
        if isinstance(cfg, dict) and cfg:
            mapped_groups = {}
            for name, default_motors in groups.items():
                cfg_entry = cfg.get(name, {}) if isinstance(cfg.get(name, {}), dict) else {}
                raw_motors = cfg_entry.get("motors", list(default_motors))
                parsed = set()
                if isinstance(raw_motors, list):
                    for mid in raw_motors:
                        try:
                            parsed.add(int(mid))
                        except Exception:
                            pass
                mapped_groups[name] = parsed if parsed else set(default_motors)
            groups = mapped_groups

        serial_map = getattr(backend, "controller_serial", {}) or {}
        tele_map = getattr(backend, "latest_motor_telemetry_time", {}) or {}

        states = []
        for name in sorted(groups.keys()):
            conn = serial_map.get(name)
            connected = bool(conn and getattr(conn, "is_open", False))
            fresh = False
            for mid in groups.get(name, set()):
                ts = tele_map.get(int(mid))
                if ts is None:
                    continue
                try:
                    if (now - float(ts)) <= stale_sec:
                        fresh = True
                        break
                except Exception:
                    continue
            states.append((name, connected, fresh))
        return tuple(states)

    def _init_backend(self):
        try:
            backend.load_robot_config()
            with open(backend.URDF_FILE, "r", encoding="utf-8") as f:
                urdf_data = f.read().replace('type="continuous"', 'type="revolute"')
            with tempfile.NamedTemporaryFile(delete=False, mode="w", suffix=".urdf", encoding="utf-8") as temp_urdf:
                temp_urdf.write(urdf_data)
                temp_path = temp_urdf.name
            backend.my_chain = backend.Chain.from_urdf_file(temp_path)
            os.remove(temp_path)
            joint_count = len(backend.my_chain.links)
            backend.current_joint_angles = backend.clamp_seed_angles_to_bounds([0.0] * joint_count)
            backend.refresh_locked_joint_rads(backend.current_joint_angles)
            backend.apply_chain_bounds_from_config()
            if not self._connect_serial_passive():
                self._append_log("Serial connection not established at startup.")
            elif not backend.sync_solver_seed_from_telemetry(update_target=True):
                backend.refresh_locked_joint_rads(backend.current_joint_angles)
                backend.apply_chain_bounds_from_config()
                backend.sync_target_from_joint_angles(backend.current_joint_angles)
            backend.planar_target_y = float(backend.current_target_xyz[1])
        except Exception as exc:
            traceback.print_exc()
            self._append_log(f"Backend init warning: {exc}")
            # Keep the UI alive so users can adjust connection settings and retry.
            try:
                if getattr(backend, "current_joint_angles", None) is None:
                    backend.current_joint_angles = [0.0] * 7
            except Exception:
                pass

    def _build_ui(self):
        root = QWidget()
        self.setCentralWidget(root)
        root_layout = QVBoxLayout(root)
        root_layout.setContentsMargins(8, 8, 8, 8)
        root_layout.setSpacing(6)

        splitter = QSplitter(Qt.Horizontal)
        root_layout.addWidget(splitter, 1)

        left_scroll = QScrollArea()
        left_scroll.setWidgetResizable(True)
        left_scroll.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        splitter.addWidget(left_scroll)

        left_panel = QWidget()
        left_layout = QVBoxLayout(left_panel)
        left_layout.setContentsMargins(0, 0, 0, 0)
        left_layout.setSpacing(6)
        left_panel.setMaximumWidth(330)
        left_scroll.setWidget(left_panel)

        self.canvas = ArmCanvas(self)
        splitter.addWidget(self.canvas)
        splitter.setSizes([310, 970])
        splitter.setStretchFactor(0, 0)
        splitter.setStretchFactor(1, 1)

        title = QLabel("Robot Arm IK Dashboard")
        title.setAlignment(Qt.AlignCenter)
        title.setStyleSheet("font-size: 18px; font-weight: bold; padding: 4px;")
        root_layout.insertWidget(0, title)

        controls_group = QGroupBox("Controls")
        controls_layout = QGridLayout(controls_group)
        controls_layout.setHorizontalSpacing(4)
        controls_layout.setVerticalSpacing(4)
        left_layout.addWidget(controls_group)

        self.btn_connect = QPushButton()
        self.btn_connect.clicked.connect(self.toggle_connection)
        controls_layout.addWidget(self.btn_connect, 0, 0, 1, 2)

        self.btn_execute = QPushButton("Execute")
        self.btn_execute.clicked.connect(lambda: self.execute_now())
        controls_layout.addWidget(self.btn_execute, 1, 0)

        self.btn_estop = QPushButton("E-Stop")
        self.btn_estop.clicked.connect(lambda: self.estop())
        controls_layout.addWidget(self.btn_estop, 1, 1)

        self.btn_alt = QPushButton("Alternative IK")
        self.btn_alt.clicked.connect(lambda: self.find_alternative())
        controls_layout.addWidget(self.btn_alt, 2, 0)

        self.btn_preview = QPushButton("Preview")
        self.btn_preview.clicked.connect(lambda: self.preview_plan())
        controls_layout.addWidget(self.btn_preview, 2, 1)

        self.btn_zero = QPushButton("Set Current Pose as Zero")
        self.btn_zero.clicked.connect(lambda: self.capture_zero())
        controls_layout.addWidget(self.btn_zero, 3, 0, 1, 2)

        self.btn_move_zero = QPushButton("Move Configured Joints to Zero")
        self.btn_move_zero.clicked.connect(lambda: self.move_to_zero())
        controls_layout.addWidget(self.btn_move_zero, 4, 0, 1, 2)

        self.chk_live = QCheckBox("Live Execute")
        self.chk_live.setChecked(backend.live_update)
        self.chk_live.toggled.connect(self.toggle_live_execute)
        controls_layout.addWidget(self.chk_live, 5, 0)

        self.chk_sync = QCheckBox("Sync Ghost to Telemetry")
        self.chk_sync.setChecked(backend.sync_model_to_telemetry)
        self.chk_sync.toggled.connect(self.toggle_sync)
        controls_layout.addWidget(self.chk_sync, 5, 1)

        self.chk_mirror = QCheckBox("Mirror Display X")
        self.chk_mirror.setChecked(backend.DISPLAY_MIRROR_X)
        self.chk_mirror.toggled.connect(self.toggle_mirror)
        controls_layout.addWidget(self.chk_mirror, 6, 0)

        self.btn_sync_target = QPushButton("Target From Telemetry")
        self.btn_sync_target.clicked.connect(lambda: self.sync_target_from_telemetry())
        controls_layout.addWidget(self.btn_sync_target, 6, 1)

        self.joint_speed_input = NoScrollSpinBox()
        self.joint_speed_input.setRange(1, 360)
        self.joint_speed_input.setSingleStep(1)
        self.joint_speed_input.setValue(20)
        self.joint_speed_input.valueChanged.connect(lambda _: self._apply_joint_speed_setting(log_change=True))
        controls_layout.addWidget(QLabel("Joint Speed (deg/s)"), 7, 0)
        controls_layout.addWidget(self.joint_speed_input, 7, 1)

        self.btn_verify_apply = QPushButton("Post-Reconnect Verify + Arm Motion")
        self.btn_verify_apply.clicked.connect(self.log_runtime_apply_plan)
        controls_layout.addWidget(self.btn_verify_apply, 8, 0, 1, 2)

        self.chk_allow_partial_connect = QCheckBox("Allow Partial Controller Connect")
        self.chk_allow_partial_connect.setChecked(True)
        self.chk_allow_partial_connect.setToolTip("If enabled, dashboard runs even if only some controllers connect.")
        controls_layout.addWidget(self.chk_allow_partial_connect, 9, 0, 1, 2)

        status_group = QGroupBox("Link Status")
        status_layout = QVBoxLayout(status_group)
        status_layout.setContentsMargins(6, 6, 6, 6)
        status_layout.setSpacing(4)
        left_layout.addWidget(status_group)

        self.status_label = QLabel()
        self.status_label.setAlignment(Qt.AlignTop | Qt.AlignLeft)
        self.status_label.setTextInteractionFlags(Qt.TextSelectableByMouse)
        status_layout.addWidget(self.status_label)

        tele_group = QGroupBox("Telemetry")
        tele_group_layout = QVBoxLayout(tele_group)
        tele_group_layout.setContentsMargins(6, 6, 6, 6)
        tele_group_layout.setSpacing(4)
        left_layout.addWidget(tele_group)

        self.telemetry_view = QTextEdit()
        self.telemetry_view.setReadOnly(True)
        self.telemetry_view.setMinimumHeight(160)
        tele_group_layout.addWidget(self.telemetry_view)

        hand_group = QGroupBox("Hand")
        hand_layout = QGridLayout(hand_group)
        hand_layout.setHorizontalSpacing(4)
        hand_layout.setVerticalSpacing(4)
        left_layout.addWidget(hand_group)

        self.btn_hand_open = QPushButton("Open")
        self.btn_hand_open.clicked.connect(lambda: self.hand_open())
        hand_layout.addWidget(self.btn_hand_open, 0, 0)

        self.btn_hand_close = QPushButton("Close")
        self.btn_hand_close.clicked.connect(lambda: self.hand_close())
        hand_layout.addWidget(self.btn_hand_close, 0, 1)

        self.btn_hand_stop = QPushButton("Stop")
        self.btn_hand_stop.clicked.connect(lambda: self.hand_stop())
        hand_layout.addWidget(self.btn_hand_stop, 1, 0, 1, 2)

        camera_group = QGroupBox("Camera")
        camera_layout = QGridLayout(camera_group)
        camera_layout.setHorizontalSpacing(4)
        camera_layout.setVerticalSpacing(4)
        left_layout.addWidget(camera_group)

        self.btn_cam_left = QPushButton("Left")
        self.btn_cam_left.clicked.connect(lambda: self.adjust_camera(0.0, -10.0))
        camera_layout.addWidget(self.btn_cam_left, 0, 0)

        self.btn_cam_right = QPushButton("Right")
        self.btn_cam_right.clicked.connect(lambda: self.adjust_camera(0.0, 10.0))
        camera_layout.addWidget(self.btn_cam_right, 0, 1)

        self.btn_cam_up = QPushButton("Up")
        self.btn_cam_up.clicked.connect(lambda: self.adjust_camera(10.0, 0.0))
        camera_layout.addWidget(self.btn_cam_up, 1, 0)

        self.btn_cam_down = QPushButton("Down")
        self.btn_cam_down.clicked.connect(lambda: self.adjust_camera(-10.0, 0.0))
        camera_layout.addWidget(self.btn_cam_down, 1, 1)

        self.btn_cam_reset = QPushButton("Reset")
        self.btn_cam_reset.clicked.connect(lambda: self.reset_camera())
        camera_layout.addWidget(self.btn_cam_reset, 2, 0, 1, 2)

        target_group = QGroupBox("Target")
        target_layout = QGridLayout(target_group)
        target_layout.setHorizontalSpacing(4)
        target_layout.setVerticalSpacing(4)
        left_layout.addWidget(target_group)

        self.target_inputs = {}
        for row, key in enumerate(("x", "y", "z", "roll", "pitch", "yaw")):
            label = QLabel(key.upper())
            spin = NoScrollDoubleSpinBox()
            spin.setDecimals(3)
            spin.setSingleStep(0.01)
            if key == "z":
                spin.setRange(0.0, 1.5)
            elif key in ("roll", "pitch", "yaw"):
                spin.setDecimals(1)
                spin.setSingleStep(1.0)
                spin.setRange(-180.0, 180.0)
            else:
                spin.setRange(-1.0, 1.0)
            spin.valueChanged.connect(self.on_target_changed)
            spin.installEventFilter(self)  # Track focus for editing
            target_layout.addWidget(label, row, 0)
            target_layout.addWidget(spin, row, 1)
            self.target_inputs[key] = spin

        self.step_input = NoScrollDoubleSpinBox()
        self.step_input.setDecimals(3)
        self.step_input.setRange(0.001, 0.05)
        self.step_input.setSingleStep(0.001)
        self.step_input.setValue(0.01)
        target_layout.addWidget(QLabel("Key Step"), 6, 0)
        target_layout.addWidget(self.step_input, 6, 1)

        joint_group = QGroupBox("Joint Targets (deg)")
        joint_layout = QGridLayout(joint_group)
        joint_layout.setHorizontalSpacing(4)
        joint_layout.setVerticalSpacing(4)
        left_layout.addWidget(joint_group)

        self.joint_inputs = {}
        for row, joint_idx in enumerate(range(1, 7)):
            label = QLabel(f"J{joint_idx}")
            spin = NoScrollDoubleSpinBox()
            spin.setDecimals(2)
            spin.setRange(-360.0, 360.0)
            spin.setSingleStep(1.0)
            spin.valueChanged.connect(lambda value, idx=joint_idx: self.on_joint_changed(idx, value))
            spin.installEventFilter(self)  # Track focus for editing
            joint_layout.addWidget(label, row, 0)
            joint_layout.addWidget(spin, row, 1)
            self.joint_inputs[joint_idx] = spin

        self.joint_table = QTableWidget(6, 4)
        self.joint_table.setHorizontalHeaderLabels(["Joint", "Target", "Current", "Error"])
        self.joint_table.verticalHeader().setVisible(False)
        self.joint_table.horizontalHeader().setSectionResizeMode(QHeaderView.Stretch)
        left_layout.addWidget(self.joint_table)

        tabs = QTabWidget()
        self.info_tabs = tabs
        left_layout.addWidget(tabs, 1)

        log_tab = QWidget()
        log_layout = QVBoxLayout(log_tab)
        self.log_view = QTextEdit()
        self.log_view.setReadOnly(True)
        log_layout.addWidget(self.log_view)
        tabs.addTab(log_tab, "Event Log")

        tele_tab = QWidget()
        tele_layout = QVBoxLayout(tele_tab)
        self.telemetry_detail_view = QTextEdit()
        self.telemetry_detail_view.setReadOnly(True)
        tele_layout.addWidget(self.telemetry_detail_view)
        tabs.addTab(tele_tab, "Telemetry")
        self.telemetry_tab_index = tabs.indexOf(tele_tab)

        # Wrist visualization tab
        wrist_viz_tab = QWidget()
        wrist_viz_layout = QVBoxLayout(wrist_viz_tab)
        self.wrist_viz_widget = WristVisualizationWidget()
        wrist_viz_layout.addWidget(self.wrist_viz_widget)
        tabs.addTab(wrist_viz_tab, "Wrist Viz")
        self.wrist_viz_tab_index = tabs.indexOf(wrist_viz_tab)

        help_tab = QWidget()
        help_layout = QVBoxLayout(help_tab)
        self.help_view = QTextEdit()
        self.help_view.setReadOnly(True)
        self.help_view.setPlainText(
            "Controls\n\n"
            "- Connect: open/close serial link\n"
            "- Execute: send current joint target\n"
            "- Preview: copy target to actual preview pose\n"
            "- E-Stop: send stop command\n"
            "- Set Current Pose as Zero: save abs zero offsets\n"
            "- Move Configured Joints to Zero: send configured zero command\n"
            "- Target From Telemetry: align IK target with current real pose\n"
            "- Mirror Display X: flip the rendered view only\n"
            "- Key Bindings: remap controls and save them to ik_dashboard_keys.json\n"
            "- Startup Config: applies stored robot_config settings and telemetry bring-up on connect\n"
        )
        help_layout.addWidget(self.help_view)
        tabs.addTab(help_tab, "Help")

        keys_group = QGroupBox("Keyboard")
        keys_layout = QVBoxLayout(keys_group)
        keys_layout.setContentsMargins(6, 6, 6, 6)
        keys_layout.setSpacing(4)
        left_layout.addWidget(keys_group)

        self.keys_help = QLabel(
            "Keyboard mappings are editable below and saved to disk.\n"
            "Click the dashboard first so it has focus."
        )
        self.keys_help.setWordWrap(True)
        keys_layout.addWidget(self.keys_help)

        self.keys_status = QLabel("Pressed: <none>")
        self.keys_status.setWordWrap(True)
        self.keys_status.setStyleSheet("font-family: monospace; background: #f4f4f4; padding: 4px; border-radius: 4px;")
        keys_layout.addWidget(self.keys_status)

        keymap_group = QGroupBox("Key Bindings")
        keymap_layout = QGridLayout(keymap_group)
        keymap_layout.setHorizontalSpacing(4)
        keymap_layout.setVerticalSpacing(4)
        left_layout.addWidget(keymap_group)

        action_order = [
            "move_x_neg", "move_x_pos", "move_y_pos", "move_y_neg", "move_z_neg", "move_z_pos",
            "roll_neg", "roll_pos", "pitch_pos", "pitch_neg", "yaw_neg", "yaw_pos",
            "cam_left", "cam_right", "cam_up", "cam_down",
            "execute", "preview", "sync_target", "estop", "hand_open", "hand_close",
        ]
        for row, action in enumerate(action_order):
            label = QLabel(ACTION_LABELS[action])
            combo = QComboBox()
            combo.addItems(KEY_OPTIONS)
            combo.setCurrentText(self.keymap.get(action, "None"))
            combo.currentTextChanged.connect(lambda value, act=action: self.on_keymap_changed(act, value))
            indicator = QLabel()
            indicator.setAlignment(Qt.AlignCenter)
            indicator.setMinimumHeight(24)
            indicator.setStyleSheet("background-color: #e9ecef; color: #222; border: 1px solid #c5c9ce; border-radius: 4px; font-weight: bold;")
            keymap_layout.addWidget(label, row, 0)
            keymap_layout.addWidget(combo, row, 1)
            keymap_layout.addWidget(indicator, row, 2)
            self.keymap_inputs[action] = combo
            self.key_indicator_labels[action] = indicator

        self.btn_save_keys = QPushButton("Save Key Bindings")
        self.btn_save_keys.clicked.connect(self.save_keymap)
        keymap_layout.addWidget(self.btn_save_keys, len(action_order), 0, 1, 2)

        self.btn_reset_keys = QPushButton("Reset Defaults")
        self.btn_reset_keys.clicked.connect(self.reset_keymap_defaults)
        keymap_layout.addWidget(self.btn_reset_keys, len(action_order), 2)

        left_layout.addStretch(1)

    def _load_keymap(self):
        if not os.path.exists(KEYMAP_FILE):
            return dict(DEFAULT_KEYMAP)
        try:
            with open(KEYMAP_FILE, "r", encoding="utf-8") as f:
                data = json.load(f)
            if not isinstance(data, dict):
                return dict(DEFAULT_KEYMAP)
            out = dict(DEFAULT_KEYMAP)
            for action in DEFAULT_KEYMAP:
                value = str(data.get(action, out[action]))
                out[action] = value if value in KEY_OPTIONS else out[action]
            return out
        except Exception:
            return dict(DEFAULT_KEYMAP)

    def save_keymap(self):
        try:
            with open(KEYMAP_FILE, "w", encoding="utf-8") as f:
                json.dump(self.keymap, f, indent=2)
            self._append_log(f"Saved key bindings to {os.path.basename(KEYMAP_FILE)}.")
        except Exception as exc:
            self._append_log(f"Failed to save key bindings: {exc}")

    def reset_keymap_defaults(self):
        self.keymap = dict(DEFAULT_KEYMAP)
        for action, combo in self.keymap_inputs.items():
            combo.blockSignals(True)
            combo.setCurrentText(self.keymap.get(action, "None"))
            combo.blockSignals(False)
        self.save_keymap()
        self.refresh_all(skip_inputs=True)

    def on_keymap_changed(self, action, value):
        self.keymap[action] = value if value in KEY_OPTIONS else "None"
        self.refresh_all(skip_inputs=True, draw_canvas=False)

    def _connect_serial_passive(self):
        was_connected = self._backend_connected()
        try:
            ok = backend.connect_serial(perform_startup_actions=False)
        except Exception as exc:
            self._append_log(f"Serial connect failed: {exc}")
            return False
        if not ok:
            return False

        allow_partial = True
        if hasattr(self, "chk_allow_partial_connect"):
            allow_partial = bool(self.chk_allow_partial_connect.isChecked())
        if not allow_partial:
            serial_map = getattr(backend, "controller_serial", {}) or {}
            connected = {
                str(name)
                for name, conn in serial_map.items()
                if conn and getattr(conn, "is_open", False)
            }
            cfg_map = getattr(backend, "controller_map", {}) or {}
            required = set()
            if isinstance(cfg_map, dict):
                for name, cfg in cfg_map.items():
                    if not isinstance(cfg, dict):
                        continue
                    port = str(cfg.get("port", "") or "").strip().upper()
                    if port and port != "COM0":
                        required.add(str(name))
            missing = sorted(required - connected)
            if missing:
                backend.disconnect_serial()
                self._append_log(
                    "Strict connect mode blocked startup; missing controller(s): "
                    + ", ".join(missing)
                )
                return False

        backend.send_command("S; TON")
        self._engage_motion_safety_lock("serial connect")
        if backend.wait_for_telemetry(timeout_sec=1.5):
            self._append_log("Connected serial device; telemetry is live.")
        else:
            self._append_log("Connected serial device; waiting for telemetry.")
        reason = "manual reconnect" if was_connected else "startup connect"
        self._apply_runtime_config_bundle(reason)
        self._last_auto_apply_time = time.time()
        self._last_telemetry_live = bool(backend.has_fresh_telemetry())
        return True

    def _apply_runtime_config_bundle(self, reason):
        try:
            backend.send_command("S; TON")
            backend.load_robot_config()
            applied = backend.apply_robot_config_to_arm()
            self._last_runtime_apply_ok = bool(applied)
            if applied:
                self._append_log(f"Auto-applied config/ENCMAP/ABSPID after {reason}.")
                backend.wait_for_telemetry(timeout_sec=1.5)
                if backend.sync_solver_seed_from_telemetry(update_target=True, require_all_controllers=False):
                    self._append_log("Synchronized dashboard pose from telemetry.")
                else:
                    all_ctrl_ok = bool(getattr(backend, "all_required_controllers_healthy", lambda require_fresh=True: False)(True))
                    if not all_ctrl_ok:
                        self._append_log("Pose sync blocked: required controller telemetry is missing/stale.")
                    else:
                        self._append_log("Pose sync from telemetry not ready after auto-apply.")
            else:
                self._append_log(f"Auto-apply failed after {reason}; motion stays locked.")
                self._engage_motion_safety_lock("runtime apply failed")
            return applied
        except Exception as exc:
            self._last_runtime_apply_ok = False
            self._append_log(f"Auto-apply failed after {reason}: {exc}")
            self._engage_motion_safety_lock("runtime apply exception")
            return False

    def _engage_motion_safety_lock(self, reason):
        self._motion_armed = False
        self._motion_block_reason = str(reason)
        self._motion_block_logged = False
        reset_motion = getattr(backend, "reset_runtime_motion_state", None)
        if callable(reset_motion):
            reset_motion()
        if bool(getattr(backend, "live_update", False)):
            backend.live_update = False
            if hasattr(self, "chk_live"):
                self.chk_live.blockSignals(True)
                self.chk_live.setChecked(False)
                self.chk_live.blockSignals(False)
        self._append_log(
            f"Motion lock engaged ({reason}). Run 'Post-Reconnect Verify + Arm Motion' before Execute/Live motion."
        )

    def _is_motion_allowed(self):
        if not self._last_runtime_apply_ok:
            if not self._motion_block_logged:
                self._append_log("Motion blocked: runtime config apply is not confirmed. Reconnect and verify.")
                self._motion_block_logged = True
            return False
        all_ctrl_ok = bool(getattr(backend, "all_required_controllers_healthy", lambda require_fresh=True: False)(False))
        if not all_ctrl_ok:
            if not self._motion_block_logged:
                self._append_log("Motion blocked: one or more required controllers are offline.")
                self._motion_block_logged = True
            return False
        if not self._backend_connected() or not bool(getattr(backend, "has_fresh_telemetry", lambda: False)()):
            if not self._motion_block_logged:
                self._append_log("Motion blocked: telemetry is not live. Reconnect and run 'Post-Reconnect Verify + Arm Motion'.")
                self._motion_block_logged = True
            return False
        if self._motion_armed:
            return True
        if not self._motion_block_logged:
            self._append_log(
                f"Motion blocked: {self._motion_block_reason}. Click 'Post-Reconnect Verify + Arm Motion'."
            )
            self._motion_block_logged = True
        return False

    def log_runtime_apply_plan(self):
        try:
            if not self._last_runtime_apply_ok:
                self._append_log("Verify blocked: runtime apply not confirmed. Reconnect to retry auto-apply.")
                self._engage_motion_safety_lock("runtime apply not confirmed")
                return
            backend.load_robot_config()
            planner = getattr(backend, "get_expected_runtime_apply_plan", None)
            if planner is None:
                self._append_log("Runtime verify unavailable: backend planner not found.")
                return
            plan = planner() or {}
            connected = plan.get("connected_controllers", [])
            self._append_log(
                "Verify plan: connected controllers = "
                + (", ".join(connected) if connected else "none")
            )
            self._append_log(
                f"Verify plan: wrist diff enabled={bool(plan.get('wrist_diff_enabled', False))}, "
                f"wrist online={bool(plan.get('wrist_online', False))}."
            )

            enc_map_by_controller = plan.get("enc_map_by_controller", {}) or {}
            for controller_name in sorted(enc_map_by_controller.keys()):
                mapping = enc_map_by_controller.get(controller_name, {}) or {}
                if mapping:
                    mapped_text = ", ".join(
                        f"E{int(enc)}->M{int(mid)}" for enc, mid in sorted(mapping.items(), key=lambda item: int(item[0]))
                    )
                else:
                    mapped_text = "(no ENCMAP assignments)"
                self._append_log(f"Verify ENCMAP[{controller_name}]: {mapped_text}")

            abspid_map = plan.get("abspid_enable_by_motor", {}) or {}
            abspid_text = ", ".join(
                f"M{int(mid)}={int(enable)}" for mid, enable in sorted(abspid_map.items(), key=lambda item: int(item[0]))
            ) if abspid_map else "(none)"
            self._append_log(f"Verify ABSPID: {abspid_text}")

            skipped = plan.get("abspid_skipped_unmapped", []) or []
            if skipped:
                self._append_log(
                    "Verify note: ABSPID disabled for unmapped motors: "
                    + ", ".join(f"M{int(mid)}" for mid in skipped)
                )

            skip_state_getter = getattr(backend, "get_abspid_skip_state", None)
            runtime_skips = skip_state_getter() if callable(skip_state_getter) else {}
            if runtime_skips:
                details = ", ".join(
                    f"M{int(mid)} ({reason})" for mid, reason in sorted(runtime_skips.items(), key=lambda item: int(item[0]))
                )
                self._append_log("Verify failed: firmware reported ABSPID skips: " + details)
                self._append_log("Motion remains locked. Re-run apply/verify after encoder detection is stable.")
                return

            if not backend.sync_solver_seed_from_telemetry(update_target=True, require_all_controllers=False):
                self._append_log("Verify warning: telemetry sync unavailable; motion remains locked.")
                return

            backend.actual_joint_angles = list(backend.current_joint_angles)
            backend.refresh_locked_joint_rads(backend.actual_joint_angles)
            self._motion_armed = True
            self._motion_block_reason = ""
            self._motion_block_logged = False
            self._append_log("Verify complete: motion armed using current telemetry pose.")
        except Exception as exc:
            self._append_log(f"Runtime verify failed: {exc}")

    def _start_timers(self):
        self.refresh_timer = QTimer(self)
        self.refresh_timer.timeout.connect(self.refresh_all)
        self.refresh_timer.start(250)

        self.live_timer = QTimer(self)
        self.live_timer.timeout.connect(self._live_execute_tick)
        self.live_timer.start(100)

        self.keyboard_timer = QTimer(self)
        self.keyboard_timer.timeout.connect(self._keyboard_tick)
        self.keyboard_timer.start(50)  # Slightly slower for responsiveness

    def _apply_joint_speed_setting(self, log_change=False):
        if not hasattr(self, "joint_speed_input"):
            return
        speed = max(1.0, float(self.joint_speed_input.value()))
        prev_speed = float(getattr(backend, "MAX_COMMAND_RATE_DEG_PER_SEC", speed))
        backend.MAX_COMMAND_RATE_DEG_PER_SEC = speed
        if log_change and abs(prev_speed - speed) > 1e-6:
            self._append_log(f"Joint speed set to {speed:.0f} deg/s.")

    def _append_log(self, message):
        stamp = time.strftime("%H:%M:%S")
        line = f"[{stamp}] {message}"
        self._log_lines.append(line)
        self._log_lines = self._log_lines[-200:]
        if hasattr(self, "log_view"):
            self.log_view.setPlainText("\n".join(self._log_lines))
            self.log_view.verticalScrollBar().setValue(self.log_view.verticalScrollBar().maximum())

    def _send_current_joint_target(self, smooth=False, repeat_count=1):
        if not self._is_motion_allowed():
            return False
        self._apply_joint_speed_setting(log_change=False)
        backend.current_joint_angles = backend.clamp_seed_angles_to_bounds(
            backend.enforce_locked_joints(backend.current_joint_angles)
        )
        backend.actual_joint_angles = list(backend.current_joint_angles)
        backend.refresh_locked_joint_rads(backend.actual_joint_angles)
        backend.send_joints_to_arm(
            backend.current_joint_angles,
            smooth=smooth,
            max_ramp_steps=20 if smooth else 1,
            force_immediate=False,
            repeat_count=repeat_count,
        )
        backend.last_send_time = time.time()
        return True

    def toggle_connection(self):
        if self._backend_connected():
            backend.disconnect_serial()
            self._last_runtime_apply_ok = False
            self._append_log("Disconnected serial device.")
        else:
            ok = self._connect_serial_passive()
            self._append_log("Serial connection failed." if not ok else "Passive startup complete: no motion commands sent.")
        self.refresh_all()

    def execute_now(self):
        sent = self._send_current_joint_target(smooth=True, repeat_count=1)
        self._append_log("Execute sent." if sent else "Execute blocked (see motion lock/status log).")
        self.refresh_all()

    def preview_plan(self):
        if backend.actual_joint_angles is None:
            tele_angles = backend.get_current_real_joint_angles()
            backend.actual_joint_angles = list(tele_angles) if tele_angles is not None else list(backend.current_joint_angles)
        backend.is_previewing = True
        backend.preview_start_time = time.time()
        self._append_log("Preview animation started (current -> target).")
        self.refresh_all()

    def estop(self):
        self._pressed_keys.clear()
        self._key_action_fired.clear()
        self._engage_motion_safety_lock("manual E-Stop")
        backend.emergency_stop_callback(None)
        self._append_log("Emergency stop sent.")
        self.refresh_all()

    def find_alternative(self):
        backend.find_alternative_ik(None)
        self._append_log("Alternative IK search requested.")
        self.refresh_all()

    def capture_zero(self):
        backend.set_current_pose_as_motor_and_ik_zero(None)
        self._append_log("Captured current pose as zero.")
        self.refresh_all()

    def move_to_zero(self):
        moved = backend.move_configured_joints_to_zero_on_connect()
        self._append_log("Move-to-zero requested." if moved else "Move-to-zero skipped.")
        self.refresh_all()

    def hand_open(self):
        backend.open_hand(255)
        self._append_log("Hand open requested.")
        self.refresh_all(skip_inputs=True)

    def hand_close(self):
        backend.close_hand(255)
        self._append_log("Hand close requested.")
        self.refresh_all(skip_inputs=True)

    def hand_stop(self):
        backend.stop_hand()
        self._append_log("Hand stop requested.")
        self.refresh_all(skip_inputs=True)

    def _set_hand_from_keys(self, actions):
        open_pressed = "hand_open" in actions
        close_pressed = "hand_close" in actions
        desired = None
        if open_pressed and not close_pressed:
            desired = "open"
        elif close_pressed and not open_pressed:
            desired = "close"
        if desired == self._hand_key_active:
            return
        if desired == "open":
            backend.open_hand(255)
            self._hand_key_active = desired
        elif desired == "close":
            backend.close_hand(255)
            self._hand_key_active = desired
        else:
            if self._hand_key_active is not None:
                backend.stop_hand()
            self._hand_key_active = None

    def toggle_live_execute(self, checked):
        backend.live_update = bool(checked)
        self._append_log(f"Live Execute {'enabled' if checked else 'disabled'}.")

    def toggle_sync(self, checked):
        backend.sync_model_to_telemetry = bool(checked)
        self._append_log(f"Sync to telemetry {'enabled' if checked else 'disabled'}.")
        self.refresh_all()

    def toggle_mirror(self, checked):
        backend.DISPLAY_MIRROR_X = bool(checked)
        self._append_log(f"Display mirror {'enabled' if checked else 'disabled' }.")
        self.refresh_all(skip_inputs=True)

    def sync_target_from_telemetry(self):
        if backend.sync_solver_seed_from_telemetry(update_target=True, require_all_controllers=False):
            self._append_log("Target synchronized from telemetry pose.")
        else:
            self._append_log("Target sync from telemetry skipped: no telemetry.")
        self.refresh_all()

    def adjust_camera(self, elev_delta, azim_delta):
        backend.cam_elev = max(-90.0, min(90.0, backend.cam_elev + elev_delta))
        backend.cam_azim += azim_delta
        self.refresh_all(skip_inputs=True)

    def reset_camera(self):
        backend.cam_elev = 20.0
        backend.cam_azim = -60.0
        self.refresh_all(skip_inputs=True)

    def _sorted_pressed_keys(self):
        return sorted(self._pressed_keys, key=lambda v: (len(v), v))

    def _pressed_actions(self):
        actions = set()
        for action, key_name in self.keymap.items():
            if key_name and key_name != "None" and key_name.lower() in self._pressed_keys:
                actions.add(action)
        return actions

    def _event_to_key_name(self, event):
        key = event.key()
        text = event.text().upper() if event.text() else ""
        if key == Qt.Key_Left:
            return "Left"
        if key == Qt.Key_Right:
            return "Right"
        if key == Qt.Key_Up:
            return "Up"
        if key == Qt.Key_Down:
            return "Down"
        if key == Qt.Key_Space:
            return "Space"
        if text in {k for k in KEY_OPTIONS if len(k) == 1 and k.isalpha()}:
            return text
        return None

    def _keyboard_tick(self):
        if not self._pressed_keys:
            if self._hand_key_active is not None:
                backend.stop_hand()
                self._hand_key_active = None
            self._key_action_fired.clear()
            return

        move_step = float(self.step_input.value()) if hasattr(self, "step_input") else 0.01
        rot_step = 2.0
        cam_step = 3.0
        changed = False

        actions = self._pressed_actions()
        self._set_hand_from_keys(actions)

        if "move_x_neg" in actions:
            backend.current_target_xyz[0] -= move_step
            changed = True
        if "move_x_pos" in actions:
            backend.current_target_xyz[0] += move_step
            changed = True
        if "move_y_pos" in actions:
            backend.planar_target_y += move_step
            changed = True
        if "move_y_neg" in actions:
            backend.planar_target_y -= move_step
            changed = True
        if "move_z_neg" in actions:
            backend.current_target_xyz[2] -= move_step
            changed = True
        if "move_z_pos" in actions:
            backend.current_target_xyz[2] += move_step
            changed = True
        if "roll_neg" in actions:
            backend.current_target_rpy[0] -= rot_step
            changed = True
        if "roll_pos" in actions:
            backend.current_target_rpy[0] += rot_step
            changed = True
        if "pitch_pos" in actions:
            backend.current_target_rpy[1] += rot_step
            changed = True
        if "pitch_neg" in actions:
            backend.current_target_rpy[1] -= rot_step
            changed = True
        if "yaw_neg" in actions:
            backend.current_target_rpy[2] -= rot_step
            changed = True
        if "yaw_pos" in actions:
            backend.current_target_rpy[2] += rot_step
            changed = True

        if "cam_left" in actions:
            backend.cam_azim -= cam_step
            changed = True
        if "cam_right" in actions:
            backend.cam_azim += cam_step
            changed = True
        if "cam_up" in actions:
            backend.cam_elev = max(-90.0, min(90.0, backend.cam_elev + cam_step))
            changed = True
        if "cam_down" in actions:
            backend.cam_elev = max(-90.0, min(90.0, backend.cam_elev - cam_step))
            changed = True

        edge_actions = {
            "execute": self.execute_now,
            "estop": self.estop,
            "preview": self.preview_plan,
            "sync_target": self.sync_target_from_telemetry,
        }
        for action, callback in edge_actions.items():
            if action in actions and action not in self._key_action_fired:
                callback()
                self._key_action_fired.add(action)
            elif action not in actions:
                self._key_action_fired.discard(action)

        if not changed:
            return

        backend.current_target_xyz[0] = max(-1.0, min(1.0, backend.current_target_xyz[0]))
        backend.planar_target_y = max(-1.0, min(1.0, backend.planar_target_y))
        backend.current_target_xyz[1] = backend.planar_target_y if backend.PLANAR_MODE else max(-1.0, min(1.0, backend.current_target_xyz[1]))
        backend.current_target_xyz[2] = max(0.0, min(1.5, backend.current_target_xyz[2]))
        backend.current_target_rpy = [
            max(-180.0, min(180.0, float(v)))
            for v in backend.current_target_rpy
        ]

        solved = backend.solve_ik_for_target(backend.current_joint_angles)
        if solved is not None:
            backend.current_joint_angles = list(solved)
            backend.refresh_locked_joint_rads(backend.current_joint_angles)
        self.refresh_all(skip_inputs=False, draw_canvas=False)

    def keyPressEvent(self, event):
        mapped = self._event_to_key_name(event)
        if mapped is not None:
            self._pressed_keys.add(mapped.lower())
            self.refresh_all(skip_inputs=True, draw_canvas=False)
            event.accept()
            return
        super().keyPressEvent(event)

    def keyReleaseEvent(self, event):
        mapped = self._event_to_key_name(event)
        if mapped is not None and mapped.lower() in self._pressed_keys:
            self._pressed_keys.discard(mapped.lower())
            self.refresh_all(skip_inputs=True, draw_canvas=False)
            event.accept()
            return
        super().keyReleaseEvent(event)

    def eventFilter(self, obj, event):
        """Track when inputs are being edited to prevent overwriting."""
        # Check if attributes exist (they may not during initialization)
        if not hasattr(self, '_editing_inputs'):
            return super().eventFilter(obj, event)
        
        # Find which input this is
        input_key = None
        if hasattr(self, 'target_inputs'):
            for key, spin in self.target_inputs.items():
                if spin is obj:
                    input_key = key
                    break
        if input_key is None and hasattr(self, 'joint_inputs'):
            for joint_idx, spin in self.joint_inputs.items():
                if spin is obj:
                    input_key = f"joint_{joint_idx}"
                    break
        
        if input_key is not None:
            if event.type() == QEvent.Type.FocusIn:
                self._editing_inputs.add(input_key)
            elif event.type() == QEvent.Type.FocusOut:
                self._editing_inputs.discard(input_key)
        
        return super().eventFilter(obj, event)

    def on_target_changed(self):
        if self._building_controls:
            return
        backend.current_target_xyz = [
            float(self.target_inputs["x"].value()),
            float(self.target_inputs["y"].value()),
            float(self.target_inputs["z"].value()),
        ]
        backend.current_target_rpy = [
            float(self.target_inputs["roll"].value()),
            float(self.target_inputs["pitch"].value()),
            float(self.target_inputs["yaw"].value()),
        ]
        backend.planar_target_y = float(self.target_inputs["y"].value())
        solved = backend.solve_ik_for_target(backend.current_joint_angles)
        if solved is not None:
            backend.current_joint_angles = list(solved)
            backend.refresh_locked_joint_rads(backend.current_joint_angles)
        self.refresh_all(skip_inputs=True)

    def on_joint_changed(self, joint_idx, value):
        if self._building_controls:
            return
        if joint_idx < len(backend.current_joint_angles):
            backend.current_joint_angles[joint_idx] = np.radians(float(value))
        backend.current_joint_angles = backend.enforce_locked_joints(backend.current_joint_angles)
        backend.refresh_locked_joint_rads(backend.current_joint_angles)
        solved = backend.solve_ik_for_target(backend.current_joint_angles)
        if solved is not None:
            backend.current_joint_angles = list(solved)
            backend.refresh_locked_joint_rads(backend.current_joint_angles)
            backend.sync_target_from_joint_angles(backend.current_joint_angles)
        self.refresh_all(skip_inputs=True)

    def _live_execute_tick(self):
        if backend.live_update:
            if not self._is_motion_allowed():
                return
            now = time.time()
            if now - backend.last_send_time > 0.1:
                self._send_current_joint_target(smooth=False, repeat_count=1)
                self.refresh_all(skip_inputs=True)

    def refresh_all(self, skip_inputs=False, draw_canvas=True):
        self._building_controls = True
        try:
            if not skip_inputs:
                # Only update inputs that are not being edited
                for key in ("x", "y", "z", "roll", "pitch", "yaw"):
                    if key not in self._editing_inputs:
                        if key == "x":
                            self.target_inputs[key].setValue(float(backend.current_target_xyz[0]))
                        elif key == "y":
                            self.target_inputs[key].setValue(float(backend.planar_target_y if backend.PLANAR_MODE else backend.current_target_xyz[1]))
                        elif key == "z":
                            self.target_inputs[key].setValue(float(backend.current_target_xyz[2]))
                        elif key == "roll":
                            self.target_inputs[key].setValue(float(backend.current_target_rpy[0]))
                        elif key == "pitch":
                            self.target_inputs[key].setValue(float(backend.current_target_rpy[1]))
                        elif key == "yaw":
                            self.target_inputs[key].setValue(float(backend.current_target_rpy[2]))
                for joint_idx in range(1, 7):
                    if f"joint_{joint_idx}" not in self._editing_inputs and joint_idx < len(backend.current_joint_angles):
                        self.joint_inputs[joint_idx].setValue(float(np.degrees(backend.current_joint_angles[joint_idx])))

            connected = self._backend_connected()
            telemetry_live = bool(connected and backend.has_fresh_telemetry())
            controller_state = self._controller_runtime_state_snapshot()
            controller_recovered = False
            controller_degraded = False
            if self._last_controller_state_snapshot is not None:
                prev_map = {
                    name: (prev_connected, prev_fresh)
                    for name, prev_connected, prev_fresh in self._last_controller_state_snapshot
                }
                for name, is_connected, is_fresh in controller_state:
                    prev_connected, prev_fresh = prev_map.get(name, (False, False))
                    if is_connected and not prev_connected:
                        controller_recovered = True
                    if prev_connected and not is_connected:
                        controller_degraded = True
            self._last_controller_state_snapshot = controller_state

            if controller_degraded:
                self._engage_motion_safety_lock("controller degraded")

            if (telemetry_live and not self._last_telemetry_live) or controller_recovered:
                now = time.time()
                if (now - self._last_auto_apply_time) >= 2.0:
                    reason = "controller recovery" if controller_recovered else "telemetry reconnect"
                    self._engage_motion_safety_lock(reason)
                    self._apply_runtime_config_bundle(reason)
                    self._last_auto_apply_time = now
            self._last_telemetry_live = telemetry_live

            if telemetry_live:
                conn_state = ("Connected", "background-color: lightgreen;")
            elif connected:
                conn_state = ("Port Open", "background-color: khaki;")
            else:
                conn_state = ("Disconnected", "background-color: lightcoral;")
            if conn_state != self._last_connection_state:
                self.btn_connect.setText(conn_state[0])
                self.btn_connect.setStyleSheet(conn_state[1])
                self._last_connection_state = conn_state

            self.btn_estop.setStyleSheet("background-color: salmon; font-weight: bold;")
            status_text = backend.get_connection_status_text() + f"\nLast Telemetry: {backend.last_telemetry_line}"
            if status_text != self._last_status_text:
                self.status_label.setText(status_text)
                self._last_status_text = status_text
            self.chk_live.setChecked(bool(backend.live_update))
            self.chk_sync.setChecked(bool(backend.sync_model_to_telemetry))
            self.chk_mirror.setChecked(bool(backend.DISPLAY_MIRROR_X))
            pressed = self._sorted_pressed_keys()
            pressed_text = "Pressed: " + (", ".join(pressed) if pressed else "<none>")
            if pressed_text != self._last_pressed_text:
                self.keys_status.setText(pressed_text)
                self._last_pressed_text = pressed_text
            active_actions = self._pressed_actions()
            indicator_snapshot = (
                tuple(sorted(active_actions)),
                tuple((a, self.keymap.get(a, "None")) for a in sorted(self.key_indicator_labels.keys())),
            )
            if indicator_snapshot != self._last_indicator_snapshot:
                for action, label in self.key_indicator_labels.items():
                    key_name = self.keymap.get(action, "None")
                    if label.text() != key_name:
                        label.setText(key_name)
                    if action in active_actions:
                        label.setStyleSheet("background-color: #6fda8c; color: #0b2b12; border: 1px solid #2d7a40; border-radius: 4px; font-weight: bold;")
                    else:
                        label.setStyleSheet("background-color: #e9ecef; color: #222; border: 1px solid #c5c9ce; border-radius: 4px; font-weight: bold;")
                self._last_indicator_snapshot = indicator_snapshot

            telemetry_rows = []
            for joint_idx in range(1, 7):
                pos = backend.latest_motor_pos_deg.get(joint_idx)
                apos = backend.latest_motor_apos_deg.get(joint_idx)
                line = f"J{joint_idx}: POS={pos if pos is not None else '---'}  APOS={apos if apos is not None else '---'}"
                telemetry_rows.append(line)
            wrist = get_wrist_dashboard_snapshot()
            if wrist is not None:
                telemetry_rows.append("")
                telemetry_rows.append("Wrist Differential:")
                telemetry_rows.append(
                    f"Pitch J{wrist['pitch_joint_id']}: target={_fmt_deg(wrist['pitch_target']).strip()} current={_fmt_deg(wrist['pitch_actual']).strip()} enc=M{wrist['pitch_encoder_motor_id']}"
                )
                telemetry_rows.append(
                    f"Roll  J{wrist['roll_joint_id']}: target={_fmt_deg(wrist['roll_target']).strip()} current={_fmt_deg(wrist['roll_actual']).strip()} enc=M{wrist['roll_encoder_motor_id']}"
                )
                telemetry_rows.append(
                    f"Mixed motor cmds: M{wrist['motor_a_id']}={_fmt_rpm(wrist['motor_a_cmd']).strip()} RPM  M{wrist['motor_b_id']}={_fmt_rpm(wrist['motor_b_cmd']).strip()} RPM"
                )
            hand = get_hand_dashboard_snapshot()
            telemetry_rows.append("")
            telemetry_rows.append("Hand:")
            telemetry_rows.append(f"Controller: {hand['controller']}")
            telemetry_rows.append(f"PWM: {hand['pwm']}")

            telemetry_text = "\n".join(telemetry_rows)
            if telemetry_text != self._last_telemetry_text:
                self.telemetry_view.setPlainText(telemetry_text)
                if hasattr(self, "info_tabs") and self.info_tabs.currentIndex() == self.telemetry_tab_index:
                    self.telemetry_detail_view.setPlainText(telemetry_text)
                self._last_telemetry_text = telemetry_text

            # Update wrist visualization widget
            wrist_snap = get_wrist_dashboard_snapshot()
            if (
                wrist_snap
                and hasattr(self, "wrist_viz_widget")
                and hasattr(self, "info_tabs")
                and self.info_tabs.currentIndex() == self.wrist_viz_tab_index
            ):
                self.wrist_viz_widget.update_wrist(
                    pitch_actual=wrist_snap.get("pitch_actual"),
                    roll_actual=wrist_snap.get("roll_actual"),
                    pitch_target=wrist_snap.get("pitch_target"),
                    roll_target=wrist_snap.get("roll_target"),
                    motor_a_cmd=wrist_snap.get("motor_a_cmd"),
                    motor_b_cmd=wrist_snap.get("motor_b_cmd"),
                )

            for row, joint_idx in enumerate(range(1, 7)):
                tgt = float(np.degrees(backend.current_joint_angles[joint_idx])) if joint_idx < len(backend.current_joint_angles) else 0.0
                cur = backend.get_current_ik_deg_from_apos(joint_idx)
                err = None if cur is None else abs(cur - tgt)
                row_values = (
                    f"J{joint_idx}",
                    f"{tgt:7.2f}°",
                    "---" if cur is None else f"{cur:7.2f}°",
                    "---" if err is None else f"{err:6.2f}°",
                )
                if self._last_table_values.get(joint_idx) != row_values:
                    self._last_table_values[joint_idx] = row_values
                    for col, text in enumerate(row_values):
                        item = self.joint_table.item(row, col)
                        if item is None:
                            item = QTableWidgetItem(text)
                            item.setTextAlignment(Qt.AlignCenter)
                            self.joint_table.setItem(row, col, item)
                        elif item.text() != text:
                            item.setText(text)

            status_snapshot = backend.get_connection_status_text() + "\n" + backend.last_telemetry_line
            if status_snapshot != self._last_status_snapshot:
                self._append_log("Status updated.")
                self._last_status_snapshot = status_snapshot

            if draw_canvas:
                self.canvas.draw_arm()
        finally:
            self._building_controls = False

    def closeEvent(self, event):
        try:
            if self._backend_connected():
                backend.disconnect_serial()
        finally:
            super().closeEvent(event)


def main():
    app = QApplication(sys.argv)
    window = DashboardWindow()
    window.show()
    sys.exit(app.exec())


if __name__ == "__main__":
    main()
