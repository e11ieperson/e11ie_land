import numpy as np
import math
import matplotlib.pyplot as plt
from matplotlib.widgets import Slider, Button, CheckButtons
import serial
import serial.tools.list_ports
import time
import threading
import sys
import traceback
import tempfile
import os
import json
import re
import queue

CONFIG_FILE = "robot_config.json"
robot_config = {}
wrist_diff_config = {}
controller_config = {}

try:
    from ikpy.chain import Chain
except ImportError:
    print("Error: ikpy or its dependencies not found.")
    print("Please install requirements by running:")
    print("pip install ikpy pyserial matplotlib numpy scipy")
    sys.exit(1)

try:
    from pynput import keyboard
except ImportError:
    print("Error: pynput library not found.")
    print("Please install it running: pip install pynput")
    sys.exit(1)

# --- Configuration ---
BAUD_RATE = 115200
URDF_FILE = "URDF/URDF.urdf"

# Disable default matplotlib key bindings to prevent interference with WASD Controls
for key in list(plt.rcParams.keys()):
    if key.startswith('keymap.'):
        plt.rcParams[key] = []

# --- Globals ---
ser = None
read_thread = None
tx_thread = None
wrist_diff_thread = None
serial_running = True
cmd_queue = queue.Queue()
controller_ports = {"base": None, "joint4": None, "wrist": None}
controller_serial = {"base": None, "joint4": None, "wrist": None}
controller_read_threads = {"base": None, "joint4": None, "wrist": None}
controller_tx_threads = {"base": None, "joint4": None, "wrist": None}
controller_cmd_queues = {"base": queue.Queue(), "joint4": queue.Queue(), "wrist": queue.Queue()}
controller_last_telemetry_line = {"base": "", "joint4": "", "wrist": ""}
expected_controller_names = set()
my_chain = None
current_target_xyz = [0.2, 0.0, 0.3]  # Default safe position
current_target_rpy = [0.0, 0.0, 0.0]  # Roll, Pitch, Yaw
current_joint_angles = [0.0]*6
actual_joint_angles = None
is_previewing = False
preview_start_time = 0
PREVIEW_DURATION = 2.5 # seconds for preview animation
live_update = False
sync_model_to_telemetry = True
last_send_time = 0

PLANAR_MODE = True
PLANAR_ACTIVE_MOTORS = {2, 3}
LOCKED_MOTORS = {1, 4, 5, 6}
LOCKED_BOUNDS_EPS_RAD = 1e-4
DEFAULT_IK_ZERO_OFFSETS = {3: -90.0}
locked_joint_rads = {mid: 0.0 for mid in range(1, 7)}
planar_target_y = 0.0

cam_elev = 20.0
cam_azim = -60.0
DISPLAY_MIRROR_X = True
arm_line = None
ghost_line = None
target_point = None
joint_texts = []
arm_line_top = None
ghost_line_top = None
target_point_top = None
arm_line_side = None
ghost_line_side = None
target_point_side = None
ax_top = None
ax_side = None

txt_joint_dashboard = None
btn_connection = None
txt_connection_status = None

# --- Safety Controls ---
FULL_RANGE_EPS = 359.5
MAX_COMMAND_STEP_DEG = 6.0
MAX_COMMAND_RATE_DEG_PER_SEC = 120.0
COMMAND_DEADBAND_DEG = 0.20
RAMP_INTERVAL_SEC = 0.03
RECENT_COMMAND_HOLD_SEC = 0.35

last_sent_logical_deg = {i: None for i in range(1, 7)}
latest_motor_pos_deg = {}
latest_motor_apos_deg = {}
latest_motor_telemetry_time = {}
last_command_wall_time = 0.0
last_serial_command = ""
last_serial_command_time = 0.0
serial_command_count = 0
last_telemetry_line = ""
wrist_diff_target_logical_deg = {"pitch": None, "roll": None}
wrist_diff_pid_state = {
    "pitch": {"integral": 0.0, "prev_error": 0.0},
    "roll": {"integral": 0.0, "prev_error": 0.0},
}
wrist_diff_last_update_time = 0.0
wrist_diff_last_motor_cmd = (None, None)
hand_pwm_command = 0
abspid_skip_state = {}

TELEMETRY_STALE_SEC = 1.0
AUTO_MOVE_TO_ZERO_ON_CONNECT = True
AUTO_MOVE_TO_ZERO_WAIT_SEC = 1.5
CONTROLLER_MOTOR_GROUPS = {
    "base": {1, 2, 3},
    "joint4": {4},
    "wrist": {5, 6},
}
BROADCAST_COMMANDS = {"S", "TON", "TOFF", "T"}


def _safe_float(v):
    try:
        return float(v)
    except Exception:
        return None


def _get_zero_offset(mid):
    try:
        return float(robot_config.get(str(mid), {}).get("zero_offset", 0.0))
    except Exception:
        return 0.0


def _get_controller_defaults():
    return {
        "base": {"port": None, "motors": [1, 2, 3]},
        "joint4": {"port": None, "motors": [4]},
        "wrist": {"port": None, "motors": [5, 6]},
    }


def _normalize_controller_config(raw_cfg):
    merged = _get_controller_defaults()
    if not isinstance(raw_cfg, dict):
        return merged
    for name in ("base", "joint4", "wrist"):
        cfg = raw_cfg.get(name, {}) if isinstance(raw_cfg.get(name, {}), dict) else {}
        if cfg.get("port") not in (None, ""):
            merged[name]["port"] = str(cfg.get("port"))
        motors = cfg.get("motors")
        if isinstance(motors, list):
            out = []
            for mid in motors:
                try:
                    out.append(int(mid))
                except Exception:
                    pass
            if out:
                merged[name]["motors"] = out
    return merged


def _controller_for_motor(mid):
    try:
        motor_id = int(mid)
    except Exception:
        return "base"
    active_groups = CONTROLLER_MOTOR_GROUPS
    if controller_config:
        active_groups = {
            name: set(cfg.get("motors", []))
            for name, cfg in controller_config.items()
        }
    for controller_name, motor_ids in active_groups.items():
        if motor_id in motor_ids:
            return controller_name
    return "base"


def _controller_owns_motor(controller_name, motor_id):
    cfg = controller_config.get(controller_name, {}) if isinstance(controller_config.get(controller_name, {}), dict) else {}
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


def _connected_controller_names():
    return [name for name, conn in controller_serial.items() if conn and conn.is_open]


def _required_controller_names():
    if expected_controller_names:
        return set(expected_controller_names)
    return set(_connected_controller_names())


def all_required_controllers_healthy(require_fresh=True):
    required = _required_controller_names()
    if not required:
        return False
    now = time.time()
    for name in sorted(required):
        conn = controller_serial.get(name)
        if not (conn and conn.is_open):
            return False
        if require_fresh:
            fresh = False
            for mid in CONTROLLER_MOTOR_GROUPS.get(name, set()):
                ts = latest_motor_telemetry_time.get(int(mid))
                if ts is None:
                    continue
                try:
                    if (now - float(ts)) <= TELEMETRY_STALE_SEC:
                        fresh = True
                        break
                except Exception:
                    continue
            if not fresh:
                return False
    return True


def _get_primary_serial():
    for name in ("base", "joint4", "wrist"):
        conn = controller_serial.get(name)
        if conn and conn.is_open:
            return conn
    return None


def _sync_legacy_serial_refs():
    global ser, read_thread, tx_thread, cmd_queue, last_telemetry_line
    ser = _get_primary_serial()
    read_thread = controller_read_threads.get("base") or controller_read_threads.get("joint4") or controller_read_threads.get("wrist")
    tx_thread = controller_tx_threads.get("base") or controller_tx_threads.get("joint4") or controller_tx_threads.get("wrist")
    cmd_queue = controller_cmd_queues["base"]
    last_telemetry_line = controller_last_telemetry_line.get("base") or controller_last_telemetry_line.get("joint4") or controller_last_telemetry_line.get("wrist") or ""


def _connected_controller_count():
    return len(_connected_controller_names())


def is_serial_connected():
    return _connected_controller_count() > 0


def clear_abspid_skip_state():
    abspid_skip_state.clear()


def get_abspid_skip_state():
    return dict(abspid_skip_state)


def _parse_command_segments(cmd):
    return [part.strip() for part in str(cmd).split(";") if part.strip()]


def _route_segment(segment):
    toks = segment.split()
    if not toks:
        return set()
    op = toks[0].upper()
    if op in BROADCAST_COMMANDS:
        return set(CONTROLLER_MOTOR_GROUPS.keys())
    if op in {"P", "PR", "M", "MV", "KP", "KI", "KD", "AOL", "SKP", "SKI", "SKD", "SOL", "ABSPID", "CAL", "JLIM", "GEAR"}:
        if len(toks) >= 2:
            return {_controller_for_motor(toks[1])}
    if op == "ENCMAP":
        if len(toks) >= 3:
            target_motor = toks[2]
            if str(target_motor) == "0":
                return set(CONTROLLER_MOTOR_GROUPS.keys())
            return {_controller_for_motor(target_motor)}
        return set(CONTROLLER_MOTOR_GROUPS.keys())
    if op == "ENCREVSET":
        if len(toks) >= 4:
            return {_controller_for_motor(toks[3])}
        return set(CONTROLLER_MOTOR_GROUPS.keys())
    if op in {"HAND", "HOPEN", "HCLOSE", "HSTOP"}:
        return {"wrist"}
    return set(CONTROLLER_MOTOR_GROUPS.keys())


def _route_command_packets(cmd):
    routed = {name: [] for name in CONTROLLER_MOTOR_GROUPS}
    for segment in _parse_command_segments(cmd):
        destinations = _route_segment(segment)
        for controller_name in destinations:
            routed[controller_name].append(segment)
    return {name: "; ".join(parts) for name, parts in routed.items() if parts}


def _clear_pending_commands(controller_name=None):
    queue_names = [controller_name] if controller_name else list(controller_cmd_queues.keys())
    for name in queue_names:
        q = controller_cmd_queues[name]
        while not q.empty():
            try:
                q.get_nowait()
                q.task_done()
            except Exception:
                break


def _clear_controller_owned_telemetry(controller_name):
    for mid in list(CONTROLLER_MOTOR_GROUPS.get(controller_name, set())):
        latest_motor_pos_deg.pop(int(mid), None)
        latest_motor_apos_deg.pop(int(mid), None)
        latest_motor_telemetry_time.pop(int(mid), None)


def _handle_controller_disconnect(controller_name, reason=""):
    conn = controller_serial.get(controller_name)
    if conn and conn.is_open:
        try:
            conn.close()
        except Exception:
            pass
    controller_serial[controller_name] = None
    controller_ports[controller_name] = None
    controller_read_threads[controller_name] = None
    controller_tx_threads[controller_name] = None
    controller_last_telemetry_line[controller_name] = ""
    _clear_pending_commands(controller_name)
    _clear_controller_owned_telemetry(controller_name)
    _sync_legacy_serial_refs()
    update_connection_button()
    detail = f" ({reason})" if reason else ""
    print(f"Controller {controller_name} disconnected{detail}.")


def _reset_command_tracking_state():
    global last_command_wall_time
    last_command_wall_time = 0.0
    for mid in last_sent_logical_deg:
        last_sent_logical_deg[mid] = None


def reset_runtime_motion_state():
    clear_wrist_diff_targets(send_stop=False)
    _reset_command_tracking_state()
    _clear_pending_commands()


def set_hand_pwm(pwm_value):
    global hand_pwm_command
    try:
        pwm = int(round(float(pwm_value)))
    except Exception:
        pwm = 0
    pwm = max(-255, min(255, pwm))
    hand_pwm_command = pwm
    send_command(f"HAND {pwm}")
    print(f"Hand PWM command: {pwm}")
    return pwm


def open_hand(pwm_value=255):
    return set_hand_pwm(abs(pwm_value))


def close_hand(pwm_value=255):
    return set_hand_pwm(-abs(pwm_value))


def stop_hand():
    return set_hand_pwm(0)


def find_serial_ports():
    ports = list(serial.tools.list_ports.comports())
    if not ports:
        return []
    preferred = []
    fallback = []
    for port in ports:
        desc = f"{port.description} {port.device}".lower()
        if any(token in desc for token in ("ch340", "arduino", "serial", "usb")):
            preferred.append(port.device)
        else:
            fallback.append(port.device)
    ordered = preferred + fallback
    seen = []
    for device in ordered:
        if device not in seen:
            seen.append(device)
    return seen


def _ordered_controller_ports():
    available = find_serial_ports()
    ordered = []
    used = set()
    for name in ("base", "joint4", "wrist"):
        preferred = None
        if isinstance(controller_config.get(name, {}), dict):
            preferred = controller_config[name].get("port")
        if preferred and preferred in available and preferred not in used:
            ordered.append((name, preferred))
            used.add(preferred)
    remaining = [port for port in available if port not in used]
    for name in ("base", "joint4", "wrist"):
        if any(existing_name == name for existing_name, _ in ordered):
            continue
        if not remaining:
            break
        ordered.append((name, remaining.pop(0)))
    return ordered


def _get_wrist_diff_defaults():
    return {
        "enabled": False,
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
            "motor_b_roll_sign": -1.0,
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
            "integral_limit": 30.0,
        },
    }


def get_wrist_diff_config():
    cfg = _get_wrist_diff_defaults()
    if not isinstance(wrist_diff_config, dict):
        return cfg
    cfg.update({k: v for k, v in wrist_diff_config.items() if k not in {"mix", "pid"}})
    mix = dict(cfg["mix"])
    if isinstance(wrist_diff_config.get("mix"), dict):
        mix.update(wrist_diff_config.get("mix", {}))
    cfg["mix"] = mix
    pid = dict(cfg["pid"])
    if isinstance(wrist_diff_config.get("pid"), dict):
        pid.update(wrist_diff_config.get("pid", {}))
    cfg["pid"] = pid
    return cfg


def wrist_diff_enabled():
    cfg = get_wrist_diff_config()
    return bool(cfg.get("enabled", False))


def wrist_diff_joint_ids():
    cfg = get_wrist_diff_config()
    return int(cfg.get("pitch_joint_id", 5)), int(cfg.get("roll_joint_id", 6))


def wrist_diff_motor_ids():
    cfg = get_wrist_diff_config()
    return int(cfg.get("motor_a_id", 5)), int(cfg.get("motor_b_id", 6))


def wrist_diff_encoder_motor_id(axis_name):
    cfg = get_wrist_diff_config()
    if axis_name == "pitch":
        return int(cfg.get("pitch_encoder_motor_id", 5))
    return int(cfg.get("roll_encoder_motor_id", 6))


def wrist_diff_joint_id_for_axis(axis_name):
    pitch_joint_id, roll_joint_id = wrist_diff_joint_ids()
    return pitch_joint_id if axis_name == "pitch" else roll_joint_id


def reset_wrist_diff_pid_state():
    global wrist_diff_last_update_time, wrist_diff_last_motor_cmd
    for axis_name in ("pitch", "roll"):
        wrist_diff_pid_state[axis_name]["integral"] = 0.0
        wrist_diff_pid_state[axis_name]["prev_error"] = 0.0
    wrist_diff_last_update_time = 0.0
    wrist_diff_last_motor_cmd = (None, None)


def clear_wrist_diff_targets(send_stop=False):
    wrist_diff_target_logical_deg["pitch"] = None
    wrist_diff_target_logical_deg["roll"] = None
    reset_wrist_diff_pid_state()
    if send_stop and is_serial_connected() and wrist_diff_enabled():
        motor_a_id, motor_b_id = wrist_diff_motor_ids()
        send_command(f"M {motor_a_id} 0; M {motor_b_id} 0")


def set_wrist_diff_targets(pitch_target_deg=None, roll_target_deg=None):
    if pitch_target_deg is not None:
        wrist_diff_target_logical_deg["pitch"] = float(pitch_target_deg)
    if roll_target_deg is not None:
        wrist_diff_target_logical_deg["roll"] = float(roll_target_deg)
    reset_wrist_diff_pid_state()


def get_wrist_motor_logical_deg():
    cfg = get_wrist_diff_config()
    motor_a_id = int(cfg.get("motor_a_id", 5))
    motor_b_id = int(cfg.get("motor_b_id", 6))
    motor_a_apos = latest_motor_apos_deg.get(motor_a_id)
    motor_b_apos = latest_motor_apos_deg.get(motor_b_id)
    if motor_a_apos is None or motor_b_apos is None:
        return None, None
    motor_a_logical = float(motor_a_apos) - _get_abs_zero_offset(motor_a_id)
    motor_b_logical = float(motor_b_apos) - _get_abs_zero_offset(motor_b_id)
    return motor_a_logical, motor_b_logical


def get_wrist_diff_actual_axes_deg():
    cfg = get_wrist_diff_config()
    mix = cfg.get("mix", {})
    motor_a_logical, motor_b_logical = get_wrist_motor_logical_deg()
    if motor_a_logical is None or motor_b_logical is None:
        return None, None

    a11 = float(mix.get("motor_a_pitch_sign", 1.0))
    a12 = float(mix.get("motor_a_roll_sign", 1.0))
    a21 = float(mix.get("motor_b_pitch_sign", 1.0))
    a22 = float(mix.get("motor_b_roll_sign", -1.0))
    det = (a11 * a22) - (a12 * a21)
    if abs(det) < 1e-6:
        return None, None

    pitch_actual = ((motor_a_logical * a22) - (a12 * motor_b_logical)) / det
    roll_actual = ((a11 * motor_b_logical) - (motor_a_logical * a21)) / det
    return float(pitch_actual), float(roll_actual)


def get_wrist_output_logical_deg(axis_name):
    pitch_actual, roll_actual = get_wrist_diff_actual_axes_deg()
    if axis_name == "pitch":
        return pitch_actual
    return roll_actual


def _get_abs_zero_offset(mid):
    try:
        return float(robot_config.get(str(mid), {}).get("abs_zero_offset", 0.0))
    except Exception:
        return 0.0


def _get_ik_zero_offset(mid):
    try:
        cfg = robot_config.get(str(mid), {})
        if "ik_zero_offset" in cfg:
            return float(cfg.get("ik_zero_offset", 0.0))
        if int(mid) in DEFAULT_IK_ZERO_OFFSETS:
            return float(DEFAULT_IK_ZERO_OFFSETS[int(mid)])
        return float(cfg.get("zero_offset", 0.0))
    except Exception:
        return 0.0


def get_current_logical_deg_from_apos(mid):
    if wrist_diff_enabled():
        pitch_joint_id, roll_joint_id = wrist_diff_joint_ids()
        if int(mid) == pitch_joint_id:
            return get_wrist_output_logical_deg("pitch")
        if int(mid) == roll_joint_id:
            return get_wrist_output_logical_deg("roll")
    apos = latest_motor_apos_deg.get(int(mid))
    if apos is None:
        return None
    return apos - _get_abs_zero_offset(mid)


def get_current_ik_deg_from_apos(mid):
    logical_deg = get_current_logical_deg_from_apos(mid)
    if logical_deg is None:
        return None
    zoff = _get_ik_zero_offset(mid)
    return logical_deg - zoff


def clampf(v, lo, hi):
    return max(lo, min(hi, v))


def wrap360f(angle):
    while angle >= 360.0:
        angle -= 360.0
    while angle < 0.0:
        angle += 360.0
    return angle


def nearest_equivalent(angle_deg, reference_deg):
    turns = round((reference_deg - angle_deg) / 360.0)
    return angle_deg + (turns * 360.0)


def is_full_range(min_a, max_a):
    return (max_a - min_a) >= FULL_RANGE_EPS


def normalize_target_for_limits(target_deg, min_a, max_a, reference_deg=None):
    if is_full_range(min_a, max_a):
        return wrap360f(target_deg)
    if min_a <= target_deg <= max_a:
        return target_deg
    if reference_deg is None:
        reference_deg = (min_a + max_a) * 0.5
    else:
        reference_deg = clampf(reference_deg, min_a, max_a)

    # For limited joints, do not "wrap then clamp" (that can pin to a hard stop).
    # Instead, choose a 360°-equivalent target that already lies inside bounds.
    candidates = []
    min_turns = int(math.floor((min_a - target_deg) / 360.0)) - 1
    max_turns = int(math.ceil((max_a - target_deg) / 360.0)) + 1
    for turns in range(min_turns, max_turns + 1):
        cand = target_deg + (360.0 * turns)
        if min_a <= cand <= max_a:
            candidates.append(cand)

    if candidates:
        return min(candidates, key=lambda cand: abs(cand - reference_deg))

    return clampf(target_deg, min_a, max_a)


def shortest_delta_deg(current_deg, target_deg):
    delta = target_deg - current_deg
    if delta > 180.0:
        delta -= 360.0
    if delta < -180.0:
        delta += 360.0
    return delta


def get_motor_limits(mid):
    cfg = robot_config.get(str(mid), {})
    min_a = float(cfg.get("min_angle", 0.0))
    max_a = float(cfg.get("max_angle", 360.0))
    if max_a <= min_a:
        min_a, max_a = 0.0, 360.0
    return min_a, max_a


def get_start_command_position(mid, target_deg):
    prev = last_sent_logical_deg.get(mid)
    if prev is not None and (time.time() - last_command_wall_time) < RECENT_COMMAND_HOLD_SEC:
        return prev
    logical_now = get_current_logical_deg_from_apos(mid)
    if logical_now is not None:
        return logical_now
    tele = latest_motor_pos_deg.get(mid)
    if tele is not None:
        return tele
    if prev is not None:
        return prev
    return target_deg


def command_step_toward(mid, current_deg, target_deg, max_step_deg):
    min_a, max_a = get_motor_limits(mid)
    if is_full_range(min_a, max_a):
        cur = wrap360f(current_deg)
        tgt = wrap360f(target_deg)
        delta = shortest_delta_deg(cur, tgt)
        if abs(delta) <= max_step_deg:
            return tgt, True
        return wrap360f(cur + math.copysign(max_step_deg, delta)), False

    cur = normalize_target_for_limits(current_deg, min_a, max_a, reference_deg=(min_a + max_a) * 0.5)
    tgt = normalize_target_for_limits(target_deg, min_a, max_a, reference_deg=cur)
    delta = tgt - cur
    if abs(delta) <= max_step_deg:
        return tgt, True
    return cur + math.copysign(max_step_deg, delta), False


def command_distance_deg(mid, a_deg, b_deg):
    min_a, max_a = get_motor_limits(mid)
    if is_full_range(min_a, max_a):
        return abs(shortest_delta_deg(wrap360f(a_deg), wrap360f(b_deg)))
    a_lim = normalize_target_for_limits(a_deg, min_a, max_a, reference_deg=(min_a + max_a) * 0.5)
    b_lim = normalize_target_for_limits(b_deg, min_a, max_a, reference_deg=a_lim)
    return abs(b_lim - a_lim)


def load_robot_config():
    global robot_config, wrist_diff_config, controller_config
    robot_config = {}
    wrist_diff_config = {}
    controller_config = _get_controller_defaults()
    if not os.path.exists(CONFIG_FILE):
        return
    try:
        with open(CONFIG_FILE, "r") as f:
            data = json.load(f)
        motors = data.get("motors", {}) if isinstance(data, dict) else {}
        if isinstance(motors, dict):
            robot_config = motors
        diff_cfg = data.get("wrist_differential", {}) if isinstance(data, dict) else {}
        if isinstance(diff_cfg, dict):
            wrist_diff_config = diff_cfg
        controller_config = _normalize_controller_config(data.get("controllers", {}))
    except Exception as e:
        print(f"Warning: Failed to load {CONFIG_FILE}: {e}")


def get_expected_runtime_apply_plan():
    desired_enc_map_by_controller = {name: {} for name in CONTROLLER_MOTOR_GROUPS.keys()}
    for mid in sorted(robot_config.keys(), key=lambda x: int(x)):
        cfg = robot_config.get(str(mid), {})
        enc_idx = cfg.get("abs_encoder_index")
        if enc_idx is None:
            continue
        try:
            mid_i = int(mid)
            enc_i = int(enc_idx)
            controller_name = _controller_for_motor(mid_i)
            desired_enc_map_by_controller.setdefault(controller_name, {})[enc_i] = mid_i
        except Exception:
            pass

    wrist_conn = controller_serial.get("wrist")
    wrist_online = bool(wrist_conn and wrist_conn.is_open)
    if wrist_diff_enabled() and wrist_online:
        diff_cfg = get_wrist_diff_config()
        for enc_key, motor_key in (("pitch_encoder_index", "pitch_encoder_motor_id"),
                                   ("roll_encoder_index", "roll_encoder_motor_id")):
            enc_idx = diff_cfg.get(enc_key)
            motor_id = diff_cfg.get(motor_key)
            try:
                if enc_idx is not None:
                    motor_i = int(motor_id)
                    enc_i = int(enc_idx)
                    controller_name = _controller_for_motor(motor_i)
                    desired_enc_map_by_controller.setdefault(controller_name, {})[enc_i] = motor_i
            except Exception:
                pass

    wrist_joint_set = set(wrist_diff_joint_ids()) if wrist_diff_enabled() else set()
    abspid_enable_by_motor = {}
    skipped_abs_pid = []
    for mid in sorted(robot_config.keys(), key=lambda x: int(x)):
        cfg = robot_config.get(str(mid), {})
        enable = 1 if bool(cfg.get("abs_pid_enabled", False)) else 0
        if cfg.get("abs_encoder_index") is None:
            if enable == 1:
                skipped_abs_pid.append(int(mid))
            enable = 0
        if wrist_diff_enabled() and wrist_online and int(mid) in wrist_joint_set:
            enable = 0
        abspid_enable_by_motor[int(mid)] = int(enable)

    return {
        "connected_controllers": _connected_controller_names(),
        "wrist_online": wrist_online,
        "wrist_diff_enabled": bool(wrist_diff_enabled()),
        "enc_map_by_controller": desired_enc_map_by_controller,
        "abspid_enable_by_motor": abspid_enable_by_motor,
        "abspid_skipped_unmapped": sorted(set(skipped_abs_pid)),
    }


def apply_robot_config_to_arm():
    _sync_legacy_serial_refs()
    if not is_serial_connected():
        print("Config apply skipped: serial not connected.")
        return False
    if not all_required_controllers_healthy(require_fresh=False):
        print("Config apply aborted: one or more required controllers are offline.")
        return False
    if not robot_config:
        print("Config apply skipped: robot_config is empty.")
        return False

    clear_abspid_skip_state()

    # Safe bring-up first to avoid stale runtime state on reconnect.
    send_command("S; TON")
    time.sleep(0.05)

    for mid in sorted(robot_config.keys(), key=lambda x: int(x)):
        if not all_required_controllers_healthy(require_fresh=False):
            print("Config apply aborted: controller dropped during gain/limit apply.")
            return False
        cfg = robot_config.get(str(mid), {})
        try:
            gr = float(cfg.get("gear_ratio", 1.0))
            mn = float(cfg.get("min_angle", 0.0))
            mx = float(cfg.get("max_angle", 360.0))
            kp = float(cfg.get("kp", 5.0))
            ki = float(cfg.get("ki", 1.0))
            kd = float(cfg.get("kd", 0.0))
            aol = float(cfg.get("aol", 500.0))
            skp = float(cfg.get("skp", 0.3))
            ski = float(cfg.get("ski", 0.0))
            skd = float(cfg.get("skd", 0.0))
            sol = float(cfg.get("sol", 20.0))
        except Exception as e:
            print(f"Config apply skipped for motor {mid}: {e}")
            continue

        batch = (
            f"GEAR {mid} {gr:.6f}"
            f"; JLIM {mid} {mn:.1f} {mx:.1f}"
            f"; KP {mid} {kp:.4f}; KI {mid} {ki:.4f}; KD {mid} {kd:.4f}; AOL {mid} {aol:.2f}"
            f"; SKP {mid} {skp:.4f}; SKI {mid} {ski:.4f}; SKD {mid} {skd:.4f}; SOL {mid} {sol:.2f}"
        )
        send_command(batch)
        time.sleep(0.05)

    desired_enc_map_by_controller = {name: {} for name in CONTROLLER_MOTOR_GROUPS.keys()}
    for mid in sorted(robot_config.keys(), key=lambda x: int(x)):
        if not all_required_controllers_healthy(require_fresh=False):
            print("Config apply aborted: controller dropped during ENCREVSET apply.")
            return False
        cfg = robot_config.get(str(mid), {})
        enc_idx = cfg.get("abs_encoder_index")
        if enc_idx is None:
            continue
        try:
            mid_i = int(mid)
            enc_i = int(enc_idx)
            controller_name = _controller_for_motor(mid_i)
            desired_enc_map_by_controller.setdefault(controller_name, {})[enc_i] = mid_i
        except Exception:
            pass

    wrist_conn = controller_serial.get("wrist")
    wrist_online = bool(wrist_conn and wrist_conn.is_open)
    if wrist_diff_enabled() and wrist_online:
        diff_cfg = get_wrist_diff_config()
        for enc_key, motor_key in (("pitch_encoder_index", "pitch_encoder_motor_id"),
                                   ("roll_encoder_index", "roll_encoder_motor_id")):
            enc_idx = diff_cfg.get(enc_key)
            motor_id = diff_cfg.get(motor_key)
            try:
                if enc_idx is not None:
                    motor_i = int(motor_id)
                    enc_i = int(enc_idx)
                    controller_name = _controller_for_motor(motor_i)
                    desired_enc_map_by_controller.setdefault(controller_name, {})[enc_i] = motor_i
            except Exception:
                pass
    elif wrist_diff_enabled() and not wrist_online:
        print("Wrist differential is enabled but wrist controller is offline; skipping wrist ENCMAP overrides.")

    # Always reset all encoder slots so stale mappings from a previous power cycle
    # cannot survive reconnect and break ABSPID bring-up.
    for enc_idx in range(8):
        if not all_required_controllers_healthy(require_fresh=False):
            print("Config apply aborted: controller dropped during ENCMAP reset.")
            return False
        send_command(f"ENCMAP {enc_idx} 0")
        time.sleep(0.01)

    for controller_name in sorted(desired_enc_map_by_controller.keys()):
        per_ctrl_map = desired_enc_map_by_controller.get(controller_name, {})
        for enc_idx in sorted(per_ctrl_map.keys()):
            if not all_required_controllers_healthy(require_fresh=False):
                print("Config apply aborted: controller dropped during ENCMAP apply.")
                return False
            send_command(f"ENCMAP {enc_idx} {per_ctrl_map[enc_idx]}")
            time.sleep(0.02)

    for mid in sorted(robot_config.keys(), key=lambda x: int(x)):
        cfg = robot_config.get(str(mid), {})
        enc_idx = cfg.get("abs_encoder_index")
        if enc_idx is None:
            continue
        try:
            enc_i = int(enc_idx)
            rev_i = 1 if bool(cfg.get("abs_encoder_reversed", True)) else 0
            send_command(f"ENCREVSET {enc_i} {rev_i} {int(mid)}")
            time.sleep(0.02)
        except Exception:
            pass

    # Firmware marks abs encoders as "detected" asynchronously in updateAbsEncoders().
    # Give that loop a short settle window after remapping before enabling ABSPID.
    time.sleep(0.20)

    abspid_packets = []
    wrist_joint_set = set(wrist_diff_joint_ids()) if wrist_diff_enabled() else set()
    skipped_abs_pid = []
    for mid in sorted(robot_config.keys(), key=lambda x: int(x)):
        if not all_required_controllers_healthy(require_fresh=False):
            print("Config apply aborted: controller dropped during ABSPID apply.")
            return False
        cfg = robot_config.get(str(mid), {})
        enable = 1 if bool(cfg.get("abs_pid_enabled", False)) else 0
        if cfg.get("abs_encoder_index") is None:
            if enable == 1:
                skipped_abs_pid.append(int(mid))
            enable = 0
        if wrist_diff_enabled() and wrist_online and int(mid) in wrist_joint_set:
            enable = 0
        abspid_packets.append((int(mid), int(enable)))
        send_command(f"ABSPID {mid} {enable}")
        time.sleep(0.02)

    # Retry enabled ABSPID after remap/detection settle to handle
    # "ABSPID skipped: encoder not detected yet" timing races on reconnect.
    for _ in range(3):
        time.sleep(0.10)
        for mid, enable in abspid_packets:
            if not all_required_controllers_healthy(require_fresh=False):
                print("Config apply aborted: controller dropped during ABSPID retry.")
                return False
            if enable == 1:
                send_command(f"ABSPID {mid} 1")
                time.sleep(0.01)

    if skipped_abs_pid:
        print(
            "ABSPID disabled for unmapped motors: "
            + ", ".join(f"M{m}" for m in sorted(set(skipped_abs_pid)))
        )

    if not all_required_controllers_healthy(require_fresh=False):
        print("Config apply failed: required controller offline at completion.")
        return False
    print("Applied robot_config.json settings to arm controller.")
    return True


def save_robot_config():
    try:
        existing = {}
        if os.path.exists(CONFIG_FILE):
            try:
                with open(CONFIG_FILE, "r") as f:
                    loaded = json.load(f)
                    if isinstance(loaded, dict):
                        existing = loaded
            except Exception:
                existing = {}
        payload = dict(existing)
        payload["controllers"] = _normalize_controller_config(controller_config)
        payload["motors"] = robot_config
        payload["wrist_differential"] = wrist_diff_config
        with open(CONFIG_FILE, "w") as f:
            json.dump(payload, f, indent=2)
    except Exception as e:
        print(f"Warning: Failed to save {CONFIG_FILE}: {e}")


def get_current_ik_rad_from_telemetry(mid):
    cur_deg = get_current_ik_deg_from_apos(mid)
    if cur_deg is None:
        tele = latest_motor_pos_deg.get(int(mid))
        if tele is None:
            return None
        cur_deg = tele - _get_ik_zero_offset(mid)
    return math.radians(cur_deg)


def get_current_real_joint_angles():
    if my_chain is None:
        return None
    n = len(my_chain.links)
    vals = [0.0] * n
    have_any = False
    for j in range(1, min(7, n)):
        ang = get_current_ik_rad_from_telemetry(j)
        if ang is None:
            if actual_joint_angles is not None and j < len(actual_joint_angles):
                vals[j] = actual_joint_angles[j]
            elif j < len(current_joint_angles):
                vals[j] = current_joint_angles[j]
        else:
            vals[j] = ang
            have_any = True
    if not have_any:
        return None
    vals = clamp_seed_angles_to_bounds(vals)
    vals = enforce_locked_joints(vals)
    return vals


def update_wrist_diff_controller_once():
    global wrist_diff_last_update_time, wrist_diff_last_motor_cmd
    if not wrist_diff_enabled():
        return False
    if not is_serial_connected():
        return False
    pitch_target = wrist_diff_target_logical_deg.get("pitch")
    roll_target = wrist_diff_target_logical_deg.get("roll")
    if pitch_target is None and roll_target is None:
        return False
    pitch_actual, roll_actual = get_wrist_diff_actual_axes_deg()
    if pitch_actual is None or roll_actual is None:
        return False

    cfg = get_wrist_diff_config()
    pid_cfg = cfg.get("pid", {})
    now = time.time()
    if wrist_diff_last_update_time > 0.0:
        dt = clampf(now - wrist_diff_last_update_time, 0.01, 0.10)
    else:
        dt = 0.02
    wrist_diff_last_update_time = now

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
        state = wrist_diff_pid_state[axis_name]
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

    mix = cfg.get("mix", {})
    motor_a_id, motor_b_id = wrist_diff_motor_ids()
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

    prev_a, prev_b = wrist_diff_last_motor_cmd
    if prev_a is None or prev_b is None or abs(prev_a - motor_a_cmd) >= 0.5 or abs(prev_b - motor_b_cmd) >= 0.5:
        send_command(f"M {motor_a_id} {motor_a_cmd:.2f}; M {motor_b_id} {motor_b_cmd:.2f}")
        wrist_diff_last_motor_cmd = (motor_a_cmd, motor_b_cmd)
    return True


def wrist_diff_control_loop():
    while serial_running:
        try:
            update_wrist_diff_controller_once()
            time.sleep(0.02)
        except Exception:
            time.sleep(0.05)


def sync_solver_seed_from_telemetry(update_target=False, require_all_controllers=True):
    global current_joint_angles, actual_joint_angles, current_target_xyz
    if require_all_controllers and not all_required_controllers_healthy(require_fresh=True):
        return False
    tele_angles = get_current_real_joint_angles()
    if tele_angles is None:
        return False
    synced_angles = clamp_seed_angles_to_bounds(enforce_locked_joints(tele_angles))
    current_joint_angles = list(synced_angles)
    actual_joint_angles = list(synced_angles)
    refresh_locked_joint_rads(synced_angles)
    if update_target and my_chain is not None:
        sync_target_from_joint_angles(synced_angles)
    return True


def get_link_bounds_rad(link_index):
    if my_chain is None or link_index < 0 or link_index >= len(my_chain.links):
        return None
    link = my_chain.links[link_index]
    bounds = getattr(link, 'bounds', None)
    if not isinstance(bounds, (tuple, list)) or len(bounds) != 2:
        return None
    lo, hi = bounds
    try:
        lo = float(lo)
        hi = float(hi)
    except Exception:
        return None
    if not np.isfinite(lo) or not np.isfinite(hi) or hi < lo:
        return None
    return lo, hi


def clamp_seed_angles_to_bounds(seed_angles):
    if my_chain is None:
        return seed_angles
    n = len(my_chain.links)
    if seed_angles is None:
        seed = [0.0] * n
    else:
        seed = list(seed_angles)
        if len(seed) < n:
            seed.extend([0.0] * (n - len(seed)))
        elif len(seed) > n:
            seed = seed[:n]

    for i in range(n):
        bounds = get_link_bounds_rad(i)
        if bounds is None:
            continue
        lo, hi = bounds
        if seed[i] < lo:
            seed[i] = lo
        elif seed[i] > hi:
            seed[i] = hi
    return seed


def motor_to_chain_index(mid):
    if my_chain is None:
        return None
    idx = int(mid)
    return idx if 0 <= idx < len(my_chain.links) else None


def refresh_locked_joint_rads(source_angles=None):
    global locked_joint_rads
    if source_angles is None:
        source_angles = actual_joint_angles if actual_joint_angles is not None else current_joint_angles
    if source_angles is None:
        return
    for mid in LOCKED_MOTORS:
        chain_idx = motor_to_chain_index(mid)
        if chain_idx is None or chain_idx >= len(source_angles):
            continue
        try:
            locked_joint_rads[mid] = float(source_angles[chain_idx])
        except Exception:
            pass


def apply_chain_bounds_from_config():
    if my_chain is None:
        return
    for mid in range(1, 7):
        chain_idx = motor_to_chain_index(mid)
        if chain_idx is None:
            continue
        link = my_chain.links[chain_idx]
        if not hasattr(link, 'bounds'):
            continue
        cfg = robot_config.get(str(mid), {})
        mn = float(cfg.get("min_angle", 0.0))
        mx = float(cfg.get("max_angle", 360.0))
        zoff = _get_ik_zero_offset(mid)
        if PLANAR_MODE and mid in LOCKED_MOTORS:
            locked = float(locked_joint_rads.get(mid, 0.0))
            link.bounds = (locked - LOCKED_BOUNDS_EPS_RAD, locked + LOCKED_BOUNDS_EPS_RAD)
            continue
        lower = math.radians(mn - zoff)
        upper = math.radians(mx - zoff)
        if upper < lower:
            lower, upper = upper, lower
        link.bounds = (lower, upper)


def enforce_locked_joints(joint_angles):
    if joint_angles is None:
        return joint_angles
    angles = list(joint_angles)
    if PLANAR_MODE:
        for mid in LOCKED_MOTORS:
            chain_idx = motor_to_chain_index(mid)
            if chain_idx is None or chain_idx >= len(angles):
                continue
            angles[chain_idx] = float(locked_joint_rads.get(mid, angles[chain_idx]))
    return angles


def solve_ik_for_target(seed_angles=None):
    if my_chain is None:
        return None
    if seed_angles is None:
        seed_angles = current_joint_angles
    seed = clamp_seed_angles_to_bounds(enforce_locked_joints(seed_angles))
    apply_chain_bounds_from_config()
    target_xyz = list(current_target_xyz)
    if PLANAR_MODE:
        target_xyz[1] = planar_target_y
    solved = my_chain.inverse_kinematics(target_xyz, initial_position=seed)
    solved = clamp_seed_angles_to_bounds(enforce_locked_joints(solved))
    return solved


def sync_target_from_joint_angles(joint_angles):
    global current_target_xyz, planar_target_y
    if my_chain is None or joint_angles is None:
        return
    try:
        fk = my_chain.forward_kinematics(joint_angles)
        current_target_xyz = list(fk[:3, 3])
        if PLANAR_MODE:
            planar_target_y = float(current_target_xyz[1])
            current_target_xyz[1] = planar_target_y
    except Exception:
        pass


# --- Serial Functions ---
def find_serial_port():
    ports = find_serial_ports()
    return ports[0] if ports else None


def _connect_single_controller(controller_name, port_name):
    try:
        conn = serial.Serial(port_name, BAUD_RATE, timeout=0.1)
    except Exception as exc:
        msg = str(exc)
        if "SetCommState" in msg or "PermissionError" in msg:
            msg += " (port busy or invalid; ensure no other app has this COM port open)"
        raise RuntimeError(f"{controller_name}:{port_name} open failed: {msg}") from exc
    time.sleep(2)
    controller_ports[controller_name] = port_name
    controller_serial[controller_name] = conn
    reader = threading.Thread(target=read_from_port, args=(controller_name,), daemon=True)
    controller_read_threads[controller_name] = reader
    reader.start()
    tx = controller_tx_threads.get(controller_name)
    if tx is None or not tx.is_alive():
        tx = threading.Thread(target=command_tx_loop, args=(controller_name,), daemon=True)
        controller_tx_threads[controller_name] = tx
        tx.start()
    return conn

def connect_serial(perform_startup_actions=True):
    global wrist_diff_thread, last_command_wall_time, expected_controller_names
    if _connected_controller_count() > 0:
        if not expected_controller_names:
            expected_controller_names = set(_connected_controller_names())
        _sync_legacy_serial_refs()
        update_connection_button()
        return True
    print("Looking for controller serial ports...")
    assignments = _ordered_controller_ports()
    if not assignments:
        print("No serial ports found! Please plug in your device.")
        update_connection_button()
        return False

    connected = []
    failures = []
    try:
        for controller_name, port_name in assignments:
            print(f"Connecting {controller_name} controller to {port_name}...")
            try:
                _connect_single_controller(controller_name, port_name)
                connected.append((controller_name, port_name))
            except Exception as exc:
                failures.append((controller_name, port_name, str(exc)))

        if not connected:
            for controller_name, port_name, err in failures:
                print(f"Connect failed [{controller_name}:{port_name}] {err}")
            update_connection_button()
            return False

        expected_controller_names = {name for name, _ in connected}
        _sync_legacy_serial_refs()
        print(f"Connected controllers: {', '.join(f'{name}={controller_ports[name]}' for name, _ in connected)}")
        for controller_name, port_name, err in failures:
            print(f"Connect failed [{controller_name}:{port_name}] {err}")

        latest_motor_pos_deg.clear()
        latest_motor_apos_deg.clear()
        latest_motor_telemetry_time.clear()
        reset_runtime_motion_state()
        if wrist_diff_thread is None or not wrist_diff_thread.is_alive():
            wrist_diff_thread = threading.Thread(target=wrist_diff_control_loop, daemon=True)
            wrist_diff_thread.start()

        if perform_startup_actions:
            send_command("S; TON")
            load_robot_config()
            print("Loaded robot_config.json for IK limits/offset mapping.")
            apply_robot_config_to_arm()
            time.sleep(0.25)
            sync_solver_seed_from_telemetry(update_target=True)
            move_configured_joints_to_zero_on_connect()
            last_command_wall_time = time.time()
        print("Serial connected and telemetry enabled.")
        update_connection_button()
        return True
    except Exception as e:
        print(f"Failed to connect controllers: {e}")
        disconnect_serial()
        update_connection_button()
        return False

def disconnect_serial():
    global ser, read_thread, tx_thread, last_telemetry_line, hand_pwm_command, expected_controller_names
    clear_wrist_diff_targets(send_stop=True)
    send_command("HSTOP")
    hand_pwm_command = 0
    for controller_name, conn in list(controller_serial.items()):
        if conn and conn.is_open:
            try:
                conn.close()
            except Exception:
                pass
        controller_serial[controller_name] = None
        controller_ports[controller_name] = None
        controller_read_threads[controller_name] = None
        controller_tx_threads[controller_name] = None
        controller_last_telemetry_line[controller_name] = ""
    latest_motor_pos_deg.clear()
    latest_motor_apos_deg.clear()
    latest_motor_telemetry_time.clear()
    clear_abspid_skip_state()
    reset_runtime_motion_state()
    expected_controller_names = set()
    ser = None
    read_thread = None
    tx_thread = None
    last_telemetry_line = ""
    update_connection_button()
    return True


def update_connection_button():
    global btn_connection
    if btn_connection is None:
        return
    connected = _connected_controller_count() > 0
    tele_ok = has_fresh_telemetry()
    if connected and tele_ok:
        label = f'Connected ({_connected_controller_count()})'
        color = 'lightgreen'
    elif connected:
        label = f'Port Open ({_connected_controller_count()})'
        color = 'khaki'
    else:
        label = 'Disconnected'
        color = 'lightcoral'
    btn_connection.label.set_text(label)
    try:
        btn_connection.ax.set_facecolor(color)
    except Exception:
        pass


def has_fresh_telemetry():
    now = time.time()
    for ts in latest_motor_telemetry_time.values():
        try:
            if (now - float(ts)) <= TELEMETRY_STALE_SEC:
                return True
        except Exception:
            pass
    return False


def get_connection_status_text():
    connected = _connected_controller_count() > 0
    tele_ok = has_fresh_telemetry()
    now = time.time()
    if last_serial_command_time > 0:
        cmd_age = now - last_serial_command_time
        cmd_age_str = f"{cmd_age:4.2f}s ago"
    else:
        cmd_age_str = "never"
    if latest_motor_telemetry_time:
        newest = max(latest_motor_telemetry_time.values())
        tele_age_str = f"{(now - newest):4.2f}s ago"
    else:
        tele_age_str = "never"
    last_cmd = last_serial_command if last_serial_command else "<none>"
    if len(last_cmd) > 58:
        last_cmd = last_cmd[:55] + "..."
    port_parts = []
    for name in ("base", "joint4", "wrist"):
        port_name = controller_ports.get(name)
        if port_name:
            port_parts.append(f"{name}:{port_name}")
    return (
        "Link Status\n"
        f"Ports:     {', '.join(port_parts) if port_parts else 'CLOSED'}\n"
        f"Telemetry: {'LIVE' if tele_ok else 'STALE'} ({tele_age_str})\n"
        f"Hand PWM:  {hand_pwm_command}\n"
        f"Sent Cmds: {serial_command_count}\n"
        f"Last Send: {cmd_age_str}\n"
        f"Last Cmd:  {last_cmd}"
    )


def wait_for_telemetry(timeout_sec=AUTO_MOVE_TO_ZERO_WAIT_SEC):
    deadline = time.time() + max(0.0, float(timeout_sec))
    while time.time() < deadline:
        if has_fresh_telemetry():
            return True
        time.sleep(0.05)
    return has_fresh_telemetry()


def get_auto_zero_motor_ids():
    motor_ids = []
    pitch_joint_id, roll_joint_id = wrist_diff_joint_ids()
    for mid in sorted(robot_config.keys(), key=lambda x: int(x)):
        cfg = robot_config.get(str(mid), {})
        if wrist_diff_enabled() and int(mid) in {pitch_joint_id, roll_joint_id}:
            continue
        if cfg.get("abs_encoder_index") is None:
            continue
        if not bool(cfg.get("abs_pid_enabled", False)):
            continue
        motor_ids.append(int(mid))
    return motor_ids


def move_configured_joints_to_zero_on_connect():
    if not AUTO_MOVE_TO_ZERO_ON_CONNECT:
        return False
    if _connected_controller_count() == 0:
        return False
    motor_ids = get_auto_zero_motor_ids()
    wrist_active = wrist_diff_enabled()
    if not motor_ids and not wrist_active:
        print("Auto-zero move skipped: no configured abs-encoder joints.")
        return False
    if not wait_for_telemetry():
        print("Auto-zero move skipped: telemetry not ready.")
        return False

    did_queue = False

    if wrist_active:
        pitch_joint_id, roll_joint_id = wrist_diff_joint_ids()
        pitch_min, pitch_max = get_motor_limits(pitch_joint_id)
        roll_min, roll_max = get_motor_limits(roll_joint_id)
        pitch_target = normalize_target_for_limits(0.0, pitch_min, pitch_max, reference_deg=0.0)
        roll_target = normalize_target_for_limits(0.0, roll_min, roll_max, reference_deg=0.0)
        if not is_full_range(pitch_min, pitch_max):
            pitch_target = clampf(pitch_target, pitch_min, pitch_max)
        if not is_full_range(roll_min, roll_max):
            roll_target = clampf(roll_target, roll_min, roll_max)
        set_wrist_diff_targets(pitch_target, roll_target)
        last_sent_logical_deg[pitch_joint_id] = pitch_target
        last_sent_logical_deg[roll_joint_id] = roll_target
        did_queue = True

    cmds = []
    for mid in motor_ids:
        min_a, max_a = get_motor_limits(mid)
        target_deg = normalize_target_for_limits(0.0, min_a, max_a, reference_deg=0.0)
        if not is_full_range(min_a, max_a):
            target_deg = clampf(target_deg, min_a, max_a)
        cmds.append(f"P {mid} {target_deg:.2f}")
        last_sent_logical_deg[mid] = target_deg

    if not cmds:
        if did_queue:
            print("Auto-zero move queued for differential wrist outputs.")
            return True
        print("Auto-zero move skipped: no valid zero commands generated.")
        return False

    packet = ";".join(cmds)
    for send_idx in range(3):
        send_command(packet)
        if send_idx < 2:
            time.sleep(0.04)
    if did_queue:
        print("Auto-zero move queued for differential wrist outputs and joints:", ", ".join(f"J{mid}" for mid in motor_ids))
    else:
        print("Auto-zero move queued for joints:", ", ".join(f"J{mid}" for mid in motor_ids))
    return True

def toggle_connection_callback(event):
    if _connected_controller_count() > 0:
        disconnect_serial()
        print("Disconnected from serial device.")
    else:
        if connect_serial():
            print("Serial connection established.")
        else:
            print("Serial connection failed.")
    render_plot()

def read_from_port(controller_name):
    global latest_motor_pos_deg, latest_motor_apos_deg, latest_motor_telemetry_time, last_telemetry_line
    motor_re = re.compile(r"M(\d+)")
    pos_re = re.compile(r"POS:([^\s|]+)")
    apos_re = re.compile(r"APOS:([^\s|]+)")
    abspid_skip_re = re.compile(r"M(\d+)\s+ABSPID skipped:\s*(.+)")
    abspid_abs_ok_re = re.compile(r"M(\d+)\s+PID source:\s*ABS\b")
    conn = controller_serial.get(controller_name)
    while conn and conn.is_open:
        try:
            if conn.in_waiting > 0:
                line = conn.readline().decode('utf-8', errors='ignore').strip()
                if line:
                    controller_last_telemetry_line[controller_name] = line
                    last_telemetry_line = line
                    print(f"[{controller_name}] {line}")

                    skip_match = abspid_skip_re.search(line)
                    if skip_match:
                        try:
                            motor_id = int(skip_match.group(1))
                            reason = skip_match.group(2).strip()
                            abspid_skip_state[motor_id] = reason
                        except Exception:
                            pass

                    abs_ok_match = abspid_abs_ok_re.search(line)
                    if abs_ok_match:
                        try:
                            motor_id = int(abs_ok_match.group(1))
                            if motor_id in abspid_skip_state:
                                del abspid_skip_state[motor_id]
                        except Exception:
                            pass

                    for token in line.split("|"):
                        token = token.strip()
                        if not token.startswith("M"):
                            continue
                        mm = motor_re.search(token)
                        pm = pos_re.search(token)
                        am = apos_re.search(token)
                        if not mm:
                            continue
                        try:
                            mid = int(mm.group(1))
                            if not _controller_owns_motor(controller_name, mid):
                                continue
                            now = time.time()
                            if pm:
                                pv = _safe_float(pm.group(1))
                                if pv is not None:
                                    latest_motor_pos_deg[mid] = pv
                            if am:
                                av = _safe_float(am.group(1))
                                if av is not None:
                                    latest_motor_apos_deg[mid] = av
                            latest_motor_telemetry_time[mid] = now
                        except Exception:
                            pass
            time.sleep(0.01)
        except (serial.SerialException, OSError) as e:
            print(f"Serial read error ({controller_name}): {e}")
            _handle_controller_disconnect(controller_name, reason="read error")
            break
        except Exception:
            _handle_controller_disconnect(controller_name, reason="reader exception")
            break

def _is_motion_command(cmd):
    motion_ops = {"S", "P", "PR", "M", "MV", "CAL", "JLIM"}
    for part in [p.strip() for p in cmd.split(";") if p.strip()]:
        toks = part.split()
        op = toks[0].upper() if toks else ""
        if op in motion_ops:
            return True
    return False


def command_tx_loop(controller_name):
    while serial_running:
        try:
            cmd = controller_cmd_queues[controller_name].get(timeout=0.1)
        except queue.Empty:
            continue
        try:
            conn = controller_serial.get(controller_name)
            if conn and conn.is_open:
                conn.write((cmd + '\n').encode('utf-8'))
                time.sleep(0.015)
        except Exception as e:
            print(f"Error sending command ({controller_name}): {e}")
            _handle_controller_disconnect(controller_name, reason="write error")
        finally:
            controller_cmd_queues[controller_name].task_done()


def send_command(cmd):
    global last_serial_command, last_serial_command_time, serial_command_count
    if _connected_controller_count() > 0:
        try:
            if _is_motion_command(cmd):
                _clear_pending_commands()
            routed_packets = _route_command_packets(cmd)
            if not routed_packets:
                print(f"[TX SKIPPED] No controller route for: {cmd}")
                return
            for controller_name, packet in routed_packets.items():
                conn = controller_serial.get(controller_name)
                if conn and conn.is_open:
                    controller_cmd_queues[controller_name].put(packet)
                    print(f"[TX QUEUED:{controller_name}] {packet}")
            last_serial_command = cmd
            last_serial_command_time = time.time()
            serial_command_count += 1
        except Exception as e:
            print(f"Error sending command: {e}")
    else:
        print(f"[TX BLOCKED] Serial not connected: {cmd}")

def send_joints_to_arm(joint_angles_rad, smooth=False, max_ramp_steps=20, force_immediate=False, repeat_count=1):
    """Send IK joint angles with limit-aware clamping and optional ramping."""
    global last_command_wall_time

    joint_angles_rad = clamp_seed_angles_to_bounds(enforce_locked_joints(joint_angles_rad))

    target_logical = {}
    use_wrist_diff = wrist_diff_enabled()
    pitch_joint_id, roll_joint_id = wrist_diff_joint_ids()

    for i in range(1, min(7, len(joint_angles_rad))):
        if PLANAR_MODE and i in LOCKED_MOTORS:
            continue
        mid = str(i)
        zero_off = _get_ik_zero_offset(mid)

        deg = np.degrees(joint_angles_rad[i])

        logical_target = deg + zero_off
        min_a, max_a = get_motor_limits(i)
        current_ref = get_start_command_position(i, logical_target)
        logical_target = normalize_target_for_limits(
            logical_target, min_a, max_a, reference_deg=current_ref
        )
        if not is_full_range(min_a, max_a):
            logical_target = clampf(logical_target, min_a, max_a)
        target_logical[i] = logical_target

    if use_wrist_diff:
        pitch_target = target_logical.get(pitch_joint_id)
        roll_target = target_logical.get(roll_joint_id)
        if pitch_target is not None or roll_target is not None:
            set_wrist_diff_targets(pitch_target, roll_target)

    if not target_logical:
        print("[TX SKIPPED] No active motor targets were generated.")
        return

    if force_immediate:
        cmds = []
        for mid, target in target_logical.items():
            if use_wrist_diff and mid in {pitch_joint_id, roll_joint_id}:
                last_sent_logical_deg[mid] = target
                continue
            cmds.append(f"P {mid} {target:.2f}")
            last_sent_logical_deg[mid] = target
        if cmds:
            packet = ";".join(cmds)
            repeat_total = max(1, int(repeat_count))
            for send_idx in range(repeat_total):
                send_command(packet)
                if send_idx + 1 < repeat_total:
                    time.sleep(0.04)
            last_command_wall_time = time.time()
        else:
            print("[TX SKIPPED] Immediate send produced no commands.")
        return

    now = time.time()
    dt = now - last_command_wall_time if last_command_wall_time > 0 else 0.05
    dt = clampf(dt, 0.02, 0.20)
    max_step = min(MAX_COMMAND_STEP_DEG, MAX_COMMAND_RATE_DEG_PER_SEC * dt)
    if max_step < 0.5:
        max_step = 0.5

    planned_current = {
        mid: get_start_command_position(mid, target)
        for mid, target in target_logical.items()
    }

    steps = max_ramp_steps if smooth else 1
    for _ in range(steps):
        cmds = []
        reached_all = True

        for mid, target in target_logical.items():
            current = planned_current[mid]
            next_cmd, reached = command_step_toward(mid, current, target, max_step)
            reached_all = reached_all and reached
            planned_current[mid] = next_cmd

            prev = last_sent_logical_deg.get(mid)
            if use_wrist_diff and mid in {pitch_joint_id, roll_joint_id}:
                last_sent_logical_deg[mid] = next_cmd
                continue
            if prev is None or command_distance_deg(mid, prev, next_cmd) >= COMMAND_DEADBAND_DEG:
                cmds.append(f"P {mid} {next_cmd:.2f}")
                last_sent_logical_deg[mid] = next_cmd
            else:
                last_sent_logical_deg[mid] = next_cmd

        if cmds:
            send_command(";".join(cmds))
            last_command_wall_time = time.time()
        else:
            print("[TX SKIPPED] Ramp/deadband suppressed this step.")

        if reached_all:
            break
        time.sleep(RAMP_INTERVAL_SEC)

# --- UI and Visualization ---
# Key states
keys_pressed = {
    'w': False, 's': False,
    'a': False, 'd': False,
    'q': False, 'e': False,
    'up': False, 'down': False,
    'left': False, 'right': False,
    'shift': False  # Modifier
}

def on_press(key):
    try:
        if hasattr(key, 'char') and key.char:
            char = key.char.lower()
            if char in keys_pressed:
                keys_pressed[char] = True
        elif key == keyboard.Key.up:
            keys_pressed['up'] = True
        elif key == keyboard.Key.down:
            keys_pressed['down'] = True
        elif key == keyboard.Key.left:
            keys_pressed['left'] = True
        elif key == keyboard.Key.right:
            keys_pressed['right'] = True
        elif key == keyboard.Key.shift or key == keyboard.Key.shift_r:
            keys_pressed['shift'] = True
    except AttributeError:
        pass

def on_release(key):
    try:
        if hasattr(key, 'char') and key.char:
            char = key.char.lower()
            if char in keys_pressed:
                keys_pressed[char] = False
        elif key == keyboard.Key.up:
            keys_pressed['up'] = False
        elif key == keyboard.Key.down:
            keys_pressed['down'] = False
        elif key == keyboard.Key.left:
            keys_pressed['left'] = False
        elif key == keyboard.Key.right:
            keys_pressed['right'] = False
        elif key == keyboard.Key.shift or key == keyboard.Key.shift_r:
            keys_pressed['shift'] = False
    except AttributeError:
        pass

def finish_preview():
    global is_previewing
    is_previewing = False

def extract_xyz(positions):
    x = [p[0,3] for p in positions]
    y = [p[1,3] for p in positions]
    z = [p[2,3] for p in positions]
    if DISPLAY_MIRROR_X:
        x = [-v for v in x]
    return x, y, z

def init_plot_elements():
    global arm_line, ghost_line, target_point, joint_texts
    global arm_line_top, ghost_line_top, target_point_top
    global arm_line_side, ghost_line_side, target_point_side
    global ax, ax_top, ax_side

    for a in [ax, ax_side]:
        if a is not None:
            a.cla()
            a.set_xlim(-0.6, 0.6)
            a.set_ylim(-0.6, 0.6)
            a.set_zlim(0, 0.8)
            a.set_xlabel('X')
            a.set_ylabel('Y')
            a.set_zlabel('Z')

    ax.set_title("Main 3D View")
    ax_side.set_title("Side View (XZ)")

    ghost_line, = ax.plot([], [], [], '-o', color='darkorange', alpha=0.4, linewidth=4, markersize=8)
    arm_line, = ax.plot([], [], [], '-o', color='tab:blue', linewidth=5, markersize=10)
    target_point, = ax.plot([], [], [], 'x', color='red', markersize=12, markeredgewidth=2)
    ghost_line_top = None
    arm_line_top = None
    target_point_top = None
    
    ghost_line_side, = ax_side.plot([], [], [], '-o', color='darkorange', alpha=0.4, linewidth=4, markersize=8)
    arm_line_side, = ax_side.plot([], [], [], '-o', color='tab:blue', linewidth=5, markersize=10)
    target_point_side, = ax_side.plot([], [], [], 'x', color='red', markersize=12, markeredgewidth=2)
    
    joint_texts = [ax.text(0, 0, 0, f"J{i}", color='darkred', fontsize=11, fontweight='bold', visible=False) for i in range(1, 7)]

def render_plot():
    global actual_joint_angles, arm_line, ghost_line, target_point, joint_texts
    global arm_line_top, ghost_line_top, target_point_top
    global arm_line_side, ghost_line_side, target_point_side
    global ax, ax_top, ax_side
    global txt_angles, txt_joint_dashboard, txt_connection_status

    if my_chain is None:
        return
        
    if actual_joint_angles is None:
        # By default, start with the current virtual position
        actual_joint_angles = list(current_joint_angles)

    if sync_model_to_telemetry and not is_previewing:
        tele_angles = get_current_real_joint_angles()
        if tele_angles is not None:
            actual_joint_angles = tele_angles

    if ghost_line is None:
        init_plot_elements()

    # compute ghost and arm poses
    ghost_pos = my_chain.forward_kinematics(actual_joint_angles, full_kinematics=True)
    gx, gy, gz = extract_xyz(ghost_pos)
    for gl in [ghost_line, ghost_line_side]:
        gl.set_data(gx, gy)
        gl.set_3d_properties(gz)

    angles_to_draw = current_joint_angles
    if is_previewing:
        t = (time.time() - preview_start_time) / PREVIEW_DURATION
        if t >= 1.0:
            finish_preview()
        else:
            angles_to_draw = [(1-t)*a + t*b for a, b in zip(actual_joint_angles, current_joint_angles)]
    
    arm_pos = my_chain.forward_kinematics(angles_to_draw, full_kinematics=True)
    x, y, z = extract_xyz(arm_pos)
    for al in [arm_line, arm_line_side]:
        al.set_data(x, y)
        al.set_3d_properties(z)

    # labels
    for i in range(1, min(7, len(arm_pos))):
        mx, my, mz = x[i], y[i], z[i]
        joint_texts[i-1].set_position_3d((mx, my, mz + 0.04))
        joint_texts[i-1].set_visible(True)

    # marker
    for tp in [target_point, target_point_side]:
        target_x = -current_target_xyz[0] if DISPLAY_MIRROR_X else current_target_xyz[0]
        tp.set_data([target_x], [current_target_xyz[1]])
        tp.set_3d_properties([current_target_xyz[2]])

    ax.view_init(elev=cam_elev, azim=cam_azim)
    ax_side.view_init(elev=0, azim=-90)

    if fig is not None:
        if txt_joint_dashboard is None:
            txt_joint_dashboard = fig.text(
                0.66,
                0.94,
                "",
                fontsize=9,
                va='top',
                ha='left',
                bbox=dict(facecolor='whitesmoke', alpha=0.9, edgecolor='black', boxstyle='round,pad=0.5'),
            )
        if txt_connection_status is None:
            txt_connection_status = fig.text(
                0.66,
                0.82,
                "",
                fontsize=9,
                va='top',
                ha='left',
                family='monospace',
                bbox=dict(facecolor='aliceblue', alpha=0.9, edgecolor='black', boxstyle='round,pad=0.5'),
            )

        rows = []
        worst_err = 0.0
        have_any_apos = False
        for j in range(1, 7):
            if j >= len(current_joint_angles):
                continue
            tgt_deg = float(np.degrees(current_joint_angles[j]))
            cur_deg = get_current_ik_deg_from_apos(j)
            if cur_deg is None:
                rows.append(f"J{j}:  tgt {tgt_deg:7.1f}°   cur   ---.-°   err  ---.-°")
                continue

            have_any_apos = True
            err = abs(cur_deg - tgt_deg)
            worst_err = max(worst_err, err)
            rows.append(f"J{j}:  tgt {tgt_deg:7.1f}°   cur {cur_deg:7.1f}°   err {err:6.1f}°")

        txt_joint_dashboard.set_text("Joint Dashboard (APOS)\n" + "\n".join(rows))
        try:
            txt_joint_dashboard.get_bbox_patch().set_facecolor('#f4f4f4' if have_any_apos else '#dddddd')
        except Exception:
            pass
        txt_connection_status.set_text(get_connection_status_text())
        try:
            if bool(ser and ser.is_open) and has_fresh_telemetry():
                txt_connection_status.get_bbox_patch().set_facecolor('#d8f5d0')
            elif bool(ser and ser.is_open):
                txt_connection_status.get_bbox_patch().set_facecolor('#fff2b3')
            else:
                txt_connection_status.get_bbox_patch().set_facecolor('#f4cccc')
        except Exception:
            pass
        update_connection_button()
        
    fig.canvas.draw_idle()

def keyboard_update_loop():
    global current_target_xyz, current_target_rpy, cam_elev, cam_azim
    step = slider_step.val # meters per tick
    rot_step = 2.0 # degrees per tick for wrist
    cam_step = 2.0 # degrees per tick for camera
    
    d1, d2, d3 = 0, 0, 0 # generic deltas
    moved_cam = False

    if keys_pressed['w']: d2 += 1 # forward 
    if keys_pressed['s']: d2 -= 1 # backward
    if keys_pressed['a']: d1 -= 1 # left
    if keys_pressed['d']: d1 += 1 # right
    if keys_pressed['e']: d3 += 1 # up
    if keys_pressed['q']: d3 -= 1 # down
    
    if keys_pressed['up']:
        cam_elev += cam_step
        moved_cam = True
    if keys_pressed['down']:
        cam_elev -= cam_step
        moved_cam = True
    if keys_pressed['left']:
        cam_azim -= cam_step
        moved_cam = True
    if keys_pressed['right']:
        cam_azim += cam_step
        moved_cam = True

    cam_elev = max(-90, min(90, cam_elev))
    
    if d1 != 0 or d2 != 0 or d3 != 0:
        if keys_pressed['shift']:
            # Adjust RPY
            new_r = max(slider_r.valmin, min(slider_r.valmax, current_target_rpy[0] + d1*rot_step))
            new_p = max(slider_p.valmin, min(slider_p.valmax, current_target_rpy[1] + d2*rot_step))
            new_y = max(slider_y.valmin, min(slider_y.valmax, current_target_rpy[2] + d3*rot_step))
            current_target_rpy = [new_r, new_p, new_y]
        else:
            # Adjust XYZ
            new_x = max(slider_x.valmin, min(slider_x.valmax, current_target_xyz[0] + d1*step))
            target_y = planar_target_y if PLANAR_MODE else current_target_xyz[1]
            new_y = max(slider_y.valmin, min(slider_y.valmax, target_y + d2*step))
            new_z = max(slider_z.valmin, min(slider_z.valmax, current_target_xyz[2] + d3*step))
            current_target_xyz = [new_x, planar_target_y if PLANAR_MODE else new_y, new_z]
        
        update_plot(val=None)
    elif is_previewing or moved_cam:
        render_plot()

def update_joint(val, idx):
    global current_target_xyz, current_joint_angles
    if is_previewing:
        return
    
    if idx < len(current_joint_angles):
        current_joint_angles[idx] = np.radians(val)
    current_joint_angles = enforce_locked_joints(current_joint_angles)
    refresh_locked_joint_rads(current_joint_angles)
    
    new_angles = solve_ik_for_target(current_joint_angles)
    if new_angles is None:
        return
    
    current_joint_angles = list(new_angles)
    
    # Update Joint sliders silently to reflect the solver's snapped state
    if 'joint_sliders' in globals():
        for i, js in enumerate(joint_sliders):
            if i+1 < len(current_joint_angles):
                js.eventson = False
                js.set_val(np.degrees(current_joint_angles[i+1]))
                js.eventson = True
        
    render_plot()

def update_plot(val=None):
    global current_target_xyz, current_target_rpy, current_joint_angles, last_send_time, planar_target_y
    global is_previewing
    
    if is_previewing:
        # Cancel preview if user manually moves target
        is_previewing = False

    if val is not None:
        # Came from slider
        x = slider_x.val
        y = slider_y.val
        z = slider_z.val
        r = slider_r.val
        p = slider_p.val
        yw = slider_y_rot.val
        
        if PLANAR_MODE:
            planar_target_y = float(y)
        current_target_xyz = [x, planar_target_y if PLANAR_MODE else y, z]
        current_target_rpy = [r, p, yw]
    else:
        # Came from keyboard, update slider silently
        slider_x.eventson = False
        slider_y.eventson = False
        slider_z.eventson = False
        slider_r.eventson = False
        slider_p.eventson = False
        slider_y_rot.eventson = False
        
        slider_x.set_val(current_target_xyz[0])
        slider_y.set_val(current_target_xyz[1])
        slider_z.set_val(current_target_xyz[2])
        slider_r.set_val(current_target_rpy[0])
        slider_p.set_val(current_target_rpy[1])
        slider_y_rot.set_val(current_target_rpy[2])
        
        slider_x.eventson = True
        slider_y.eventson = True
        slider_z.eventson = True
        slider_r.eventson = True
        slider_p.eventson = True
        slider_y_rot.eventson = True

    solved_angles = solve_ik_for_target(current_joint_angles)
    if solved_angles is not None:
        current_joint_angles = list(solved_angles)
        refresh_locked_joint_rads(current_joint_angles)
    
    # Update Joint sliders silently
    if 'joint_sliders' in globals():
        for i, js in enumerate(joint_sliders):
            if i+1 < len(current_joint_angles):
                js.eventson = False
                js.set_val(np.degrees(current_joint_angles[i+1]))
                js.eventson = True

    render_plot()

    # Live update mechanism (throttle updates to 10Hz)
    current_time = time.time()
    if live_update and (current_time - last_send_time > 0.1):
        execute_button_callback(None)

def find_alternative_ik(event):
    global current_joint_angles
    print("Searching for an alternative IK solution...")
    for _ in range(50): # try up to 50 times
        seed = [0.0] * len(current_joint_angles)
        for i in range(1, len(seed)):
            seed[i] = np.random.uniform(-np.pi, np.pi)
        seed = clamp_seed_angles_to_bounds(enforce_locked_joints(seed))
        
        test_angles = solve_ik_for_target(seed)
        if test_angles is None:
            continue
        fk = my_chain.forward_kinematics(test_angles)
        
        # Check if the generated solution actually hits the target
        dist = np.linalg.norm(fk[:3, 3] - current_target_xyz)
        if dist < 0.05: # within 5cm accuracy
            # Check if it's sufficiently different from the current pose 
            diff = np.linalg.norm(np.array(test_angles[1:]) - np.array(current_joint_angles[1:]))
            if diff > 0.5:
                current_joint_angles = list(test_angles)
                refresh_locked_joint_rads(current_joint_angles)
                print(f"Found alternative solution! (Diff: {diff:.2f} rad, Error: {dist:.4f}m)")
                
                # Update Joint sliders silently
                if 'joint_sliders' in globals():
                    for i, js in enumerate(joint_sliders):
                        if i+1 < len(current_joint_angles):
                            js.eventson = False
                            js.set_val(np.degrees(current_joint_angles[i+1]))
                            js.eventson = True
                            
                render_plot()
                return
                
    print("Could not find a significantly different Alternative IK solution for this coordinate.")

def preview_button_callback(event):
    global is_previewing, preview_start_time
    print("Previewing Plan (Ghost -> Target)...")
    is_previewing = True
    preview_start_time = time.time()
    render_plot()

def execute_button_callback(event):
    global actual_joint_angles, last_send_time, current_joint_angles
    if event is not None:
        print("Executing: Sending joint angles to Arm!")
    current_joint_angles = clamp_seed_angles_to_bounds(enforce_locked_joints(current_joint_angles))
    actual_joint_angles = list(current_joint_angles)
    refresh_locked_joint_rads(actual_joint_angles)
    # Manual execute always sends an explicit final position packet.
    # Live updates continue to use a single-step rate-limited send path.
    send_joints_to_arm(
        current_joint_angles,
        smooth=False,
        max_ramp_steps=1,
        force_immediate=True,
        repeat_count=3,
    )
    last_send_time = time.time()
    render_plot()

def toggle_live_update(label):
    global live_update
    live_update = not live_update
    if live_update:
        print("Live update ENABLED.")
    else:
        print("Live update DISABLED.")

def toggle_sync_model_to_telemetry(label):
    global sync_model_to_telemetry
    sync_model_to_telemetry = not sync_model_to_telemetry
    if sync_model_to_telemetry:
        tele_angles = get_current_real_joint_angles()
        if tele_angles is not None:
            global actual_joint_angles
            actual_joint_angles = tele_angles
            refresh_locked_joint_rads(actual_joint_angles)
        print("Ghost sync to telemetry ENABLED.")
    else:
        print("Ghost sync to telemetry DISABLED.")
    render_plot()

def set_current_pose_as_motor_and_ik_zero(event):
    updated = []
    cal_cmds = []
    for mid in (1, 2, 3, 4, 5, 6):
        logical_now = latest_motor_pos_deg.get(mid)
        if logical_now is None:
            continue
        cfg = robot_config.setdefault(str(mid), {})
        apos_now = latest_motor_apos_deg.get(mid)
        if apos_now is not None:
            cfg["abs_zero_offset"] = float(apos_now)
        cfg["zero_offset"] = 0.0
        cfg["ik_zero_offset"] = 0.0
        cal_cmds.append(f"CAL {mid} 0")
        updated.append(mid)

    if not updated:
        print("Set zero skipped: no telemetry available.")
        return

    if cal_cmds:
        send_command("; ".join(cal_cmds))
        time.sleep(0.10)
    save_robot_config()
    sync_solver_seed_from_telemetry(update_target=True)
    print("Set current pose as motor+IK zero for joints:", ", ".join(f"J{m}" for m in updated))
    render_plot()

def emergency_stop_callback(event):
    global hand_pwm_command, live_update
    print("EMERGENCY STOP TRIGGERED!")
    live_update = False
    for mid in last_sent_logical_deg:
        last_sent_logical_deg[mid] = None
    clear_wrist_diff_targets(send_stop=False)
    _clear_pending_commands()
    hand_pwm_command = 0
    for send_idx in range(3):
        send_command("S; HSTOP")
        if send_idx < 2:
            time.sleep(0.03)


def hand_open_button_callback(event):
    open_hand(255)


def hand_close_button_callback(event):
    close_hand(255)


def hand_stop_button_callback(event):
    stop_hand()

def main():
    global my_chain, fig, ax, ax_top, ax_side
    global slider_x, slider_y, slider_z, slider_step, txt_angles
    global slider_r, slider_p, slider_y_rot
    global joint_sliders, slider_prev, btn_connection, planar_target_y
    
    load_robot_config()
    print("Loading URDF file:", URDF_FILE)
    try:
        # ikpy doesn't support 'continuous' joints, so we patch it in memory
        with open(URDF_FILE, 'r') as f:
            urdf_data = f.read()
        urdf_data = urdf_data.replace('type="continuous"', 'type="revolute"')
        
        with tempfile.NamedTemporaryFile(delete=False, mode='w', suffix='.urdf') as temp_urdf:
            temp_urdf.write(urdf_data)
            temp_path = temp_urdf.name

        my_chain = Chain.from_urdf_file(temp_path)
        
        global current_joint_angles
        num_joints = len(my_chain.links)
        current_joint_angles = clamp_seed_angles_to_bounds([0.0] * num_joints)
        refresh_locked_joint_rads(current_joint_angles)
        apply_chain_bounds_from_config()
        
        os.remove(temp_path)
        
    except Exception as e:
        print(f"Failed to load URDF: {e}")
        traceback.print_exc()
        print(f"Make sure {URDF_FILE} is placed in the right directory.")
        sys.exit(1)

    connect_serial()
    if not sync_solver_seed_from_telemetry(update_target=True):
        refresh_locked_joint_rads(current_joint_angles)
        apply_chain_bounds_from_config()
        sync_target_from_joint_angles(current_joint_angles)
    planar_target_y = float(current_target_xyz[1])

    fig = plt.figure(figsize=(15, 8.5))
    fig.canvas.manager.set_window_title("6DoF Robot Arm IK Visualization")
    
    ax = fig.add_axes([0.03, 0.18, 0.36, 0.52], projection='3d')
    ax_top = None
    ax_side = fig.add_axes([0.41, 0.22, 0.18, 0.26], projection='3d')

    slider_width = 0.28
    slider_left = 0.66
    dh = 0.033
    current_y = 0.58

    axcolor = 'lightgoldenrodyellow'
    ax_x = plt.axes([slider_left, current_y, slider_width, 0.022], facecolor=axcolor); current_y -= dh
    ax_y = plt.axes([slider_left, current_y, slider_width, 0.022], facecolor=axcolor); current_y -= dh
    ax_z = plt.axes([slider_left, current_y, slider_width, 0.022], facecolor=axcolor); current_y -= dh
    
    current_y -= 0.01
    ax_r = plt.axes([slider_left, current_y, slider_width, 0.022], facecolor='lightgreen'); current_y -= dh
    ax_p = plt.axes([slider_left, current_y, slider_width, 0.022], facecolor='lightgreen'); current_y -= dh
    ax_yw= plt.axes([slider_left, current_y, slider_width, 0.022], facecolor='lightgreen'); current_y -= dh
    
    current_y -= 0.01
    ax_step = plt.axes([slider_left, current_y, slider_width, 0.022], facecolor=axcolor); current_y -= dh
    
    current_y -= 0.015
    ax_j1 = plt.axes([slider_left, current_y, slider_width, 0.022], facecolor='lightblue'); current_y -= dh
    ax_j2 = plt.axes([slider_left, current_y, slider_width, 0.022], facecolor='lightblue'); current_y -= dh
    ax_j3 = plt.axes([slider_left, current_y, slider_width, 0.022], facecolor='lightblue'); current_y -= dh
    ax_j4 = plt.axes([slider_left, current_y, slider_width, 0.022], facecolor='lightblue'); current_y -= dh
    ax_j5 = plt.axes([slider_left, current_y, slider_width, 0.022], facecolor='lightblue'); current_y -= dh
    ax_j6 = plt.axes([slider_left, current_y, slider_width, 0.022], facecolor='lightblue'); current_y -= dh
    
    current_y -= 0.01
    ax_prev_t = plt.axes([slider_left, current_y, slider_width, 0.022], facecolor=axcolor); current_y -= dh

    # Sliders mapping
    slider_x = Slider(ax_x, 'X Target', -1.0, 1.0, valinit=current_target_xyz[0])
    slider_y = Slider(ax_y, 'Y Target', -1.0, 1.0, valinit=current_target_xyz[1])
    slider_z = Slider(ax_z, 'Z Target', 0.0, 1.5, valinit=current_target_xyz[2])
    
    slider_r = Slider(ax_r, 'Roll', -180.0, 180.0, valinit=current_target_rpy[0])
    slider_p = Slider(ax_p, 'Pitch', -180.0, 180.0, valinit=current_target_rpy[1])
    slider_y_rot = Slider(ax_yw, 'Yaw', -180.0, 180.0, valinit=current_target_rpy[2])
    
    slider_step = Slider(ax_step, 'Key Step', 0.001, 0.05, valinit=0.01)

    slider_x.on_changed(update_plot)
    slider_y.on_changed(update_plot)
    slider_z.on_changed(update_plot)
    slider_r.on_changed(update_plot)
    slider_p.on_changed(update_plot)
    slider_y_rot.on_changed(update_plot)
    
    joint_sliders = []
    num_active = min(6, len(current_joint_angles)-1)
    if num_active >= 1: joint_sliders.append(Slider(ax_j1, 'J1 (deg)', -360, 360, valinit=np.degrees(current_joint_angles[1])))
    if num_active >= 2: joint_sliders.append(Slider(ax_j2, 'J2 (deg)', -360, 360, valinit=np.degrees(current_joint_angles[2])))
    if num_active >= 3: joint_sliders.append(Slider(ax_j3, 'J3 (deg)', -360, 360, valinit=np.degrees(current_joint_angles[3])))
    if num_active >= 4: joint_sliders.append(Slider(ax_j4, 'J4 (deg)', -360, 360, valinit=np.degrees(current_joint_angles[4])))
    if num_active >= 5: joint_sliders.append(Slider(ax_j5, 'J5 (deg)', -360, 360, valinit=np.degrees(current_joint_angles[5])))
    if num_active >= 6: joint_sliders.append(Slider(ax_j6, 'J6 (deg)', -360, 360, valinit=np.degrees(current_joint_angles[6])))
    
    for i, js in enumerate(joint_sliders):
        def make_update(idx):
            return lambda val: update_joint(val, idx)
        js.on_changed(make_update(i+1))

    slider_prev = Slider(ax_prev_t, 'Prev Time(s)', 0.1, 5.0, valinit=PREVIEW_DURATION)
    def update_prev_time(val):
        global PREVIEW_DURATION
        PREVIEW_DURATION = val
    slider_prev.on_changed(update_prev_time)

    ax_conn = plt.axes([0.66, 0.31, 0.14, 0.045])
    btn_connection = Button(ax_conn, 'Disconnected', color='lightcoral', hovercolor='0.9')
    btn_connection.on_clicked(toggle_connection_callback)

    ax_exec = plt.axes([0.82, 0.31, 0.12, 0.045])
    btn_exec = Button(ax_exec, 'Execute', color='lightgreen', hovercolor='0.9')
    btn_exec.on_clicked(execute_button_callback)

    ax_alt = plt.axes([0.66, 0.25, 0.14, 0.045])
    btn_alt = Button(ax_alt, 'Alternative IK', color='thistle', hovercolor='0.9')
    btn_alt.on_clicked(find_alternative_ik)

    ax_prev_btn = plt.axes([0.82, 0.25, 0.12, 0.045])
    btn_prev = Button(ax_prev_btn, 'Preview (Plan)', color='lightblue', hovercolor='0.9')
    btn_prev.on_clicked(preview_button_callback)

    ax_hand_open = plt.axes([0.66, 0.19, 0.14, 0.045])
    btn_hand_open = Button(ax_hand_open, 'Hand Open', color='palegreen', hovercolor='0.9')
    btn_hand_open.on_clicked(hand_open_button_callback)

    ax_hand_close = plt.axes([0.82, 0.19, 0.12, 0.045])
    btn_hand_close = Button(ax_hand_close, 'Hand Close', color='mistyrose', hovercolor='0.9')
    btn_hand_close.on_clicked(hand_close_button_callback)

    ax_live = plt.axes([0.66, 0.145, 0.28, 0.04])
    chk_live = CheckButtons(ax_live, ['Live Execute'], [live_update])
    chk_live.on_clicked(toggle_live_update)

    ax_sync = plt.axes([0.66, 0.10, 0.28, 0.04])
    chk_sync = CheckButtons(ax_sync, ['Sync Model to Telemetry'], [sync_model_to_telemetry])
    chk_sync.on_clicked(toggle_sync_model_to_telemetry)

    ax_hand_stop = plt.axes([0.66, 0.055, 0.28, 0.035])
    btn_hand_stop = Button(ax_hand_stop, 'Hand Stop', color='lightyellow', hovercolor='0.9')
    btn_hand_stop.on_clicked(hand_stop_button_callback)

    ax_capture = plt.axes([0.66, 0.015, 0.28, 0.035])
    btn_capture = Button(ax_capture, 'Set Current Pose as Motor+IK Zero', color='khaki', hovercolor='0.9')
    btn_capture.on_clicked(set_current_pose_as_motor_and_ik_zero)

    ax_stop = plt.axes([0.01, 0.015, 0.16, 0.05])
    btn_stop = Button(ax_stop, 'EMERGENCY STOP', color='salmon', hovercolor='red')
    btn_stop.label.set_fontweight('bold')
    btn_stop.on_clicked(emergency_stop_callback)
    update_connection_button()

    print("\n--- Controls ---")
    print("Use sliders or keys to modify the GOAL target (X, Y, Z).")
    print("Click 'Preview (Plan)' to simulate the arm moving from current state to the goal state.")
    print("Click 'Execute' to send the goal angles to the hardware and update the current state.")
    print("Click 'EMERGENCY STOP' to immediately stop all motors.")
    print("Use 'Hand Open', 'Hand Close', and 'Hand Stop' to control the PWM gripper.")
    print("Tick 'Live Execute' to automatically execute immediately when moving.")
    print("Tick 'Sync Model to Telemetry' to make only the ghost arm follow the real arm.")
    print("Click 'Set Current Pose as Motor+IK Zero' to make the current physical pose zero for both motor control and IK.")
    print("Close the window to exit.")
    
    # Start pynput keyboard listener
    listener = keyboard.Listener(on_press=on_press, on_release=on_release)
    listener.start()

    print("\n--- Additional Controls ---")
    print("Use WASD to move XYZ (Forward/Back/Left/Right). Use Q/E to move Z (Down/Up).")
    print("Hold SHIFT + WASD/QE to adjust the Roll/Pitch/Yaw instead.")
    print("Use ARROW KEYS to rotate the Main Camera View.")

    # Start matplotlib timer for smooth keyboard polling
    timer = fig.canvas.new_timer(interval=10)
    timer.add_callback(keyboard_update_loop)
    timer.start()

    # Initial Plot
    update_plot()
    
    # Show Window (blocks until closed)
    plt.show()

    listener.stop()

    if ser and ser.is_open:
        disconnect_serial()
    print("Exited.")

if __name__ == "__main__":
    main()
