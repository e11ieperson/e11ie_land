"""
multi_motor_ssh_controller.py
SSH-compatible replacement for multi_motor_keyboard_velocity_control.py.

Uses raw terminal mode (termios) instead of pynput — works over SSH with no
display required.  Hold a key → motor runs.  Release → motor stops.
Identical feel to the original pynput version.

Dependencies: pyserial   (pip install pyserial)
              No pynput needed.

Usage
-----
    python multi_motor_ssh_controller.py [--config robot_config.json]
                                         [--keymap multi_motor_keymap.json]
                                         [--baud 115200]
                                         [--settle 0.5]
"""

import argparse
import json
import os
import sys
import threading
import time
import tty
import termios
import select

import serial
import serial.tools.list_ports


DEFAULT_BAUD_RATE   = 115200
DEFAULT_SPEED       = 50
DEFAULT_KEYMAP_FILE = "multi_motor_keymap.json"
DEFAULT_SETTLE_S    = 0.5
CONNECT_RETRIES     = 5
CONNECT_RETRY_DELAY = 0.5

# How often (seconds) the main loop polls for new keypresses
KEY_POLL_HZ = 0.02   # 50 Hz

# After this many poll cycles with no key seen, treat the key as released.
# At 50 Hz, 3 cycles = 60 ms — comfortable for human reaction time.
RELEASE_CYCLES = 3


def _norm_key_name(raw):
    if raw is None:
        return None
    s = str(raw).strip().lower()
    if not s:
        return None
    aliases = {"escape": "esc", "spacebar": "space", "return": "enter"}
    return aliases.get(s, s)


# ---------------------------------------------------------------------------
# Raw-terminal key reader
# ---------------------------------------------------------------------------

class RawKeyReader:
    """
    Puts stdin into raw, no-echo mode and reads single characters without
    blocking the main thread.  Works over SSH (no display needed).

    Special keys (arrows, F-keys, Esc sequences) are collapsed to a short
    name string so they can be used in the keymap if desired.
    """

    _ANSI = {
        "\x1b[A": "up",    "\x1b[B": "down",
        "\x1b[C": "right", "\x1b[D": "left",
        "\x1b[1~": "home", "\x1b[4~": "end",
        "\x1b[5~": "pageup", "\x1b[6~": "pagedown",
        "\x1b[2~": "insert", "\x1b[3~": "delete",
        "\x1bOP": "f1", "\x1bOQ": "f2", "\x1bOR": "f3", "\x1bOS": "f4",
        "\x1b[15~": "f5",  "\x1b[17~": "f6",
        "\x1b[18~": "f7",  "\x1b[19~": "f8",
        "\x1b[20~": "f9",  "\x1b[21~": "f10",
        "\x1b[23~": "f11", "\x1b[24~": "f12",
    }

    _CTRL = {
        "\x01": "ctrl+a", "\x02": "ctrl+b", "\x03": "ctrl+c",
        "\x04": "ctrl+d", "\x05": "ctrl+e", "\x06": "ctrl+f",
        "\x07": "ctrl+g", "\x08": "backspace", "\x09": "tab",
        "\x0a": "enter",  "\x0b": "ctrl+k",   "\x0c": "ctrl+l",
        "\x0d": "enter",  "\x0e": "ctrl+n",   "\x0f": "ctrl+o",
        "\x10": "ctrl+p", "\x11": "ctrl+q",   "\x12": "ctrl+r",
        "\x13": "ctrl+s", "\x14": "ctrl+t",   "\x15": "ctrl+u",
        "\x16": "ctrl+v", "\x17": "ctrl+w",   "\x18": "ctrl+x",
        "\x19": "ctrl+y", "\x1a": "ctrl+z",   "\x1b": "esc",
        "\x7f": "backspace",
    }

    def __init__(self):
        self._fd       = sys.stdin.fileno()
        self._old_attr = termios.tcgetattr(self._fd)
        self._lock     = threading.Lock()
        self._char_buf = []

        self._running = True
        self._thread  = threading.Thread(target=self._reader_thread, daemon=True)
        self._thread.start()

    def restore(self):
        self._running = False
        try:
            termios.tcsetattr(self._fd, termios.TCSADRAIN, self._old_attr)
        except Exception:
            pass

    def _reader_thread(self):
        tty.setraw(self._fd)
        while self._running:
            r, _, _ = select.select([sys.stdin], [], [], 0.05)
            if not r:
                continue
            try:
                ch = sys.stdin.read(1)
            except Exception:
                break
            if not ch:
                break
            # Collect ANSI escape sequences
            if ch == "\x1b":
                seq = ch
                while True:
                    r2, _, _ = select.select([sys.stdin], [], [], 0.05)
                    if not r2:
                        break
                    nxt = sys.stdin.read(1)
                    seq += nxt
                    if nxt.isalpha() or nxt == "~":
                        break
                ch = seq
            with self._lock:
                self._char_buf.append(ch)

    def read_char(self):
        """Return the next buffered key name, or None if nothing waiting."""
        with self._lock:
            if not self._char_buf:
                return None
            raw = self._char_buf.pop(0)
        if raw in self._ANSI:
            return self._ANSI[raw]
        if raw in self._CTRL:
            return self._CTRL[raw]
        if len(raw) == 1:
            return raw
        return raw  # unknown escape — return raw


# ---------------------------------------------------------------------------
# Main controller
# ---------------------------------------------------------------------------

class MultiMotorSSHController:

    def __init__(self, config_path, keymap_path,
                 baud_rate=DEFAULT_BAUD_RATE, settle_s=DEFAULT_SETTLE_S):
        self.config_path  = config_path
        self.keymap_path  = keymap_path
        self.baud_rate    = int(baud_rate)
        self.settle_s     = float(settle_s)

        self.controller_cfg      = {}
        self.motor_to_controller = {}

        self.serial_by_controller = {}
        self.offline_controllers  = set()
        self.write_locks          = {}
        self.read_threads         = {}
        self.running              = True

        self.key_to_actions   = {}
        self.motor_speed      = {}
        self.current_cmd_rpm  = {}

        self.quit_key              = "esc"
        self.stop_all_key          = "space"
        self.blink_key             = "b"
        self.hand_open_key         = "["
        self.hand_close_key        = "]"
        self.hand_gentle_open_key  = "{"
        self.hand_gentle_close_key = "}"
        self.hand_controller       = None

    # ------------------------------------------------------------------
    # Config loading
    # ------------------------------------------------------------------

    def load_robot_config(self):
        if not os.path.exists(self.config_path):
            raise FileNotFoundError(f"robot_config not found: {self.config_path}")
        with open(self.config_path, "r", encoding="utf-8") as f:
            cfg = json.load(f)

        raw_ctrl = cfg.get("controllers", {}) if isinstance(cfg, dict) else {}
        if not isinstance(raw_ctrl, dict) or not raw_ctrl:
            raise ValueError("robot_config.json missing 'controllers' block")

        self.controller_cfg      = {}
        self.motor_to_controller = {}
        self.hand_controller     = None

        for name, c in raw_ctrl.items():
            if not isinstance(c, dict):
                continue
            port     = c.get("port")
            motors   = c.get("motors", [])
            if not isinstance(motors, list):
                motors = []
            motor_ids = []
            for mid in motors:
                try:
                    motor_ids.append(int(mid))
                except Exception:
                    pass
            hand_cfg = c.get("hand", {})
            self.controller_cfg[name] = {
                "port":   str(port) if port else None,
                "motors": motor_ids,
                "hand":   hand_cfg if isinstance(hand_cfg, dict) else {},
            }
            for mid in motor_ids:
                self.motor_to_controller[mid] = name
            if isinstance(hand_cfg, dict) and hand_cfg.get("enabled"):
                if self.hand_controller is not None:
                    raise ValueError(
                        f"Multiple controllers declare hand enabled: "
                        f"'{self.hand_controller}' and '{name}'"
                    )
                self.hand_controller = name

        if not self.controller_cfg:
            raise ValueError("No valid controller entries in robot_config.json")

        print(f"[hand] routed to controller '{self.hand_controller}'"
              if self.hand_controller else
              "[hand] no controller has hand enabled — hand commands will broadcast")

    def _default_keymap(self):
        motors       = sorted(self.motor_to_controller.keys())
        forward_pool = list("1234567890")
        reverse_pool = list("qwertyuiop")
        motor_map    = {}
        for idx, mid in enumerate(motors):
            fwd = forward_pool[idx] if idx < len(forward_pool) else f"f{mid}"
            rev = reverse_pool[idx] if idx < len(reverse_pool) else f"v{mid}"
            motor_map[str(mid)] = {"forward": fwd, "reverse": rev, "speed": DEFAULT_SPEED}
        return {
            "quit":              "esc",
            "stop_all":          "space",
            "blink":             "b",
            "hand_open":         "[",
            "hand_close":        "]",
            "hand_gentle_open":  "{",
            "hand_gentle_close": "}",
            "motors":            motor_map,
        }

    def load_or_create_keymap(self):
        if not os.path.exists(self.keymap_path):
            default_map = self._default_keymap()
            with open(self.keymap_path, "w", encoding="utf-8") as f:
                json.dump(default_map, f, indent=2)
            print(f"Created default keymap: {self.keymap_path}")

        with open(self.keymap_path, "r", encoding="utf-8") as f:
            keymap = json.load(f)

        self.quit_key              = _norm_key_name(keymap.get("quit",             "esc"))  or "esc"
        self.stop_all_key          = _norm_key_name(keymap.get("stop_all",         "space"))or "space"
        self.blink_key             = _norm_key_name(keymap.get("blink",            "b"))    or "b"
        self.hand_open_key         = _norm_key_name(keymap.get("hand_open",        "["))    or "["
        self.hand_close_key        = _norm_key_name(keymap.get("hand_close",       "]"))    or "]"
        self.hand_gentle_open_key  = _norm_key_name(keymap.get("hand_gentle_open", "{"))    or "{"
        self.hand_gentle_close_key = _norm_key_name(keymap.get("hand_gentle_close","}"))    or "}"

        raw_motors = keymap.get("motors", {})
        if not isinstance(raw_motors, dict):
            raise ValueError("Keymap 'motors' must be an object")

        self.key_to_actions  = {}
        self.motor_speed     = {}
        self.current_cmd_rpm = {}

        for mid, controller_name in sorted(self.motor_to_controller.items()):
            cfg = raw_motors.get(str(mid), {})
            if not isinstance(cfg, dict):
                cfg = {}
            fwd = _norm_key_name(cfg.get("forward"))
            rev = _norm_key_name(cfg.get("reverse"))
            if not fwd or not rev:
                raise ValueError(f"Keymap missing forward/reverse keys for motor {mid}")
            if fwd == rev:
                raise ValueError(f"Motor {mid} forward/reverse keys cannot be the same")
            try:
                speed = int(cfg.get("speed", DEFAULT_SPEED))
            except Exception:
                speed = DEFAULT_SPEED
            speed = max(1, min(2000, abs(speed)))

            self.motor_speed[mid]     = speed
            self.current_cmd_rpm[mid] = 0

            self.key_to_actions.setdefault(fwd, []).append((mid, +1))
            self.key_to_actions.setdefault(rev, []).append((mid, -1))

    # ------------------------------------------------------------------
    # Connection
    # ------------------------------------------------------------------

    def _available_ports(self):
        return {p.device for p in serial.tools.list_ports.comports()}

    def _open_port_with_timeout(self, port, timeout_s=8.0):
        conn_box = [None]
        exc_box  = [None]

        def _open():
            try:
                conn_box[0] = serial.Serial(
                    port, self.baud_rate,
                    timeout=0.5, write_timeout=1.0,
                    dsrdtr=False, rtscts=False, exclusive=True,
                )
            except Exception as e:
                exc_box[0] = e

        t = threading.Thread(target=_open, daemon=True)
        t.start()
        t.join(timeout=timeout_s)

        if t.is_alive():
            print(f"        [{port}] driver stalled, waiting for OS to release handle...")
            t.join(timeout=10.0)
            if t.is_alive():
                raise serial.SerialException(
                    f"Timed out after {timeout_s+10}s opening {port} "
                    f"(persistent driver hang — unplug/replug the board)"
                )
            if conn_box[0] is not None:
                return conn_box[0]
            raise serial.SerialException(
                f"Driver hang on {port} — handle now released, retrying"
            )
        if exc_box[0] is not None:
            raise exc_box[0]
        return conn_box[0]

    def _connect_one(self, name, cfg, results, lock):
        port     = cfg.get("port")
        if not port:
            with lock:
                results[name] = None
            print(f"[WARN] {name}: no COM port configured")
            return

        conn     = None
        last_exc = None
        for attempt in range(1, CONNECT_RETRIES + 1):
            available = self._available_ports()
            if port not in available:
                print(f"[WARN] {name}:{port} not detected (attempt {attempt}/{CONNECT_RETRIES})")
                if attempt < CONNECT_RETRIES:
                    time.sleep(CONNECT_RETRY_DELAY)
                continue
            try:
                print(f"[{name}] opening {port} (attempt {attempt}/{CONNECT_RETRIES})...")
                conn = self._open_port_with_timeout(port, timeout_s=8.0)
                if self.settle_s > 0:
                    time.sleep(self.settle_s)
                conn.reset_input_buffer()
                last_exc = None
                break
            except serial.SerialException as exc:
                last_exc = exc
                print(f"[{name}] attempt {attempt} failed: {exc}")
                if conn:
                    try:
                        conn.close()
                    except Exception:
                        pass
                    conn = None
                if attempt < CONNECT_RETRIES:
                    delay = 4.0 if "Timed out" in str(exc) else CONNECT_RETRY_DELAY
                    print(f"[{name}] waiting {delay}s before retry...")
                    time.sleep(delay)

        with lock:
            if conn is not None:
                results[name] = conn
            else:
                results[name] = None
                msg = f"[WARN] Failed to open {name}:{port}"
                if last_exc:
                    msg += f" -> {last_exc}"
                print(msg)

    def connect_all(self):
        results = {}
        lock    = threading.Lock()
        for name, cfg in self.controller_cfg.items():
            print(f"[{name}] connecting sequentially...")
            self._connect_one(name, cfg, results, lock)
            time.sleep(1.0)

        self.offline_controllers = set()
        for name, conn in results.items():
            if conn is not None:
                self.serial_by_controller[name] = conn
                self.write_locks[name]          = threading.Lock()
                rt = threading.Thread(
                    target=self._read_loop, args=(name,), daemon=True
                )
                self.read_threads[name] = rt
                rt.start()
            else:
                self.offline_controllers.add(name)

        if not self.serial_by_controller:
            raise RuntimeError("No configured controller ports are currently connected/open.")

        self.broadcast("TON")

        print("\nConnected controllers:")
        for name, cfg in self.controller_cfg.items():
            if name in self.serial_by_controller:
                print(f"  - {name}: {cfg.get('port')}  motors={cfg.get('motors', [])}")
        if self.offline_controllers:
            missing = ", ".join(
                f"{n}:{self.controller_cfg.get(n,{}).get('port') or '-'}"
                for n in sorted(self.offline_controllers)
            )
            print(f"[WARN] Offline controllers: {missing}")

    # ------------------------------------------------------------------
    # Serial I/O
    # ------------------------------------------------------------------

    def _read_loop(self, controller_name):
        conn = self.serial_by_controller.get(controller_name)
        while self.running and conn and conn.is_open:
            try:
                line = conn.readline()
                if line:
                    text = line.decode("utf-8", errors="replace").rstrip()
                    sys.stdout.write(f"\r[{controller_name}] {text}\n")
                    sys.stdout.flush()
            except serial.SerialException:
                break
            except Exception:
                time.sleep(0.05)

    def send_to_controller(self, controller_name, cmd):
        conn = self.serial_by_controller.get(controller_name)
        if not conn or not conn.is_open:
            return
        lock = self.write_locks.get(controller_name)
        if lock is None:
            return
        try:
            with lock:
                conn.write((cmd + "\n").encode("utf-8"))
                conn.flush()
        except serial.SerialException as exc:
            print(f"Write failed ({controller_name}): {exc}")

    def broadcast(self, cmd):
        for name in sorted(self.serial_by_controller.keys()):
            self.send_to_controller(name, cmd)

    def send_hand(self, cmd):
        if self.hand_controller:
            self.send_to_controller(self.hand_controller, cmd)
        else:
            self.broadcast(cmd)

    # ------------------------------------------------------------------
    # Motor helpers
    # ------------------------------------------------------------------

    def _set_motor_rpm(self, motor_id, rpm):
        if rpm == self.current_cmd_rpm.get(motor_id):
            return
        self.current_cmd_rpm[motor_id] = rpm
        ctrl = self.motor_to_controller[motor_id]
        self.send_to_controller(ctrl, f"M {motor_id} {rpm}")

    def _apply_held_keys(self, held_keys):
        """Compute and send RPM for all motors based on currently held keys."""
        target = {mid: 0 for mid in self.motor_to_controller}
        for key in held_keys:
            for mid, sign in self.key_to_actions.get(key, []):
                target[mid] = sign * self.motor_speed[mid]
        for mid, rpm in target.items():
            self._set_motor_rpm(mid, rpm)

    # ------------------------------------------------------------------
    # Key event handlers  (same logic as original pynput on_press/on_release)
    # ------------------------------------------------------------------

    def on_press(self, key, held_keys):
        """Called once when a key transitions from up → down."""
        if key == self.quit_key:
            return False

        if key == self.stop_all_key:
            self.broadcast("S")
            for mid in self.current_cmd_rpm:
                self.current_cmd_rpm[mid] = 0
            held_keys.clear()
            sys.stdout.write("\rSTOP ALL                              \n")
            sys.stdout.flush()
            return True

        if key == self.blink_key:
            self.broadcast("BLINK")
            return True

        if key == self.hand_open_key:
            self.send_hand("HOPEN")
            sys.stdout.write("\r[hand] HOPEN                          \n")
            sys.stdout.flush()
            return True

        if key == self.hand_close_key:
            self.send_hand("HCLOSE")
            sys.stdout.write("\r[hand] HCLOSE                         \n")
            sys.stdout.flush()
            return True

        if key == self.hand_gentle_open_key:
            self.send_hand("HOPEN 128")
            sys.stdout.write("\r[hand] HOPEN 128 (gentle)             \n")
            sys.stdout.flush()
            return True

        if key == self.hand_gentle_close_key:
            self.send_hand("HCLOSE 128")
            sys.stdout.write("\r[hand] HCLOSE 128 (gentle)            \n")
            sys.stdout.flush()
            return True

        if key in self.key_to_actions:
            held_keys.add(key)
            self._apply_held_keys(held_keys)

        return True

    def on_release(self, key, held_keys):
        """Called once when a key transitions from down → up."""
        if key in (self.hand_open_key, self.hand_close_key,
                   self.hand_gentle_open_key, self.hand_gentle_close_key):
            self.send_hand("HSTOP")
            sys.stdout.write("\r[hand] HSTOP                          \n")
            sys.stdout.flush()
            return

        if key in held_keys:
            held_keys.discard(key)
            self._apply_held_keys(held_keys)

    # ------------------------------------------------------------------
    # Status bar  (redrawn every poll cycle at bottom of terminal)
    # ------------------------------------------------------------------

    def _redraw_status(self):
        parts = []
        for mid in sorted(self.motor_to_controller.keys()):
            rpm = self.current_cmd_rpm.get(mid, 0)
            parts.append(f"M{mid}:{rpm:+4d}")
        line = "  ".join(parts)
        sys.stdout.write(f"\r{line}   (ESC=quit  SPACE=stop all)")
        sys.stdout.flush()

    # ------------------------------------------------------------------
    # Print controls
    # ------------------------------------------------------------------

    def print_controls(self):
        print("\n=== Karura Arm — Multi Motor Keyboard Velocity Control ===")
        print("Hold a key to run the motor.  Release to stop.")
        print("")
        print(f"  Quit     : {self.quit_key}")
        print(f"  Stop all : {self.stop_all_key}")
        print(f"  Blink    : {self.blink_key}")
        hand_target = self.hand_controller or "all (broadcast)"
        print(f"  Hand open  (hold): {self.hand_open_key}   gentle: {self.hand_gentle_open_key}   -> {hand_target}")
        print(f"  Hand close (hold): {self.hand_close_key}   gentle: {self.hand_gentle_close_key}")
        print("")
        print("Per-motor bindings:")
        for mid in sorted(self.motor_to_controller.keys()):
            ctrl   = self.motor_to_controller[mid]
            status = "ONLINE" if ctrl in self.serial_by_controller else "OFFLINE"
            speed  = self.motor_speed[mid]
            fwd_key = rev_key = "?"
            for k, actions in self.key_to_actions.items():
                for am, sign in actions:
                    if am == mid:
                        if sign > 0:
                            fwd_key = k
                        else:
                            rev_key = k
            print(
                f"  M{mid} ({ctrl},{status}):  "
                f"forward={fwd_key}  reverse={rev_key}  speed={speed} rpm"
            )
        print("")
        print(f"Keymap file: {self.keymap_path}")
        print("Edit that file to remap keys/speeds, then restart this script.")
        print("==========================================================\n")

    # ------------------------------------------------------------------
    # Main loop
    # ------------------------------------------------------------------

    def run(self):
        self.print_controls()
        reader = RawKeyReader()

        # keys_down: keys currently held (transitions on_press/on_release)
        # key_timers: key -> cycles remaining before auto-release
        keys_down  = set()
        key_timers = {}

        try:
            while self.running:
                time.sleep(KEY_POLL_HZ)

                ch = reader.read_char()
                key = _norm_key_name(ch) if ch is not None else None

                # Key seen this cycle — refresh its timer
                if key:
                    key_timers[key] = RELEASE_CYCLES
                    if key not in keys_down:
                        keys_down.add(key)
                        result = self.on_press(key, keys_down)
                        if result is False:
                            break

                # Decrement timers for all keys not seen this cycle
                expired = []
                for k in list(key_timers):
                    if k == key:
                        continue          # refreshed above
                    key_timers[k] -= 1
                    if key_timers[k] <= 0:
                        expired.append(k)

                for k in expired:
                    del key_timers[k]
                    if k in keys_down:
                        keys_down.discard(k)
                        self.on_release(k, keys_down)

                self._redraw_status()

        except KeyboardInterrupt:
            pass
        finally:
            reader.restore()
            self.close()

    # ------------------------------------------------------------------
    # Cleanup
    # ------------------------------------------------------------------

    def close(self):
        self.running = False
        sys.stdout.write("\n")
        try:
            self.broadcast("S")
            self.send_hand("HSTOP")
            time.sleep(0.1)
        except Exception:
            pass
        for conn in self.serial_by_controller.values():
            try:
                if conn and conn.is_open:
                    conn.close()
            except Exception:
                pass
        print("Exited.")


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser(
        description="SSH multi-controller keyboard velocity control for Karura arm"
    )
    parser.add_argument("--config",  default="robot_config.json",  help="Path to robot_config.json")
    parser.add_argument("--keymap",  default=DEFAULT_KEYMAP_FILE,   help="Path to keymap JSON")
    parser.add_argument("--baud",    type=int,   default=DEFAULT_BAUD_RATE, help="Serial baud rate")
    parser.add_argument("--settle",  type=float, default=DEFAULT_SETTLE_S,
                        help="Seconds to wait after port open (default 0.5)")
    args = parser.parse_args()

    app = MultiMotorSSHController(args.config, args.keymap, args.baud, args.settle)

    try:
        app.load_robot_config()
        app.load_or_create_keymap()
        app.connect_all()
        app.run()
    except Exception:
        import traceback
        traceback.print_exc()
        sys.exit(1)


if __name__ == "__main__":
    main()
