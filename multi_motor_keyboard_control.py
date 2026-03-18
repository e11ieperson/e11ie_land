import argparse
import json
import os
import sys
import threading
import time

import serial
import serial.tools.list_ports

try:
    from pynput import keyboard
except ImportError:
    print("Error: pynput library not found.")
    print("Please install: pip install pynput pyserial")
    sys.exit(1)


DEFAULT_BAUD_RATE = 115200
DEFAULT_SPEED = 50
DEFAULT_KEYMAP_FILE = "multi_motor_keymap.json"
DEFAULT_SETTLE_S = 0.5   # was hardcoded 2.0 — override with --settle
CONNECT_RETRIES = 5
CONNECT_RETRY_DELAY = 0.5


def _norm_key_name(raw):
    if raw is None:
        return None
    s = str(raw).strip().lower()
    if not s:
        return None
    aliases = {
        "escape": "esc",
        "spacebar": "space",
        "return": "enter",
    }
    return aliases.get(s, s)


class MultiMotorKeyboardController:
    def __init__(self, config_path, keymap_path, baud_rate=DEFAULT_BAUD_RATE,
                 settle_s=DEFAULT_SETTLE_S):
        self.config_path = config_path
        self.keymap_path = keymap_path
        self.baud_rate = int(baud_rate)
        self.settle_s = float(settle_s)

        self.controller_cfg = {}
        self.motor_to_controller = {}
        self.serial_by_controller = {}
        self.offline_controllers = set()
        self.write_locks = {}
        self.read_threads = {}
        self.running = True

        self.key_to_actions = {}
        self.motor_speed = {}
        self.motor_key_state = {}
        self.current_cmd_rpm = {}
        self.quit_key = "esc"
        self.stop_all_key = "space"
        self.blink_key = "b"
        self.hand_open_key = "["
        self.hand_close_key = "]"
        self.hand_gentle_open_key = "{"
        self.hand_gentle_close_key = "}"
        self.hand_controller = None  # set by load_robot_config

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

        self.controller_cfg = {}
        self.motor_to_controller = {}
        self.hand_controller = None  # name of the controller that owns the hand
        for name, c in raw_ctrl.items():
            if not isinstance(c, dict):
                continue
            port = c.get("port")
            motors = c.get("motors", [])
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
                "port": str(port) if port else None,
                "motors": motor_ids,
                "hand": hand_cfg if isinstance(hand_cfg, dict) else {},
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
            raise ValueError("No valid controller entries found in robot_config.json")

        if self.hand_controller:
            print(f"[hand] routed to controller '{self.hand_controller}'")
        else:
            print("[hand] no controller has hand enabled — hand commands will broadcast")

    def _default_keymap(self):
        motors = sorted(self.motor_to_controller.keys())
        forward_pool = ["1", "2", "3", "4", "5", "6", "7", "8", "9", "0"]
        reverse_pool = ["q", "w", "e", "r", "t", "y", "u", "i", "o", "p"]

        motor_map = {}
        for idx, mid in enumerate(motors):
            if idx < len(forward_pool):
                fwd = forward_pool[idx]
                rev = reverse_pool[idx]
            else:
                fwd = f"f{mid}"
                rev = f"v{mid}"
            motor_map[str(mid)] = {
                "forward": fwd,
                "reverse": rev,
                "speed": DEFAULT_SPEED,
            }

        return {
            "quit": "esc",
            "stop_all": "space",
            "blink": "b",
            "hand_open": "[",
            "hand_close": "]",
            "hand_gentle_open": "{",
            "hand_gentle_close": "}",
            "motors": motor_map,
        }

    def load_or_create_keymap(self):
        if not os.path.exists(self.keymap_path):
            default_map = self._default_keymap()
            with open(self.keymap_path, "w", encoding="utf-8") as f:
                json.dump(default_map, f, indent=2)
            print(f"Created default keymap: {self.keymap_path}")

        with open(self.keymap_path, "r", encoding="utf-8") as f:
            keymap = json.load(f)

        self.quit_key = _norm_key_name(keymap.get("quit", "esc")) or "esc"
        self.stop_all_key = _norm_key_name(keymap.get("stop_all", "space")) or "space"
        self.blink_key = _norm_key_name(keymap.get("blink", "b")) or "b"
        self.hand_open_key = _norm_key_name(keymap.get("hand_open", "[")) or "["
        self.hand_close_key = _norm_key_name(keymap.get("hand_close", "]")) or "]"
        self.hand_gentle_open_key = _norm_key_name(keymap.get("hand_gentle_open", "{")) or "{"
        self.hand_gentle_close_key = _norm_key_name(keymap.get("hand_gentle_close", "}")) or "}"

        raw_motors = keymap.get("motors", {})
        if not isinstance(raw_motors, dict):
            raise ValueError("Keymap 'motors' must be an object")

        self.key_to_actions = {}
        self.motor_speed = {}
        self.motor_key_state = {}
        self.current_cmd_rpm = {}

        for mid, controller_name in sorted(self.motor_to_controller.items()):
            cfg = raw_motors.get(str(mid), {})
            if not isinstance(cfg, dict):
                cfg = {}
            fwd = _norm_key_name(cfg.get("forward"))
            rev = _norm_key_name(cfg.get("reverse"))
            if not fwd or not rev:
                raise ValueError(
                    f"Keymap missing forward/reverse keys for motor {mid}"
                )
            if fwd == rev:
                raise ValueError(
                    f"Motor {mid} forward/reverse keys cannot be the same"
                )

            try:
                speed = int(cfg.get("speed", DEFAULT_SPEED))
            except Exception:
                speed = DEFAULT_SPEED
            speed = max(1, min(2000, abs(speed)))

            self.motor_speed[mid] = speed
            self.motor_key_state[mid] = {"forward": False, "reverse": False}
            self.current_cmd_rpm[mid] = 0

            self.key_to_actions.setdefault(fwd, []).append((mid, +1))
            self.key_to_actions.setdefault(rev, []).append((mid, -1))

            if controller_name not in self.controller_cfg:
                raise ValueError(
                    f"Motor {mid} maps to unknown controller '{controller_name}'"
                )

    # ------------------------------------------------------------------
    # Connection — parallel with retry
    # ------------------------------------------------------------------

    def _available_ports(self):
        """Return a fresh set of available COM port device strings."""
        return {p.device for p in serial.tools.list_ports.comports()}

    def _open_port_with_timeout(self, port, timeout_s=8.0):
        """Open a serial port in a worker thread with a hard wall-clock
        timeout.  On Windows + RP2350 the Serial() constructor can block
        indefinitely if the USB-serial driver is in a bad state.

        When a timeout occurs we wait for the stuck thread to fully exit
        before returning — this ensures Windows has released the handle
        before the caller attempts a retry, avoiding the PermissionError
        that otherwise appears on every subsequent attempt."""
        conn_box = [None]
        exc_box = [None]

        def _open():
            try:
                conn_box[0] = serial.Serial(
                port,
                self.baud_rate,
                timeout=0.5,
                write_timeout=1.0,
                dsrdtr=False,
                rtscts=False,
                exclusive=True   
            )
            except Exception as e:
                exc_box[0] = e

        t = threading.Thread(target=_open, daemon=True)
        t.start()
        t.join(timeout=timeout_s)

        if t.is_alive():
            # The Serial() constructor is still blocking.  Wait up to
            # 10 more seconds for it to unblock on its own — on RP2350 the
            # driver eventually times out internally and the thread exits,
            # releasing the handle cleanly.  Only then raise the exception
            # so the caller can retry without hitting PermissionError.
            print(f"        [{port}] driver stalled, waiting for OS to release handle...")
            t.join(timeout=10.0)
            if t.is_alive():
                # Still stuck after 18s total — give up entirely on this port.
                raise serial.SerialException(
                    f"Timed out after {timeout_s + 10}s opening {port} "
                    f"(persistent driver hang — unplug/replug the board)"
                )
            # Thread finally exited — if it opened successfully use that conn,
            # otherwise treat it as a normal failure so the retry loop runs.
            if conn_box[0] is not None:
                return conn_box[0]
            raise serial.SerialException(
                f"Driver hang on {port} — handle now released, retrying"
            )
        if exc_box[0] is not None:
            raise exc_box[0]
        return conn_box[0]

    def _connect_one(self, name, cfg, results, lock):
        """Try to open a single controller port (run in a thread)."""
        port = cfg.get("port")
        if not port:
            with lock:
                results[name] = None
            print(f"[WARN] {name}: no COM port configured in robot_config.json")
            return

        conn = None
        last_exc = None
        for attempt in range(1, CONNECT_RETRIES + 1):
            # Re-check port availability each attempt so we catch
            # ports that only appear after a brief USB enumeration delay.
            available = self._available_ports()
            if port not in available:
                print(
                    f"[WARN] {name}:{port} not detected "
                    f"(attempt {attempt}/{CONNECT_RETRIES})"
                )
                if attempt < CONNECT_RETRIES:
                    time.sleep(CONNECT_RETRY_DELAY)
                continue

            try:
                print(f"[{name}] opening {port} (attempt {attempt}/{CONNECT_RETRIES})...")
                conn = self._open_port_with_timeout(port, timeout_s=8.0)
                # Only wait the settle time if the board genuinely needs
                # it (e.g. genuine Arduino Uno with auto-reset).
                # Set --settle 0 for boards that don't reset on DTR.
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
                    # After a driver hang the OS needs longer to release
                    # the port handle — use a longer delay than normal.
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
        lock = threading.Lock()
        results = {}
        lock = threading.Lock()

        for name, cfg in self.controller_cfg.items():
            print(f"[{name}] connecting sequentially...")
            self._connect_one(name, cfg, results, lock)

            # 👇 CRITICAL: give Windows time to settle USB stack
            time.sleep(1.0)

        self.offline_controllers = set()
        for name, conn in results.items():
            if conn is not None:
                self.serial_by_controller[name] = conn
                self.write_locks[name] = threading.Lock()
                rt = threading.Thread(
                    target=self._read_loop, args=(name,), daemon=True
                )
                self.read_threads[name] = rt
                rt.start()
            else:
                self.offline_controllers.add(name)

        if not self.serial_by_controller:
            raise RuntimeError(
                "No configured controller ports are currently connected/open."
            )

        self.broadcast("TON")

        print("Connected controllers:")
        for name, cfg in self.controller_cfg.items():
            if name in self.serial_by_controller:
                print(
                    f"  - {name}: {cfg.get('port')}  "
                    f"motors={cfg.get('motors', [])}"
                )
        if self.offline_controllers:
            missing_text = ", ".join(
                f"{n}:{self.controller_cfg.get(n, {}).get('port') or '-'}"
                for n in sorted(self.offline_controllers)
            )
            print(
                f"[WARN] Not all configured controllers are connected: "
                f"{missing_text}"
            )

    # ------------------------------------------------------------------
    # Serial I/O
    # ------------------------------------------------------------------

    def _read_loop(self, controller_name):
        """Drain incoming serial data.  Uses blocking readline so the
        thread sleeps at the OS level instead of spinning on in_waiting."""
        conn = self.serial_by_controller.get(controller_name)
        while self.running and conn and conn.is_open:
            try:
                # readline blocks up to timeout (0.5 s) then returns b''
                line = conn.readline()
                if line:
                    # Uncomment for debug: print(f"[{controller_name}] RX: {line!r}")
                    pass
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
        """Send a hand command only to the controller that owns the hand.
        Falls back to broadcast if no controller declared hand enabled."""
        if self.hand_controller:
            self.send_to_controller(self.hand_controller, cmd)
        else:
            self.broadcast(cmd)

    # ------------------------------------------------------------------
    # Key handling
    # ------------------------------------------------------------------

    def _key_name(self, key):
        try:
            if hasattr(key, "char") and key.char:
                return key.char.lower()
            if hasattr(key, "name") and key.name:
                return key.name.lower()
        except Exception:
            pass
        return None

    def _apply_motor_targets(self):
        for mid in sorted(self.motor_to_controller.keys()):
            state = self.motor_key_state[mid]
            rpm = 0
            if state["forward"] and not state["reverse"]:
                rpm = self.motor_speed[mid]
            elif state["reverse"] and not state["forward"]:
                rpm = -self.motor_speed[mid]

            if rpm == self.current_cmd_rpm[mid]:
                continue

            self.current_cmd_rpm[mid] = rpm
            controller_name = self.motor_to_controller[mid]
            cmd = f"M {mid} {rpm}"
            self.send_to_controller(controller_name, cmd)
            print(f"[{controller_name}] {cmd}")

    def on_press(self, key):
        name = self._key_name(key)
        if not name:
            return

        if name == self.quit_key:
            print("Quit key pressed.")
            return False  # stops listener

        if name == self.stop_all_key:
            print("STOP ALL")
            self.broadcast("S")
            for mid in self.current_cmd_rpm:
                self.current_cmd_rpm[mid] = 0
                self.motor_key_state[mid]["forward"] = False
                self.motor_key_state[mid]["reverse"] = False
            return

        if name == self.blink_key:
            self.broadcast("BLINK")
            return

        # Hand (DC motor gripper) commands
        if name == self.hand_open_key:
            self.send_hand("HOPEN")
            print("[hand] HOPEN")
            return

        if name == self.hand_close_key:
            self.send_hand("HCLOSE")
            print("[hand] HCLOSE")
            return

        if name == self.hand_gentle_open_key:
            self.send_hand("HOPEN 128")
            print("[hand] HOPEN 128 (gentle)")
            return

        if name == self.hand_gentle_close_key:
            self.send_hand("HCLOSE 128")
            print("[hand] HCLOSE 128 (gentle)")
            return

        actions = self.key_to_actions.get(name, [])
        changed = False
        for mid, sign in actions:
            slot = "forward" if sign > 0 else "reverse"
            if not self.motor_key_state[mid][slot]:
                self.motor_key_state[mid][slot] = True
                changed = True
        if changed:
            self._apply_motor_targets()

    def on_release(self, key):
        name = self._key_name(key)
        if not name:
            return

        # Hand keys are momentary: release sends HAND_STOP so the
        # gripper doesn't keep running after you lift the key.
        if name in (self.hand_open_key, self.hand_close_key,
                    self.hand_gentle_open_key, self.hand_gentle_close_key):
            self.send_hand("HSTOP")
            print("[hand] HSTOP")
            return

        actions = self.key_to_actions.get(name, [])
        changed = False
        for mid, sign in actions:
            slot = "forward" if sign > 0 else "reverse"
            if self.motor_key_state[mid][slot]:
                self.motor_key_state[mid][slot] = False
                changed = True
        if changed:
            self._apply_motor_targets()

    # ------------------------------------------------------------------
    # Display
    # ------------------------------------------------------------------

    def print_controls(self):
        print("\n=== Karura Arm — Multi Motor Keyboard Velocity Control ===")
        print("How to use:")
        print("  1) Keep firmware running on every controller board in robot_config.json.")
        print("  2) Start this script, then press/hold each mapped key to command motor RPM.")
        print("  3) Release the key to stop that motor (sends M <id> 0).")
        print("  4) Use Stop-All if anything behaves unexpectedly.")
        print("  5) Press Quit to exit (script sends stop before closing ports).")
        print("")
        print("Runtime controls:")
        print(
            f"  Quit: {self.quit_key}  |  Stop all: {self.stop_all_key}  |  "
            f"Blink: {self.blink_key}"
        )
        hand_target = self.hand_controller or "all (no hand controller set)"
        print(
            f"  Hand full:   open={self.hand_open_key} (hold)  close={self.hand_close_key} (hold)"
        )
        print(
            f"  Hand gentle: open={self.hand_gentle_open_key} (hold)  close={self.hand_gentle_close_key} (hold)"
            f"  routed to: {hand_target}"
        )
        print("")
        print("Per-motor bindings:")
        for mid in sorted(self.motor_to_controller.keys()):
            controller_name = self.motor_to_controller[mid]
            speed = self.motor_speed[mid]
            ctrl_state = (
                "ONLINE"
                if controller_name in self.serial_by_controller
                else "OFFLINE"
            )
            fwd_key = rev_key = None
            for k, actions in self.key_to_actions.items():
                for am, sign in actions:
                    if am == mid:
                        if sign > 0:
                            fwd_key = k
                        else:
                            rev_key = k
            print(
                f"  M{mid} ({controller_name},{ctrl_state}): "
                f"forward={fwd_key}  reverse={rev_key}  speed={speed} rpm"
            )
        print("")
        print(f"Keymap file : {self.keymap_path}")
        print("Edit that file to remap keys/speeds, then restart this script.")
        print("==========================================================\n")

    # ------------------------------------------------------------------
    # Cleanup
    # ------------------------------------------------------------------

    def close(self):
        self.running = False
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


# ------------------------------------------------------------------
# Entry point
# ------------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser(
        description="Multi-controller keyboard velocity control for Karura arm motors"
    )
    parser.add_argument(
        "--config", default="robot_config.json", help="Path to robot_config.json"
    )
    parser.add_argument(
        "--keymap",
        default=DEFAULT_KEYMAP_FILE,
        help="Path to keymap JSON (created if missing)",
    )
    parser.add_argument(
        "--baud", type=int, default=DEFAULT_BAUD_RATE, help="Serial baud rate"
    )
    parser.add_argument(
        "--settle",
        type=float,
        default=DEFAULT_SETTLE_S,
        help=(
            "Seconds to wait after opening each port for board to boot "
            "(default 0.5 s; set 0 for boards that don't auto-reset on connect)"
        ),
    )
    args = parser.parse_args()

    app = MultiMotorKeyboardController(
        args.config, args.keymap, args.baud, args.settle
    )

    try:
        app.load_robot_config()
        app.load_or_create_keymap()
        app.connect_all()
        app.print_controls()

        with keyboard.Listener(
            on_press=app.on_press, on_release=app.on_release
        ) as listener:
            listener.join()

    except KeyboardInterrupt:
        pass
    except Exception:
        import traceback
        traceback.print_exc()
        sys.exit(1)
    finally:
        app.close()
        print("Exited.")


if __name__ == "__main__":
    main()
