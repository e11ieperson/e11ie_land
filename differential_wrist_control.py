import serial
import serial.tools.list_ports
import time
import sys
import threading

try:
    from pynput import keyboard
except ImportError:
    print("Error: pynput library not found.")
    print("Please install it running: pip install pynput pyserial")
    sys.exit(1)

# Configuration
BAUD_RATE = 115200
MOTOR_1_ID = 5
MOTOR_2_ID = 6
SPEED = 50

ser = None

# Key states for WASD
keys = {
    'w': False,
    's': False,
    'a': False,
    'd': False
}

current_m1 = 0
current_m2 = 0

def find_serial_port():
    ports = serial.tools.list_ports.comports()
    if not ports:
        return None
    for port in ports:
        # Match typical Arduino/CH340 serial chip descriptions
        if "CH340" in port.description or "Arduino" in port.description or "Serial" in port.description:
            return port.device
    return ports[0].device

def send_command(cmd):
    if ser and ser.is_open:
        try:
            ser.write((cmd + '\n').encode('utf-8'))
        except Exception as e:
            pass

def update_motors():
    global current_m1, current_m2
    
    # Calculate desired abstract motion
    pitch = 0  # +1 for up, -1 for down
    if keys['w']: pitch += 1
    if keys['s']: pitch -= 1
        
    roll = 0   # +1 for right, -1 for left
    if keys['d']: roll += 1
    if keys['a']: roll -= 1
        
    # Differential kinematics mixing
    # Pitch moves both motors in the same direction
    # Roll moves them in opposite directions
    m1_target = (pitch + roll) * SPEED
    m2_target = (pitch - roll) * SPEED
    
    # Cap speeds if both pitch and roll are commanded simultaneously
    if m1_target > SPEED: m1_target = SPEED
    if m1_target < -SPEED: m1_target = -SPEED
    if m2_target > SPEED: m2_target = SPEED
    if m2_target < -SPEED: m2_target = -SPEED

    # Only send commands if the target speed has changed
    if m1_target != current_m1 or m2_target != current_m2:
        current_m1 = m1_target
        current_m2 = m2_target
        
        # Batch both motor commands into one serial write (no inter-command delay needed)
        send_command(f"M {MOTOR_1_ID} {current_m1}; M {MOTOR_2_ID} {current_m2}")
        
        if current_m1 == 0 and current_m2 == 0:
            print("Stopping Motors...")
        else:
            print(f"Motor 1: {current_m1:>4} RPM | Motor 2: {current_m2:>4} RPM")

def on_press(key):
    try:
        if hasattr(key, 'char') and key.char:
            char = key.char.lower()
            if char in keys:
                if not keys[char]: # Only update if state changed
                    keys[char] = True
                    update_motors()
            elif char == 'q':
                print("Quitting...")
                return False # Stop listener
    except AttributeError:
        pass

def on_release(key):
    try:
        if hasattr(key, 'char') and key.char:
            char = key.char.lower()
            if char in keys:
                if keys[char]: # Only update if state changed
                    keys[char] = False
                    update_motors()
    except AttributeError:
        pass

def read_from_port():
    while ser and ser.is_open:
        try:
            if ser.in_waiting > 0:
                line = ser.readline().decode('utf-8', errors='ignore').strip()
                # Uncomment the line below to view Arduino debug output
                # if line: print(f"[Arduino] {line}")
                pass
            time.sleep(0.01)
        except Exception:
            break

def main():
    global ser
    print("Looking for Arduino serial port...")
    port_name = "COM13" #find_serial_port()
    
    if not port_name:
        print("No serial ports found! Please plug in your device.")
        return

    print(f"Connecting to {port_name}...")
    try:
        ser = serial.Serial(port_name, BAUD_RATE, timeout=0.1)
    except Exception as e:
        print(f"Failed to connect to {port_name}: {e}")
        return

    time.sleep(2) # Give Arduino some time to reset upon connection
    print("Connected successfully!")
    print("\n--- Differential Wrist Controls ---")
    print(" W / S : Pitch Up / Down (Both motors spin same direction)")
    print(" A / D : Roll Left / Right (Motors spin opposite directions)")
    print(" Press 'Q' to quit")
    print("-----------------------------------\n")

    # Start a thread to read incoming messages from Arduino
    read_thread = threading.Thread(target=read_from_port, daemon=True)
    read_thread.start()

    # Start keyboard listener
    with keyboard.Listener(on_press=on_press, on_release=on_release) as listener:
        listener.join()

    # Cleanup
    if ser and ser.is_open:
        send_command("S") # Stop everything before closing
        time.sleep(0.1)
        ser.close()
    print("Exited.")

if __name__ == "__main__":
    main()
