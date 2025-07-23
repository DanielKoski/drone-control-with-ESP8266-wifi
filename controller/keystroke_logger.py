#!/usr/bin/env python3
"""
Real-time WASD + arrow keys + spacebar → Arduino serial controller
Includes:
  - Auto-detect Arduino port
  - Sends key commands immediately
  - Debug prints sent bytes
  - Reads and prints Arduino ACK responses
  - Exit on ESC
"""

import sys
import time
import serial
import serial.tools.list_ports
from pynput import keyboard

def find_arduino_port():
    """Scan connected serial ports and return the first likely Arduino device."""
    ports = serial.tools.list_ports.comports()
    for port in ports:
        name = port.device.lower()
        if ("usbmodem" in name) or ("usbserial" in name) or ("tty.usb" in name):
            return port.device
    return None

def main():
    # 1) Auto-detect port
    port = find_arduino_port()
    if not port:
        print("❌ Could not find an Arduino serial port. Is it plugged in?")
        sys.exit(1)
    print(f"🔌 Using serial port: {port}")

    # 2) Open serial connection
    try:
        ser = serial.Serial(port, 9600, timeout=0.1)
    except serial.SerialException as e:
        print(f"❌ Failed to open {port}: {e}")
        sys.exit(1)

    # Give Arduino time to reset
    time.sleep(2)
    print("✅ Serial connection opened. Press ESC to quit.")

    # 3) Define key handlers
    def on_press(key):
        cmd = None
        try:
            c = key.char.lower()
            if c == 'w':      cmd = b'W'
            elif c == 'a':    cmd = b'A'
            elif c == 's':    cmd = b'S'
            elif c == 'd':    cmd = b'D'
        except AttributeError:
            if key == keyboard.Key.up:      cmd = b'U'
            elif key == keyboard.Key.down:  cmd = b'DN'
            elif key == keyboard.Key.left:  cmd = b'L'
            elif key == keyboard.Key.right: cmd = b'R'
            elif key == keyboard.Key.space: cmd = b' '  # spacebar

        if cmd:
            ser.write(cmd)
            print(f"→ sent {cmd!r}")
            # read any ACK lines from Arduino
            time.sleep(0.01)
            while ser.in_waiting:
                line = ser.readline().decode(errors='ignore').strip()
                if line:
                    print(f"<– {line}")

    def on_release(key):
        if key == keyboard.Key.esc:
            # Stop listener
            return False

    # 4) Start listening
    with keyboard.Listener(on_press=on_press, on_release=on_release) as listener:
        listener.join()

    # 5) Clean up
    ser.close()
    print("🔒 Serial port closed. Goodbye!")

if __name__ == "__main__":
    main()
