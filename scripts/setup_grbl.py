#!/usr/bin/env python3
"""
GRBL Setup Script for 6-Axis Robot
Configures EEPROM settings for the robot arm.
"""

import serial
import serial.tools.list_ports
import time
import sys

def find_port():
    ports = list(serial.tools.list_ports.comports())
    if not ports:
        return None
    
    # Try to find Arduino Mega
    for p in ports:
        if "Arduino" in p.description or "Mega" in p.description or "USB" in p.description:
            return p.device
            
    return ports[0].device

def send_command(ser, cmd):
    print(f"Sending: {cmd}")
    ser.write((cmd + "\n").encode())
    while True:
        line = ser.readline().decode().strip()
        if line:
            print(f"  Response: {line}")
        if "ok" in line:
            return True
        if "error" in line:
            print(f"  Error sending {cmd}!")
            return False

def setup_grbl():
    port = find_port()
    if not port:
        print("No serial port found.")
        sys.exit(1)
        
    print(f"Connecting to {port}...")
    try:
        ser = serial.Serial(port, 115200, timeout=1)
    except serial.SerialException as e:
        print(f"Error opening port: {e}")
        sys.exit(1)
        
    # Wait for wake up
    time.sleep(2)
    ser.reset_input_buffer()
    
    # Send Reset to be sure
    ser.write(b"\x18") # Ctrl-X
    time.sleep(1)
    while ser.in_waiting:
        print(ser.readline().decode().strip())

    print("\nConfiguring GRBL Settings...")
    
    # Calibration Values (Steps per 90 degrees)
    # J1: 99000, J2: 196000, J3: 141000, J4: 170000, J5: 147000
    steps_90 = [99000, 196000, 141000, 97000, 147000]
    steps_per_deg = [s / 90.0 for s in steps_90]
    
    settings = [
        # Steps/deg (using degrees as "mm")
        (f"$100={steps_per_deg[0]:.3f}", "X Steps/deg"),
        (f"$101={steps_per_deg[1]:.3f}", "Y Steps/deg"),
        (f"$102={steps_per_deg[2]:.3f}", "Z Steps/deg"),
        (f"$103={steps_per_deg[3]:.3f}", "A Steps/deg"),
        (f"$104={steps_per_deg[4]:.3f}", "B Steps/deg"),
        
        # Max Rate (deg/min) - Example: 1800 deg/min = 30 deg/sec
        ("$110=5000", "X Max Rate"),
        ("$111=5000", "Y Max Rate"),
        ("$112=5000", "Z Max Rate"),
        ("$113=5000", "A Max Rate"),
        ("$114=5000", "B Max Rate"),
        
        # Acceleration (deg/sec^2)
        ("$120=200", "X Accel"),
        ("$121=200", "Y Accel"),
        ("$122=200", "Z Accel"),
        ("$123=200", "A Accel"),
        ("$124=200", "B Accel"),
        
        # Invert Masks
        # Step Port Invert ($2): 0 (None)
        ("$2=0", "Step Port Invert"),
        # Dir Port Invert ($3): Invert Z(2) and B(4) -> Binary 10100 = 20
        # Validated from manual_control.py logic
        ("$3=20", "Dir Port Invert"),
        # Step Enable Invert ($4): 0 (Active Low for RAMPS)
        ("$4=0", "Enable Port Invert"),
        
        # Homing (Disable for now to prevent lockout)
        ("$22=0", "Homing Cycle Disable"),
        
        # Soft Limits (Disable until confident)
        ("$20=0", "Soft Limits Disable"),
        ("$21=0", "Hard Limits Disable"),
    ]
    
    for cmd, desc in settings:
        print(f"Setting {desc}...")
        send_command(ser, cmd)
        
    print("\nReading Settings:")
    send_command(ser, "$$")
    
    print("\nConfiguration Complete.")
    ser.close()

if __name__ == "__main__":
    setup_grbl()

