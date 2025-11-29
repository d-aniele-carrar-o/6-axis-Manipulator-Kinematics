#!/usr/bin/env python3
"""
GRBL Robot Control
Control 6-axis robot using GRBL firmware (grbl-Mega-5X).
"""

import serial
import serial.tools.list_ports
import time
import numpy as np
import sys
import os
import threading
import re

# Ensure we can import RobotKinematics
sys.path.append(os.path.dirname(os.path.abspath(__file__)))
from robot_kinematics import RobotKinematics

class GrblRobotControl:
    def __init__(self, port=None, baud=115200):
        self.connect(port, baud)
        
        # Kinematics
        self.kinematics = RobotKinematics("3Dprinted")
        
        # State
        self.status = "Unknown"
        self.mpos = np.zeros(5) # Machine Position in DEGREES
        self.wpos = np.zeros(5) # Work Position in DEGREES
        
        # Start status poller
        self.running = True
        self.poller = threading.Thread(target=self._status_loop)
        self.poller.daemon = True
        self.poller.start()
        
    def connect(self, port, baud):
        if port is None:
            ports = list(serial.tools.list_ports.comports())
            if not ports:
                raise Exception("No serial ports found!")
            print("Available ports:")
            for i, p in enumerate(ports):
                print(f" {i}: {p.device}")
            
            if len(ports) == 1:
                port = ports[0].device
            else:
                try:
                    choice = int(input(f"Select port (0-{len(ports)-1}): "))
                    port = ports[choice].device
                except ValueError:
                    port = ports[0].device
        
        print(f"Connecting to {port}...")
        self.ser = serial.Serial(port, baud, timeout=0.1)
        
        # Wake up GRBL
        self.ser.write(b"\r\n\r\n")
        time.sleep(2)
        self.ser.flushInput()
        print("Connected to GRBL!")
        
        # Unlock if alarm
        self.send_command("$X")

    def _status_loop(self):
        while self.running:
            if self.ser.is_open:
                try:
                    self.ser.write(b"?")
                    while True:
                        line = self.ser.readline().decode().strip()
                        if line.startswith('<'):
                            self._parse_status(line)
                            break
                        if not line:
                            break
                except:
                    pass
            time.sleep(0.1)

    def _parse_status(self, line):
        # Example: <Idle|MPos:0.000,0.000,0.000,0.000,0.000|FS:0,0>
        # Or: <Idle|MPos:0.000,0.000,0.000,0.000,0.000|Bf:15,128|FS:0,0>
        try:
            content = line.strip('<>').split('|')
            self.status = content[0]
            
            for field in content[1:]:
                if field.startswith('MPos:'):
                    coords = field[5:].split(',')
                    self.mpos = np.array([float(c) for c in coords])
                elif field.startswith('WPos:'):
                    coords = field[5:].split(',')
                    self.wpos = np.array([float(c) for c in coords])
        except Exception as e:
            # print(f"Parse error: {e}")
            pass

    def send_command(self, cmd):
        # print(f"Sending: {cmd}")
        self.ser.write((cmd + "\n").encode())
        while True:
            line = self.ser.readline().decode().strip()
            if line == "ok":
                return True
            if line.startswith("error"):
                print(f"GRBL Error: {line}")
                return False
            # Ignore status reports in command stream
            if line.startswith('<'):
                self._parse_status(line)

    def get_joint_angles(self):
        """Get current joint angles in Radians"""
        # GRBL is configured in Degrees
        # MPos contains [X, Y, Z, A, B] in Degrees
        
        # Convert Deg -> Rad
        q_deg = self.mpos
        q_rad = np.radians(q_deg)
        
        # Add J6 (0.0) as GRBL only controls 5 axes
        return np.concatenate([q_rad, [0.0]])

    def move_joints_rel(self, joint_idx, delta_deg, speed=3000):
        """Move specific joint by delta degrees"""
        # GRBL uses G91 for relative mode
        axis_chars = ['X', 'Y', 'Z', 'A', 'B']
        if joint_idx < 1 or joint_idx > 5:
            print("Invalid joint")
            return
            
        axis = axis_chars[joint_idx-1]
        
        cmd_seq = [
            "G91", # Relative
            f"G1 {axis}{delta_deg} F{speed}",
            "G90"  # Back to Absolute
        ]
        
        for cmd in cmd_seq:
            self.send_command(cmd)
            
        print(f"Moved J{joint_idx} by {delta_deg} deg")

    def move_cartesian_rel(self, axis, val_mm, speed=3000):
        """Move task space relative (mm)"""
        # 1. Get current pose
        q_curr = self.get_joint_angles()
        T_curr, _ = self.kinematics.forward_kinematics(q_curr)
        
        # 2. Target Pose
        T_target = T_curr.copy()
        idx = {'X':0, 'Y':1, 'Z':2}[axis]
        T_target[idx, 3] += (val_mm / 1000.0) # mm -> m
        
        print(f"Moving {axis} by {val_mm}mm")
        print(f"Current Pos: {T_curr[:3, 3]}")
        print(f"Target Pos:  {T_target[:3, 3]}")
        
        # 3. IK
        q_new, success = self.kinematics.numerical_inverse_kinematics(
            q_curr, T_target, max_iter=100
        )
        
        if not success:
            print("IK Failed!")
            return
            
        # 4. Move
        # Convert Rad -> Deg
        q_deg = np.degrees(q_new[:5])
        
        cmd = f"G1 X{q_deg[0]:.3f} Y{q_deg[1]:.3f} Z{q_deg[2]:.3f} A{q_deg[3]:.3f} B{q_deg[4]:.3f} F{speed}"
        self.send_command(cmd)
        
    def interactive_mode(self):
        print("\n=== GRBL Robot Control ===")
        print("Joint Mode:     J<1-5> <+/-> <deg> [speed]")
        print("Cartesian Mode: <X/Y/Z> <+/-> <mm> [speed]")
        print("Commands: h=set home, s=status, q=quit")
        
        while True:
            try:
                cmd = input("> ").strip().upper()
                
                if cmd == 'Q':
                    break
                elif cmd == 'H':
                    print("Setting Home (G92)...")
                    self.send_command("G92 X0 Y0 Z0 A0 B0")
                elif cmd == 'S':
                    print(f"Status: {self.status}")
                    print(f"MPos (Deg): {self.mpos}")
                    q = self.get_joint_angles()
                    T, _ = self.kinematics.forward_kinematics(q)
                    print(f"CPos (m):   {T[:3, 3]}")
                elif cmd:
                    self.parse_command(cmd)
            except KeyboardInterrupt:
                break
            except Exception as e:
                print(f"Error: {e}")
                
        self.running = False
        self.ser.close()

    def parse_command(self, cmd):
        parts = cmd.split()
        if not parts: return
        
        first = parts[0]
        
        if first.startswith('J'):
            try:
                joint = int(first[1:])
                direction = parts[1]
                val = float(parts[2])
                if direction == '-': val = -val
                
                speed = 3000
                if len(parts) > 3: speed = float(parts[3])
                
                self.move_joints_rel(joint, val, speed)
            except:
                print("Invalid Joint Cmd")
                
        elif first in ['X', 'Y', 'Z']:
            try:
                axis = first
                direction = parts[1]
                val = float(parts[2])
                if direction == '-': val = -val
                
                speed = 3000
                if len(parts) > 3: speed = float(parts[3])
                
                self.move_cartesian_rel(axis, val, speed)
            except:
                print("Invalid Cart Cmd")

if __name__ == "__main__":
    try:
        robot = GrblRobotControl()
        robot.interactive_mode()
    except Exception as e:
        print(f"Failed: {e}")

