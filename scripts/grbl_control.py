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
    HOME_CONFIG = np.array([0.0, np.pi/2, 0.0, 0.0, -np.pi/2, 0.0])

    def __init__(self, port=None, baud=115200):
        self.lock = threading.Lock()
        self.connect(port, baud)
        
        # Kinematics
        self.kinematics = RobotKinematics("3Dprinted")
        
        # State
        self.status = "Unknown"
        self.mpos = np.zeros(5) # Machine Position in DEGREES
        self.wpos = np.zeros(5) # Work Position in DEGREES
        
        # Internal tracking since polling is disabled/flaky
        self.simulated_mpos = np.zeros(5) 
        
        # Start status poller
        self.running = True
        # self.poller = threading.Thread(target=self._status_loop)
        # self.poller.daemon = True
        # self.poller.start()
        
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
                # Polling frequency
                time.sleep(0.1)
                
                # Check if lock is free - don't block if main thread is sending
                if self.lock.acquire(blocking=False):
                    try:
                        self.ser.write(b"?")
                        # Read immediately
                        while True:
                            line = self.ser.readline().decode().strip()
                            if line.startswith('<'):
                                self._parse_status(line)
                                break
                            elif line == 'ok':
                                # Unexpected ok? ignore
                                pass
                            elif not line:
                                break
                    except:
                        pass
                    finally:
                        self.lock.release()

    def _parse_status(self, line):
        # Example: <Idle|MPos:0.000,0.000,0.000,0.000,0.000|FS:0,0>
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
            pass

    def send_command(self, cmd):
        print(f"DEBUG Sending: {cmd}")
        with self.lock:
            # self.ser.flushInput() # Removed
            self.ser.write((cmd + "\n").encode())
            
            start_t = time.time()
            buffer = ""
            while True:
                try:
                    # Read all available characters
                    if self.ser.in_waiting > 0:
                        chunk = self.ser.read(self.ser.in_waiting).decode(errors='ignore')
                        buffer += chunk
                    else:
                        time.sleep(0.01)
                except Exception as e:
                    print(f"Serial Error: {e}")
                
                # Process buffer line by line
                while '\n' in buffer:
                    line, buffer = buffer.split('\n', 1)
                    line = line.strip()
                    
                    if not line: continue
                    
                    print(f"DEBUG RX: '{line}'")
                    
                    # Handle Reset / Welcome Message
                    if "Grbl" in line and "['$' for help]" in line:
                         print("!!! WARNING: Arduino RESET detected !!!")
                         # We must unlock
                         time.sleep(0.1)
                         self.ser.write(b"$X\n")
                         continue

                    # Handle Alarm Lock Error
                    if line == "error:9":
                        print("!!! Locked (error:9). Sending $X to unlock...")
                        self.ser.write(b"$X\n")
                        # We don't return False yet, we hope the next 'ok' is for the command or the unlock
                        # But actually the original command is likely dead.
                        return False

                    if line == "ok":
                        return True
                    if line.startswith("error"):
                        print(f"GRBL Error: {line}")
                        return False
                    if line.startswith('<'):
                        self._parse_status(line)
                
                # Timeout
                if time.time() - start_t > 5.0:
                    print(f"TIMEOUT waiting for 'ok'. Buffer content: '{buffer}'")
                    return False

    def get_joint_angles(self):
        """Get current joint angles in Radians"""
        # GRBL MPos is Degrees relative to HOME
        # Use simulated mpos for now
        q_deg_rel = self.simulated_mpos # Degrees
        q_rad_rel = np.radians(q_deg_rel)
        
        # Absolute DH Angles = Home Config + Relative Motion
        # Assumes GRBL 0,0,0,0,0 corresponds to HOME_CONFIG
        q_abs = self.HOME_CONFIG.copy()
        q_abs[:5] += q_rad_rel
        
        return q_abs

    def move_joints_rel(self, joint_idx, delta_deg, speed=3000):
        """Move specific joint by delta degrees"""
        axis_chars = ['X', 'Y', 'Z', 'A', 'B']
        if joint_idx < 1 or joint_idx > 5:
            print("Invalid joint")
            return
            
        axis = axis_chars[joint_idx-1]
        
        # Just use Relative G-Code
        cmd_seq = [
            "G91", 
            f"G1 {axis}{delta_deg} F{speed}",
            "G90"
        ]
        
        success = True
        for cmd in cmd_seq:
            if not self.send_command(cmd):
                success = False
                break
        
        if success:
            self.simulated_mpos[joint_idx-1] += delta_deg
            print(f"Moved J{joint_idx} by {delta_deg} deg")

    def move_cartesian_rel(self, axis, val_mm, speed=3000):
        """Move task space relative (mm)"""
        # 1. Get current absolute DH angles
        q_curr = self.get_joint_angles()
        
        # 2. Get current Pose
        T_curr, _ = self.kinematics.forward_kinematics(q_curr)
        
        # 3. Target Pose
        T_target = T_curr.copy()
        idx = {'X':0, 'Y':1, 'Z':2}[axis]
        T_target[idx, 3] += (val_mm / 1000.0) # mm -> m
        
        print(f"Moving {axis} by {val_mm}mm")
        print(f"Current Pos: {T_curr[:3, 3]}")
        print(f"Target Pos:  {T_target[:3, 3]}")
        
        # 4. IK for Target Pose (Absolute Angles)
        q_new_abs, success = self.kinematics.numerical_inverse_kinematics(
            q_curr, T_target, max_iter=100
        )
        
        if not success:
            print("IK Failed!")
            return
            
        # 5. Convert Absolute DH Angles -> GRBL Target (Degrees rel to Home)
        # q_abs = Home + q_rel
        # q_rel = q_abs - Home
        q_rad_rel = q_new_abs[:5] - self.HOME_CONFIG[:5]
        q_deg_rel = np.degrees(q_rad_rel)
        
        print(f"DEBUG Absolute Angles: {np.degrees(q_new_abs[:5])}")
        print(f"DEBUG Target GRBL (Deg): {q_deg_rel}")
        
        # Send Absolute G1 command (G90 is default)
        cmd = f"G1 X{q_deg_rel[0]:.3f} Y{q_deg_rel[1]:.3f} Z{q_deg_rel[2]:.3f} A{q_deg_rel[3]:.3f} B{q_deg_rel[4]:.3f} F{speed}"
        if self.send_command(cmd):
             self.simulated_mpos = q_deg_rel
             print(f"Moved to new position.")
        
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
                    if self.send_command("G92 X0 Y0 Z0 A0 B0"):
                        self.simulated_mpos = np.zeros(5)
                    # Also reset internal state if needed, but polling updates it
                elif cmd == 'S':
                    print(f"Status: {self.status}")
                    print(f"Simulated MPos (Deg): {self.simulated_mpos}")
                    q = self.get_joint_angles()
                    T, _ = self.kinematics.forward_kinematics(q)
                    print(f"CPos (m):   {T[:3, 3]}")
                    print(f"Home Config: {np.degrees(self.HOME_CONFIG)}")
                elif cmd:
                    self.parse_command(cmd)
            except KeyboardInterrupt:
                break
            except Exception as e:
                print(f"Error: {e}")
                import traceback
                traceback.print_exc()
                
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
