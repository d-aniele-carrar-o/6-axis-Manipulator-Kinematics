#!/usr/bin/env python3
"""
GRBL Robot Control
Control 6-axis robot using GRBL firmware (grbl-Mega-5X).
"""

import serial
import serial.tools.list_ports
import time
import numpy as np
import threading
from scipy.spatial.transform import Rotation as R

from robot_kinematics import RobotKinematics
from CNC import CNCPlanner, CNCController

class GrblRobotControl:
    HOME_CONFIG = np.array([0.0, np.pi/2, 0.0, 0.0, -np.pi/2, 0.0])
    # J1, J2, J3, J4, J5 Signs for GRBL (1 = Normal, -1 = Inverted)
    # Based on manual_control.py: J3 and J5 are inverted
    AXIS_SIGNS = np.array([1.0, -1.0, -1.0, 1.0, -1.0])

    def __init__(self, port=None, baud=115200):
        self.lock = threading.Lock()
        self.connect(port, baud)
        
        # Kinematics
        self.kinematics = RobotKinematics("3Dprinted")
        
        # CNC Controller for Trajectory Generation
        self.controller = CNCController(self.kinematics)
        
        # State
        self.status = "Unknown"
        self.mpos = np.zeros(5) # Machine Position in DEGREES
        self.wpos = np.zeros(5) # Work Position in DEGREES
        
        # Internal tracking since polling is disabled/flaky
        self.simulated_mpos = np.zeros(5) 
        
        # Start status poller
        self.running = True
        
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

    def send_command(self, cmd):
        # print(f"DEBUG Sending: {cmd}")
        with self.lock:
            self.ser.write((cmd + "\n").encode())
            
            start_t = time.time()
            buffer = ""
            while True:
                try:
                    if self.ser.in_waiting > 0:
                        chunk = self.ser.read(self.ser.in_waiting).decode(errors='ignore')
                        buffer += chunk
                    else:
                        time.sleep(0.01)
                except Exception as e:
                    print(f"Serial Error: {e}")
                
                while '\n' in buffer:
                    line, buffer = buffer.split('\n', 1)
                    line = line.strip()
                    
                    if not line: continue
                    # print(f"DEBUG RX: '{line}'")
                    
                    if "Grbl" in line and "['$' for help]" in line:
                         print("!!! WARNING: Arduino RESET detected !!!")
                         time.sleep(0.1)
                         self.ser.write(b"$X\n")
                         continue

                    if line == "error:9":
                        print("!!! Locked (error:9). Sending $X to unlock...")
                        self.ser.write(b"$X\n")
                        return False

                    if line == "ok":
                        return True
                    if line.startswith("error"):
                        print(f"GRBL Error: {line}")
                        return False
                    if line.startswith('<'):
                        self._parse_status(line)
                
                if time.time() - start_t > 5.0:
                    print(f"TIMEOUT waiting for 'ok'. Buffer content: '{buffer}'")
                    return False

    def _parse_status(self, line):
        try:
            content = line.strip('<>').split('|')
            self.status = content[0]
            for field in content[1:]:
                if field.startswith('MPos:'):
                    coords = field[5:].split(',')
                    self.mpos = np.array([float(c) for c in coords])
        except Exception as e:
            pass

    def get_joint_angles(self):
        """Get current joint angles in Radians"""
        # GRBL MPos is Degrees relative to HOME
        q_deg_grbl = self.simulated_mpos # Degrees in GRBL frame
        
        # Convert to Kinematic Frame (Apply Signs)
        q_deg_kin = q_deg_grbl * self.AXIS_SIGNS
        q_rad_rel = np.radians(q_deg_kin)
        
        # Absolute DH Angles = Home Config + Relative Motion
        q_abs = self.HOME_CONFIG.copy()
        q_abs[:5] += q_rad_rel
        
        return q_abs

    def move_joints_rel(self, joint_idx, delta_deg, speed=1750):
        """Move specific joint by delta degrees"""
        axis_chars = ['X', 'Y', 'Z', 'A', 'B']
        if joint_idx < 1 or joint_idx > 5:
            print("Invalid joint")
            return
            
        axis = axis_chars[joint_idx-1]
        
        # Apply Sign Inversion for GRBL
        delta_grbl = delta_deg * self.AXIS_SIGNS[joint_idx-1]
        
        cmd_seq = [
            "G91", 
            f"G1 {axis}{delta_grbl:.3f} F{speed}",
            "G90"
        ]
        
        success = True
        for cmd in cmd_seq:
            if not self.send_command(cmd):
                success = False
                break
        
        if success:
            self.simulated_mpos[joint_idx-1] += delta_grbl
            print(f"Moved J{joint_idx} by {delta_deg} deg (GRBL: {delta_grbl:.3f})")

    def move_cartesian_rel(self, axis, val_mm, speed=1750):
        """
        Move task space relative (mm) using CNC Planner for straight line trajectory.
        """
        # 1. Get current absolute DH angles
        q_curr = self.get_joint_angles()
        
        # 2. Get current Pose
        T_curr, _ = self.kinematics.forward_kinematics(q_curr)
        
        # 3. Target Pose
        T_target = T_curr.copy()
        idx = {'X':0, 'Y':1, 'Z':2}[axis]
        T_target[idx, 3] += (val_mm / 1000.0) # mm -> m
        
        print(f"Moving {axis} by {val_mm}mm (Linear Interpolated)")
        print(f"Current Pos: {T_curr[:3, 3]}")
        print(f"Target Pos:  {T_target[:3, 3]}")
        
        # 4. Use CNC Planner to generate straight line trajectory
        planner = CNCPlanner()
        
        # Convert Speed: GRBL F is deg/min (approx). We need m/s.
        # This is tricky because we are mixing units. 
        # User provides 'speed' likely in GRBL units (1750).
        # Let's assume a slow safe Cartesian speed.
        v_cartesian = 0.02 # 20mm/s
        a_cartesian = 0.1  # 100mm/s^2
        
        # Current Orientation (Preserve)
        rpy_curr = R.from_matrix(T_curr[:3, :3]).as_euler('xyz')
        
        planner.add_move(
            position=T_target[:3, 3],
            euler_rpy=rpy_curr,
            v_max=v_cartesian,
            a_max=a_cartesian,
            mode='exact_stop'
        )
        
        # 5. Plan and Execute
        print("Generating Linear Trajectory...")
        segments = planner.plan(T_curr, dt=0.04) # 25Hz
        
        # Run Control Loop (Differential IK)
        results = self.controller.execute(segments, q_curr)
        time_log, q_traj = results[0], results[1]
        
        print(f"Trajectory Size: {len(q_traj)} points. Duration: {time_log[-1]:.2f}s")
        
        # 6. Stream to Robot
        self.stream_trajectory(q_traj, time_log)

    def stream_trajectory(self, q_traj_rad, time_traj):
        """
        Stream a full trajectory (list of joint angles in radians) to GRBL.
        Uses Call-Response flow control.
        """
        if len(q_traj_rad) == 0: return True
        
        print(f"Starting Stream. Points: {len(q_traj_rad)}")
        total_points = len(q_traj_rad)
        start_time = time.time()
        
        for i in range(total_points):
            q_rad_abs = q_traj_rad[i]
            
            # 2. Convert to GRBL Degrees (Relative to Home)
            q_rad_rel = q_rad_abs[:5] - self.HOME_CONFIG[:5]
            q_deg_rel_kin = np.degrees(q_rad_rel)
            
            # Apply Signs
            q_deg_rel_grbl = q_deg_rel_kin * self.AXIS_SIGNS
            
            # 3. Calculate Feedrate (Speed)
            # F is deg/min.
            if i < total_points - 1:
                dt = time_traj[i+1] - time_traj[i]
                if dt < 1e-4: dt = 0.01
                
                # Next point for speed calc
                q_next_rad = q_traj_rad[i+1][:5] - self.HOME_CONFIG[:5]
                q_next_kin = np.degrees(q_next_rad)
                q_next_grbl = q_next_kin * self.AXIS_SIGNS
                
                # Max joint displacement
                max_delta = np.max(np.abs(q_next_grbl - q_deg_rel_grbl))
                
                speed_deg_s = max_delta / dt
                feedrate = speed_deg_s * 60.0 
                
                if feedrate < 10: feedrate = 10
            else:
                feedrate = 1000 # Last point
                
            # 4. Construct G-Code
            cmd = f"G1 X{q_deg_rel_grbl[0]:.3f} Y{q_deg_rel_grbl[1]:.3f} Z{q_deg_rel_grbl[2]:.3f} A{q_deg_rel_grbl[3]:.3f} B{q_deg_rel_grbl[4]:.3f} F{feedrate:.1f}"
            
            # 5. Send
            if not self.send_command(cmd):
                print(f"Stream Failed at point {i}")
                return False
                
            # Update internal state
            self.simulated_mpos = q_deg_rel_grbl
            
        duration = time.time() - start_time
        print(f"Stream Complete. Took {duration:.2f}s for {time_traj[-1]:.2f}s trajectory.")
        return True

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
                elif cmd == 'S':
                    print(f"Status: {self.status}")
                    print(f"Simulated MPos (Deg): {self.simulated_mpos}")
                    q = self.get_joint_angles()
                    T, _ = self.kinematics.forward_kinematics(q)
                    print(f"CPos (m):   {T[:3, 3]}")
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
                speed = 1750
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
                speed = 1750
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
