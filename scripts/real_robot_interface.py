#!/usr/bin/env python3
"""
Real Robot Control Interface
Control 6-axis robot using GRBL firmware (grbl-Mega-5X).
"""

import serial
import serial.tools.list_ports
import time
import numpy as np
import threading
import sys
from scipy.spatial.transform import Rotation as R

from robot_kinematics import RobotKinematics
from CNC import CNCPlanner, CNCController


class GrblRobotControl:
    HOME_CONFIG = np.array([0.0, np.pi/2, 0.0, 0.0, -np.pi/2, 0.0])
    # J1, J2, J3, J4, J5 Signs for SOFTWARE inversion
    # J2, J3, J5 need software inversion based on user calibration
    AXIS_SIGNS = np.array([1.0, -1.0, -1.0, 1.0, -1.0])

    def __init__(self, port=None, baud=115200, debug=False):
        self.debug = debug
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
        if self.debug:
            print(f"  TX: {cmd}")
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
                    if self.debug:
                        print(f"  RX: {line}")
                    
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
                        print(f"GRBL Error: {line} (for cmd: {cmd})")
                        return False
                    if line.startswith('<'):
                        self._parse_status(line)
                
                if time.time() - start_t > 5.0:
                    print(f"TIMEOUT waiting for 'ok'. Buffer: '{buffer}'")
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

    def query_settings(self):
        """Query and display GRBL settings"""
        with self.lock:
            self.ser.write(b"$$\n")
            time.sleep(0.5)
            
            while self.ser.in_waiting:
                line = self.ser.readline().decode(errors='ignore').strip()
                if line:
                    print(f"  {line}")

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
            print("Invalid joint (use 1-5)")
            return
            
        axis = axis_chars[joint_idx-1]
        
        # Apply Sign Inversion for GRBL
        delta_grbl = delta_deg * self.AXIS_SIGNS[joint_idx-1]
        
        # Use Relative Mode (G91) for jogging
        # This is more robust than absolute mode if tracked position is off
        delta_str = f"{delta_grbl:.3f}"
        gcode = f"G91 G1 {axis}{delta_str} F{speed}"
        print(f"J{joint_idx} (Jog): Sending '{gcode}'")
        
        # Send command
        success = self.send_command(gcode)
        
        # Always restore absolute mode (G90)
        self.send_command("G90")
        
        if success:
            # Update internal tracking
            self.simulated_mpos[joint_idx-1] += delta_grbl
            print(f"  -> OK: J{joint_idx} moved by {delta_deg} deg (Tracked: {self.simulated_mpos[joint_idx-1]:.3f})")
        else:
            print(f"  -> FAILED: J{joint_idx} command not executed!")

    def move_cartesian_rel(self, axis, val_mm, speed=1750):
        """
        Move task space relative (mm) using CNC Planner for straight line trajectory.
        """
        # 1. Get current absolute DH angles
        q_curr = self.get_joint_angles()
        
        # 2. Get current Pose
        T_curr = self.kinematics.forward_kinematics(q_curr)
        
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
        segments = planner.plan(T_curr, dt=0.02) # Finer dt
        
        # Fresh Controller
        controller = CNCController(self.kinematics)
        
        # Run Control Loop (Differential IK)
        try:
            results = controller.execute(segments, q_curr)
        except KeyboardInterrupt:
            print("Generation cancelled.")
            return

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
            if i % 10 == 0:
                sys.stdout.write(f"Streaming {i}/{total_points}...\r")
                sys.stdout.flush()

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
            # Format values to match GRBL resolution exactly
            coords = [f"{v:.3f}" for v in q_deg_rel_grbl]
            target_vals = np.array([float(c) for c in coords])
            
            cmd = f"G1 X{coords[0]} Y{coords[1]} Z{coords[2]} A{coords[3]} B{coords[4]} F{feedrate:.1f}"
            
            # 5. Send
            if not self.send_command(cmd):
                print(f"Stream Failed at point {i}")
                return False
                
            # Update internal state to match exact sent values
            self.simulated_mpos = target_vals
            
        duration = time.time() - start_time
        print(f"Stream Complete. Took {duration:.2f}s for {time_traj[-1]:.2f}s trajectory.")
        return True

    def interactive_mode(self):
        print("\n=== GRBL Robot Control ===")
        print("Joint Mode:     J<1-5> <+/-> <deg> [speed]")
        print("Cartesian Mode: <X/Y/Z> <+/-> <mm> [speed]")
        print("Commands: h=set home, s=status, d=toggle debug, q=quit")
        print(f"AXIS_SIGNS (software): {self.AXIS_SIGNS}")
        print(f"  J1->X, J2->Y, J3->Z, J4->A, J5->B")
        print(f"  (Hardware inverts Z,B via $3=20)")
        
        while True:
            try:
                cmd = input("> ").strip().upper()
                
                if cmd == 'Q':
                    break
                elif cmd == 'D':
                    self.debug = not self.debug
                    print(f"Debug mode: {'ON' if self.debug else 'OFF'}")
                elif cmd == 'H':
                    print("Setting Home (G92)...")
                    if self.send_command("G92 X0 Y0 Z0 A0 B0"):
                        self.simulated_mpos = np.zeros(5)
                elif cmd == 'S':
                    print(f"Status: {self.status}")
                    print(f"Simulated MPos (Deg): {self.simulated_mpos}")
                    q = self.get_joint_angles()
                    T = self.kinematics.forward_kinematics(q)
                    print(f"CPos (m):   {T[:3, 3]}")
                elif cmd == '$$':
                    # Query GRBL settings
                    print("Querying GRBL settings...")
                    self.query_settings()
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
                # Flexible parsing: J1 10, J1 + 10, J1 -10
                joint_str = first[1:]
                if not joint_str: raise ValueError("Joint number must follow J (e.g. J1)")
                joint = int(joint_str)
                
                # Default values
                val = 0.0
                speed = 1750
                
                if len(parts) >= 2:
                    remaining = parts[1:]
                    
                    # Handle optional direction sign as separate token
                    if remaining[0] in ['+', '-']:
                        sign = remaining.pop(0)
                        if not remaining: raise ValueError("Missing value")
                        val = float(remaining.pop(0))
                        if sign == '-': val = -val
                    else:
                        val = float(remaining.pop(0))
                        
                    # Speed
                    if remaining:
                        speed = float(remaining[0])
                else:
                    raise ValueError("Missing value")

                self.move_joints_rel(joint, val, speed)
            except Exception as e:
                print(f"Invalid Joint Cmd: {e}")
                
        elif first in ['X', 'Y', 'Z']:
            try:
                axis = first
                
                # Default values
                val = 0.0
                speed = 1750
                
                if len(parts) >= 2:
                    remaining = parts[1:]
                    
                    # Handle optional direction sign as separate token
                    if remaining[0] in ['+', '-']:
                        sign = remaining.pop(0)
                        if not remaining: raise ValueError("Missing value")
                        val = float(remaining.pop(0))
                        if sign == '-': val = -val
                    else:
                        val = float(remaining.pop(0))
                        
                    # Speed
                    if remaining:
                        speed = float(remaining[0])
                else:
                    raise ValueError("Missing value")

                self.move_cartesian_rel(axis, val, speed)
            except Exception as e:
                print(f"Invalid Cart Cmd: {e}")


if __name__ == "__main__":
    try:
        robot = GrblRobotControl()
        robot.interactive_mode()
    except Exception as e:
        print(f"Failed: {e}")


