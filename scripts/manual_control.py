#!/usr/bin/env python3
"""
Manual Robot Control - Focused Joint Movement
"""

import serial
import serial.tools.list_ports
import time
import numpy as np

from robot_kinematics import RobotKinematics

class ManualRobotControl:
    def __init__(self, port=None, baud=115200):
        # Connect to robot
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
        self.ser = serial.Serial(port, baud, timeout=1)
        time.sleep(2)
        print("Connected!")
        
        # Kinematics Setup
        self.kinematics = RobotKinematics("3Dprinted")
        
        # Calibration Data (Steps per 90 degrees)
        self.steps_90_deg = np.array([99000, 146000, 139500, 170000, 138000])
        self.steps_per_rad = self.steps_90_deg / (np.pi / 2.0)
        
        # Home Configuration (Rad) - corresponds to 0 steps
        # [J1, J2, J3, J4, J5, J6]
        self.home_config = np.array([0.0, np.pi/2, 0.0, 0.0, -np.pi/2, 0.0])
        
        # Current State
        self.current_steps = np.zeros(5, dtype=int)  # X, Y, Z, A, B (J1-J5) relative to start
        self.current_joint_angles = self.home_config.copy()
        
        self.last_command = None
        
    def wait_for_response(self):
        """Wait for OK from Arduino"""
        while True:
            if self.ser.in_waiting:
                line = self.ser.readline().decode().strip()
                if "OK" in line or "READY" in line:
                    break
            time.sleep(0.05)
    
    def send_gcode(self, cmd):
        """Send G-code command"""
        # print(f"Sending: {cmd}")
        self.ser.write((cmd + "\n").encode())
        self.wait_for_response()
        
    def steps_to_radians(self, steps):
        """Convert steps to joint angles (radians)"""
        # steps array is length 5
        # angles array is length 6
        
        # Copy home config
        angles = self.home_config.copy()
        
        # Calculate deltas for first 5 joints
        # Direction reversal for J3 and J5 (indices 2 and 4)
        # steps = delta_rad * steps_per_rad * sign
        # delta_rad = steps / (steps_per_rad * sign)
        
        signs = np.array([1, 1, -1, 1, -1])
        
        delta_rad = steps / (self.steps_per_rad * signs)
        angles[:5] += delta_rad
        
        return angles

    def radians_to_steps(self, angles):
        """Convert joint angles (radians) to steps"""
        # angles array is length 6
        
        # Calculate delta from home
        delta_rad = angles[:5] - self.home_config[:5]
        
        signs = np.array([1, 1, -1, 1, -1])
        steps = delta_rad * self.steps_per_rad * signs
        
        return steps.astype(int)

    def move_joint(self, joint, steps, speed=3000):
        """Move a specific joint by given steps"""
        self.current_steps[joint-1] += steps
        
        # Update joint angles tracking
        self.current_joint_angles = self.steps_to_radians(self.current_steps)
        
        # Send ALL joint positions
        cmd = f"G1 X{self.current_steps[0]} Y{self.current_steps[1]} Z{self.current_steps[2]} A{self.current_steps[3]} B{self.current_steps[4]} F{speed}"
        self.send_gcode(cmd)
        
        # Store last command for repeat
        self.last_command = ('joint', joint, steps, speed)
        
        direction = "+" if steps > 0 else "-"
        print(f"J{joint} {direction}{abs(steps)} -> Total: {self.current_steps[joint-1]}")
        
    def move_cartesian(self, axis, value_mm, speed=3000):
        """Move end effector in task space (value in mm)"""
        value_m = value_mm / 1000.0
        
        # 1. Get current pose
        T_curr, _ = self.kinematics.forward_kinematics(self.current_joint_angles)
        
        # 2. Calculate target pose
        T_target = T_curr.copy()
        if axis == 'X':
            T_target[0, 3] += value_m
        elif axis == 'Y':
            T_target[1, 3] += value_m
        elif axis == 'Z':
            T_target[2, 3] += value_m
            
        print(f"Moving {axis} by {value_mm}mm")
        print(f"Current Pos: {T_curr[:3, 3]}")
        print(f"Target Pos:  {T_target[:3, 3]}")
        
        # 3. Inverse Kinematics
        # Use numerical IK to find joint angles for target pose
        # We start search from current angles
        q_new, success = self.kinematics.numerical_inverse_kinematics(
            self.current_joint_angles, 
            T_target, 
            max_iter=100, 
            tol=1e-4
        )
        
        if not success:
            print("Error: Inverse Kinematics failed to find a solution!")
            return
            
        # 4. Convert to steps
        new_steps = self.radians_to_steps(q_new)
        
        # Check for large jumps (safety)
        diff = np.abs(new_steps - self.current_steps)
        if np.any(diff > 50000): # Arbitrary safety limit
            print(f"Safety Warning: Large joint movement detected! {diff}")
            if input("Continue? (y/n): ").lower() != 'y':
                return

        # 5. Execute
        self.current_steps = new_steps
        self.current_joint_angles = q_new # Update angles to the IK result (which might include J6 change)
        
        cmd = f"G1 X{self.current_steps[0]} Y{self.current_steps[1]} Z{self.current_steps[2]} A{self.current_steps[3]} B{self.current_steps[4]} F{speed}"
        self.send_gcode(cmd)
        
        self.last_command = ('cartesian', axis, value_mm, speed)
        print("Move Complete")

    def repeat_last(self):
        """Repeat the last movement command"""
        if self.last_command:
            type = self.last_command[0]
            if type == 'joint':
                _, joint, steps, speed = self.last_command
                self.move_joint(joint, steps, speed)
            elif type == 'cartesian':
                _, axis, value, speed = self.last_command
                self.move_cartesian(axis, value, speed)
        else:
            print("No previous command to repeat")
    
    def interactive_mode(self):
        """Main control loop"""
        print("\n=== Robot Manual Control ===")
        print("Joint Mode:     J<1-5> <+/-> <steps> [speed]")
        print("Cartesian Mode: <X/Y/Z> <+/-> <mm> [speed]")
        print("Examples: J1 + 500, Z + 10, X - 5 1500")
        print("Commands: h=home, s=status, r=repeat, q=quit")
        print()
        
        self.wait_for_response()  # Wait for Arduino READY
        
        while True:
            try:
                cmd = input("> ").strip()
                
                if cmd == 'q':
                    break
                elif cmd == 'h':
                    print("Homing...")
                    self.send_gcode("G28")
                    self.current_steps = np.zeros(5, dtype=int)
                    self.current_joint_angles = self.home_config.copy()
                    self.last_command = None
                    print("Homed")
                elif cmd == 's':
                    print(f"Steps: {self.current_steps}")
                    deg = np.rad2deg(self.current_joint_angles)
                    print(f"Angles (deg): {deg}")
                    T, _ = self.kinematics.forward_kinematics(self.current_joint_angles)
                    print(f"Cartesian (m): {T[:3, 3]}")
                elif cmd == 'r':
                    self.repeat_last()
                elif cmd:
                    self.parse_command(cmd)
                    
            except KeyboardInterrupt:
                print("\nExiting...")
                break
            except Exception as e:
                print(f"Error: {e}")
                import traceback
                traceback.print_exc()
    
    def parse_command(self, cmd):
        """Parse command"""
        parts = cmd.upper().split()
        if not parts: return
        
        # Check first part to determine mode
        first = parts[0]
        
        if first.startswith('J'):
            # Joint Mode
            if len(parts) < 3:
                print("Format: J<1-5> <+/-> <steps> [speed]")
                return
            
            try:
                joint = int(first[1:])
                if joint < 1 or joint > 5:
                    print("Invalid joint. Use J1-J5")
                    return
                
                direction = parts[1]
                val = int(parts[2])
                if direction == '-': val = -val
                
                speed = 3000
                if len(parts) > 3: speed = int(parts[3])
                
                self.move_joint(joint, val, speed)
            except ValueError:
                print("Invalid number format")
                
        elif first in ['X', 'Y', 'Z']:
            # Cartesian Mode
            if len(parts) < 3:
                print("Format: <X/Y/Z> <+/-> <mm> [speed]")
                return
                
            try:
                axis = first
                direction = parts[1]
                val = float(parts[2]) # mm
                if direction == '-': val = -val
                
                speed = 3000
                if len(parts) > 3: speed = int(parts[3])
                
                self.move_cartesian(axis, val, speed)
            except ValueError:
                print("Invalid number format")
        else:
             print("Unknown command")

if __name__ == "__main__":
    try:
        robot = ManualRobotControl()
        robot.interactive_mode()
    except Exception as e:
        print(f"Failed to start: {e}")
