#!/usr/bin/env python3
"""
Write Letters with 6-Axis Robot (CNC-based)
Uses CNCPlanner and CNCController for robust trajectory generation.
"""

import numpy as np
import time
import sys
import os
from scipy.spatial.transform import Rotation as R

# Add current directory to path
sys.path.append(os.path.dirname(os.path.abspath(__file__)))

from real_robot_interface import GrblRobotControl
from CNC import CNCPlanner, CNCController

class LetterWriter:
    def __init__(self, robot_control):
        self.robot = robot_control
        self.kinematics = self.robot.kinematics
        self.planner = CNCPlanner()
        self.controller = CNCController(self.kinematics)
        
        # Letter Definitions (Normalized 0-1)
        # Standard: X is width (left-right), Y is height (bottom-top)
        # NaN (represented as None) means pen lift
        self.letters = {
            'A': [[0,0], [0.5,1], [1,0], None, [0.25,0.5], [0.75,0.5]],
            'B': [[0,0], [0,1], [0.6,1], [0.6,0.5], [0,0.5], [0.6,0.5], [0.6,0], [0,0]],
            'C': [[1,0.2], [0.8,0], [0.2,0], [0,0.2], [0,0.8], [0.2,1], [0.8,1], [1,0.8]],
            'E': [[1,0], [0,0], [0,1], [1,1], None, [0,0.5], [0.6,0.5]],
            'H': [[0,0], [0,1], None, [1,0], [1,1], None, [0,0.5], [1,0.5]],
            'L': [[0,1], [0,0], [1,0]],
            'O': [[0.2,0], [0,0.2], [0,0.8], [0.2,1], [0.8,1], [1,0.8], [1,0.2], [0.8,0], [0.2,0]],
            ' ': [] 
        }

    def plan_text(self, text, scale=0.04, start_pos=None):
        """
        Plan the CNC moves for the text.
        """
        # Reset Planner
        self.planner = CNCPlanner()
        
        if start_pos is None:
            # Writing Config:
            # X: Forward distance from robot
            # Y: Left/Right (Writing direction)
            # Z: Height
            start_pos = np.array([0.20, 0.10, 0.05]) # 20cm forward, 10cm left, 5cm up
            
        print(f"Planning '{text}' starting at {start_pos}")
        
        # Orientation: Fixed Downward
        # Assuming Home is roughly [0, pi, pi] in Euler (XYZ)
        # Let's derive it from current/home pose to be safe
        T_home = self.kinematics.forward_kinematics(self.robot.HOME_CONFIG)
        # Force a clean vertical orientation if possible, or use Home orientation
        # Home pose R is roughly: X->X, Y->-Y, Z->-Z ?
        # Let's stick to Home Orientation for consistency
        R_home = T_home[:3, :3]
        rpy_home = R.from_matrix(R_home).as_euler('xyz')
        
        current_y = start_pos[1] # Start "Left"
        x_base = start_pos[0]    # Fixed Forward distance
        z_write = start_pos[2]
        z_safe = z_write + 0.015 # Lift 1.5cm
        
        # Speed settings
        v_rapid = 0.08 # m/s
        v_write = 0.02 # m/s
        accel = 0.2
        
        # Add initial move to Safe Start
        start_pt = np.array([x_base, current_y, z_safe])
        self.planner.add_move(start_pt, rpy_home, v_rapid, accel, 'exact_stop')
        
        for char in text.upper():
            if char == ' ':
                current_y -= 1.0 * scale # Move Right (Negative Y)
                continue
                
            if char not in self.letters:
                print(f"Skipping {char}")
                continue
            
            strokes = self.letters[char]
            pen_is_down = False
            
            for point in strokes:
                if point is None:
                    # Pen Lift
                    if pen_is_down:
                        # Lift at current XY
                        # We don't have current XY easily here without tracking, 
                        # but the next move logic handles the approach.
                        # Actually, we should explicit lift?
                        # No, the next point loop handles "If pen up, move to safe"
                        pen_is_down = False
                    continue
                
                # Coordinate Mapping for "Writing in front, Left to Right"
                # Letter X (Width) -> Robot -Y (Moving Right)
                # Letter Y (Height) -> Robot X (Moving Forward/Up)
                
                # Letter (0,0) is bottom-left.
                # Robot Frame: X+ is Forward, Y+ is Left.
                # Text should be readable standing at (0,0).
                # Letter Bottom -> X_base. Letter Top -> X_base + height.
                # Letter Left -> Y_current. Letter Right -> Y_current - width.
                
                # Map:
                # Robot X = x_base + point[1] * scale
                # Robot Y = current_y - point[0] * scale
                
                target_x = x_base + point[1] * scale
                target_y = current_y - point[0] * scale
                
                target_pos = np.array([target_x, target_y, z_write])
                target_safe = np.array([target_x, target_y, z_safe])
                
                if not pen_is_down:
                    # 1. Move to Target (Safe Height) - Rapid
                    self.planner.add_move(target_safe, rpy_home, v_rapid, accel, 'continuous')
                    # 2. Plunge to Write Height
                    self.planner.add_move(target_pos, rpy_home, v_write, accel, 'exact_stop')
                    pen_is_down = True
                else:
                    # Write to Target - Continuous
                    self.planner.add_move(target_pos, rpy_home, v_write, accel, 'continuous')
            
            # End of Letter - Lift
            # Last known pos was the last target_pos
            # We add a lift at the current location?
            # Or we just set pen_is_down=False, and next char will start with Safe Move.
            # But we should lift NOW to finish the letter cleanly.
            # Since we don't track "current" explicitly in loop, let's assume last point was written.
            # We can't easily add a relative move here without knowing exact coord.
            # But we know `target_safe` from last iteration corresponds to `target_pos`.
            # Let's just do a tiny lift or let the next letter handle it?
            # If we don't lift, the next rapid move will drag on the paper?
            # Wait, next letter starts with: `self.planner.add_move(target_safe...`
            # This moves from (Last_Write) to (Next_Safe).
            # This is a diagonal move up! This might mark the paper.
            # We should lift straight up.
            
            # Retrigger last X,Y calculation to lift
            # This is a bit redundant but safe
            if strokes and strokes[-1] is not None:
                last_pt = strokes[-1]
                lx = x_base + last_pt[1] * scale
                ly = current_y - last_pt[0] * scale
                lift_pos = np.array([lx, ly, z_safe])
                self.planner.add_move(lift_pos, rpy_home, v_rapid, accel, 'exact_stop')
            
            pen_is_down = False
            
            # Advance Cursor
            current_y -= 1.2 * scale

        # Return to Home Pose
        print("Adding return to Home...")
        home_pos = T_home[:3, 3]
        self.planner.add_move(home_pos, rpy_home, v_rapid, accel, 'exact_stop')

    def execute(self):
        # 1. Plan Trajectory (Differential IK)
        # Start from Robot Home Config
        print("Generating Control Trajectory...")
        start_q = self.robot.HOME_CONFIG # Or get_joint_angles()
        
        # Current FK
        start_T = self.kinematics.forward_kinematics(start_q)
        
        # Plan Segments
        segments = self.planner.plan(start_T, dt=0.04) # 25Hz
        
        if not segments:
            print("No segments generated!")
            return
            
        print(f"Generated {len(segments)} segments.")
        
        # Execute (Simulation/Generation)
        # controller.execute returns: 
        # (time_log, q_log, q_dot_log, vd_log, v_act_log, Td_log, segment_indices)
        results = self.controller.execute(segments, start_q)
        time_log, q_traj = results[0], results[1]
        
        print(f"Trajectory Size: {len(q_traj)} points, Duration: {time_log[-1]:.2f}s")
        
        # 2. Send to Real Robot
        input("Press Enter to STREAM to Robot...")
        self.robot.stream_trajectory(q_traj, time_log)

if __name__ == "__main__":
    try:
        # Connect
        robot_control = GrblRobotControl()
        
        # Homing
        print("Homing...")
        robot_control.send_command("G92 X0 Y0 Z0 A0 B0")
        robot_control.simulated_mpos = np.zeros(5)
        
        writer = LetterWriter(robot_control)
        
        while True:
            text = input("Enter text (q to quit): ").strip()
            if text.lower() == 'q': break
            
            writer.plan_text(text, scale=0.04)
            writer.execute()
            
    except Exception as e:
        print(f"Error: {e}")
