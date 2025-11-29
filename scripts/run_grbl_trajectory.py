#!/usr/bin/env python3
"""
Run Rectangle Trajectory on GRBL Robot
"""

import numpy as np
import sys
import os
from numpy import pi

# Imports
sys.path.append(os.path.dirname(os.path.abspath(__file__)))
from robot_kinematics import RobotKinematics
from CNC import CNCPlanner, CNCController
from grbl_control import GrblRobotControl

def run_rectangle_test():
    # 1. Setup
    robot = RobotKinematics("3Dprinted", home_config=np.array([0.0, pi/2, 0.0, 0.0, -np.pi/2, 0.0]))
    planner = CNCPlanner()
    
    # 2. Define Rectangle Trajectory (Smaller/Slower for testing)
    # Start slightly offset from home
    # Home Pos (Approx): X=0.187, Y=0, Z=0.162 (varies)
    
    # Let's get actual home pose
    T_home, _ = robot.forward_kinematics(robot.home_config)
    start_pos = T_home[:3, 3]
    print(f"Home Position: {start_pos}")
    
    # Define a larger rectangle in YZ plane
    # Center approx at (0.187, 0, 0.162)
    # Go +/- 5cm in Y
    
    # Define Corners relative to Start Position (Home)
    # 1. Start (Home)
    # 2. Left (Y+)
    # 3. Down (Z-)
    # 4. Right (Y-)
    # 5. Up (Z+)
    # 6. Center (Home)
    
    width = 0.05 # +/- 5cm
    height = -0.15 # 5cm down
    
    p1 = start_pos
    p2 = start_pos + np.array([0, width, 0])          # Left
    p3 = start_pos + np.array([0, width, height])  # Down
    p4 = start_pos + np.array([0, -width, height]) # Right across
    p5 = start_pos + np.array([0, -width, 0])         # Up
    p6 = start_pos                                      # Center
    
    # Orientation (Fixed Downward/Forward)
    R_home = T_home[:3, :3]
    from scipy.spatial.transform import Rotation as R
    rpy_home = R.from_matrix(R_home).as_euler('xyz')
    
    # Add moves
    speed = 0.02 # m/s (20mm/s)
    accel = 0.05  # m/s^2 - Reduced accel
    
    # Explicitly add Start point first to ensure controller starts correctly
    points = [p2, p3, p4, p5, p6]
    
    for p in points:
        planner.add_move(
            position=p,
            euler_rpy=rpy_home,
            v_max=speed, 
            a_max=accel,
            mode='exact_stop'
        )
        
    # 3. Generate Path
    print("Generating Path...")
    # Using dt=0.04s (25Hz) - Fine enough for linearity
    segments = planner.plan(T_home, dt=0.04) 
    
    # 4. Interpolate (Controller simulation)
    # We use CNCController just to generate the q_trajectory list
    # We don't need the feedback loop part, just the open-loop generation.
    # Actually CNCController.execute does Differential IK loop.
    # This generates the q_traj we need.
    controller = CNCController(robot)
    print("Computing IK...")
    res = controller.execute(segments, robot.home_config)
    
    time_log = res[0]
    q_traj = res[1]
    
    print(f"Generated {len(q_traj)} points.")
    
    # 5. Connect and Execute
    print("\n--- GRBL Connection ---")
    grbl = GrblRobotControl() # Will prompt for port
    
    # Reset to Home
    print("Homing...")
    grbl.send_command("G92 X0 Y0 Z0 A0 B0")
    grbl.simulated_mpos = np.zeros(5)
    
    input("Press Enter to START trajectory...")
    
    # Stream
    grbl.stream_trajectory(q_traj, time_log)
    
    print("Done.")

if __name__ == "__main__":
    run_rectangle_test()
