#!/usr/bin/env python3
"""
Calibration Rectangle Generator
Draws a precision rectangle to verify kinematic calibration.
"""

import numpy as np
import sys
import os
from scipy.spatial.transform import Rotation as R

# Add current directory to path
sys.path.append(os.path.dirname(os.path.abspath(__file__)))

from grbl_control import GrblRobotControl
from robot_kinematics import RobotKinematics
from CNC import CNCPlanner, CNCController

def main():
    try:
        # 1. Setup
        print("=== Robot Calibration: Rectangle Test ===")
        robot_control = GrblRobotControl()
        kinematics = robot_control.kinematics
        planner = CNCPlanner()
        controller = CNCController(kinematics)
        
        # 2. Homing
        print("Homing robot (setting current pos as 0,0,0,0,0)...")
        robot_control.send_command("G92 X0 Y0 Z0 A0 B0")
        robot_control.simulated_mpos = np.zeros(5)
        
        # 3. Define Rectangle Parameters
        # Center of workspace
        center_x = 0.20 # 200mm forward
        center_y = 0.00 # Center Y
        draw_z   = 0.05 # 50mm height
        
        # Dimensions
        rect_width_y = 0.100  # 100mm (Left-Right)
        rect_depth_x = 0.050  # 50mm (Forward-Back)
        
        # Safe Z for travel
        safe_z = draw_z + 0.015
        
        print(f"Target Rectangle:")
        print(f"  Center: X={center_x*1000:.1f}mm, Y={center_y*1000:.1f}mm, Z={draw_z*1000:.1f}mm")
        print(f"  Size:   Y_width={rect_width_y*1000:.1f}mm, X_depth={rect_depth_x*1000:.1f}mm")
        
        # 4. Define Waypoints
        # Orientation: Fixed Downward (Home Orientation)
        T_home, _ = kinematics.forward_kinematics(robot_control.HOME_CONFIG)
        rpy_home = R.from_matrix(T_home[:3, :3]).as_euler('xyz')
        
        # Corners (Counter-Clockwise starting from Front-Left)
        # Front is smaller X, Back is larger X
        # Left is +Y, Right is -Y
        
        x_front = center_x - rect_depth_x/2
        x_back  = center_x + rect_depth_x/2
        y_left  = center_y + rect_width_y/2
        y_right = center_y - rect_width_y/2
        
        p1 = np.array([x_front, y_left, draw_z])  # Front-Left
        p2 = np.array([x_back,  y_left, draw_z])  # Back-Left
        p3 = np.array([x_back,  y_right, draw_z]) # Back-Right
        p4 = np.array([x_front, y_right, draw_z]) # Front-Right
        
        # Speed
        v_rapid = 0.08
        v_draw  = 0.02 # Slow for precision
        accel   = 0.2
        
        # Build Path
        # 1. Move to P1 (Safe Z)
        planner.add_move(np.array([p1[0], p1[1], safe_z]), rpy_home, v_rapid, accel, 'exact_stop')
        # 2. Plunge P1
        planner.add_move(p1, rpy_home, v_draw, accel, 'exact_stop')
        # 3. Draw Rectangle (Continuous)
        planner.add_move(p2, rpy_home, v_draw, accel, 'continuous')
        planner.add_move(p3, rpy_home, v_draw, accel, 'continuous')
        planner.add_move(p4, rpy_home, v_draw, accel, 'continuous')
        planner.add_move(p1, rpy_home, v_draw, accel, 'continuous')
        # 4. Lift
        planner.add_move(np.array([p1[0], p1[1], safe_z]), rpy_home, v_rapid, accel, 'exact_stop')
        # 5. Return Home
        planner.add_move(T_home[:3,3], rpy_home, v_rapid, accel, 'exact_stop')
        
        # 5. Generate Trajectory
        print("Generating Trajectory (Differential IK)...")
        start_T, _ = kinematics.forward_kinematics(robot_control.HOME_CONFIG)
        segments = planner.plan(start_T, dt=0.04)
        
        results = controller.execute(segments, robot_control.HOME_CONFIG)
        time_log, q_traj = results[0], results[1]
        
        print(f"Generated {len(q_traj)} points.")
        
        # 6. Execute
        input("Press Enter to STREAM trajectory to Robot...")
        robot_control.stream_trajectory(q_traj, time_log)
        
        print("\n=== Measurements ===")
        print(f"Expected Y Width (Left-Right): {rect_width_y*1000:.1f} mm")
        print(f"Expected X Depth (Front-Back): {rect_depth_x*1000:.1f} mm")
        print("Please measure the actual drawn dimensions.")
        print("If Actual > Expected: Steps/Deg is too HIGH (or robot moved too much).")
        print("If Actual < Expected: Steps/Deg is too LOW (or robot moved too little).")
        
    except Exception as e:
        print(f"Error: {e}")
        import traceback
        traceback.print_exc()

if __name__ == "__main__":
    main()

