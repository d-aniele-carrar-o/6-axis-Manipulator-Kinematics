#!/usr/bin/env python3
"""
Trajectory Visualizer and Planner for 6-Axis Manipulators.

This script demonstrates the CNC Path Planning and Control system.
It loads configuration, plans a trajectory, simulates the control loop,
visualizes the result, and optionally executes it on the real robot.
"""

import numpy as np
import os
import yaml
from typing import Dict

from robot_kinematics import RobotKinematics
from CNC import CNCPlanner, CNCController
from trajectory_visualizer import TrajectoryVisualizer
from real_robot_interface import GrblRobotControl


def load_config(config_path: str = "config.yaml") -> Dict:
    """Load configuration from YAML file."""
    # Get the directory where main.py is located
    script_dir = os.path.dirname(os.path.abspath(__file__))
    # Construct absolute path to config.yaml
    abs_config_path = os.path.join(script_dir, config_path)
    
    with open(abs_config_path, 'r') as f:
        # Load as raw YAML first to handle structure
        config = yaml.safe_load(f)

    def evaluate_pi_expressions(data):
        """Recursively traverse and evaluate strings containing $pi."""
        if isinstance(data, dict):
            return {k: evaluate_pi_expressions(v) for k, v in data.items()}
        elif isinstance(data, list):
            return [evaluate_pi_expressions(i) for i in data]
        elif isinstance(data, str) and '$pi' in data:
            # Replace $pi with numeric value and evaluate
            # WARNING: eval is used here. Only use with trusted config files.
            expr = data.replace('$pi', str(np.pi))
            try:
                return eval(expr, {"__builtins__": None}, {})
            except Exception as e:
                print(f"Warning: Could not evaluate expression '{data}': {e}")
                return data
        else:
            return data

    return evaluate_pi_expressions(config)


def main():
    """Main function demonstrating trajectory planning and visualization."""
    # 1. Load configuration
    config = load_config()
    
    # Extract parameters
    robot_name = config['robot']['name']
    
    # Target
    end_pos = np.array(config['target']['position'])
    end_euler = config['target']['orientation']
    
    # Trajectory Settings
    traj_config = config['trajectory']
    dt = traj_config['dt']
    
    # Velocity Profile Params
    v_max = traj_config['velocity_profile']['max_velocity']
    a_max = traj_config['velocity_profile']['max_acceleration']
    
    print(f"--- Configuration Loaded ---")
    print(f"Robot: {robot_name}")
    print(f"Target: Pos={end_pos}, Euler={end_euler}")
    print(f"Trajectory: dt={dt}s")
    print(f"Profile Limits: v_max={v_max}, a_max={a_max}")
    print(f"----------------------------")

    # 2. Initialize System
    robot = RobotKinematics(robot_name, home_config=config['robot'][robot_name]['home_config'])
    planner = CNCPlanner()
    controller = CNCController(robot)
    
    print(f"Robot home configuration: {np.rad2deg(robot.home_config)} deg")
    
    # 3. Plan Path
    # Add move from Config
    # Note: CNCPlanner assumes we start from current pose (which is home initially)
    # The first segment will go from Home -> Target
    planner.add_move(
        position=end_pos,
        euler_rpy=end_euler,
        v_max=v_max, 
        a_max=a_max,
        mode='exact_stop'
    )
    
    # You can add more moves here if needed, e.g.:
    # planner.add_move(position=..., euler_rpy=...)

    print("Generating CNC Path...")
    start_T, _ = robot.forward_kinematics()
    segments = planner.plan(start_T, dt=dt)
    
    # 4. Execute (Control Loop Simulation)
    print("Simulating Control Loop...")
    results = controller.execute(segments)
    
    time_log, q_traj, q_dot_traj, vd_log, v_act_log, Td_log, segment_indices = results
    
    if len(time_log) == 0:
        print("Error: No trajectory generated.")
        return
    
    # 5. Visualization
    if config['visualization']['animate'] or config['visualization']['plot_analysis']:
        # Use get() with default 1 for speedup to be backward compatible
        speedup = config['visualization']['speedup']
        
        visualizer = TrajectoryVisualizer(
            robot, 
            frame_scale=config['visualization']['frame_scale'],
            animation_interval=config['visualization']['animation_interval'],
            speedup=speedup
        )
        
        if config['visualization']['animate']:
            print("Displaying Animation...")
            visualizer.animate_trajectory(q_traj, segments)
            
        if config['visualization']['plot_analysis']:
            print("Displaying Analysis Plots...")
            visualizer.plot_analysis(
                q_trajectory=q_traj, 
                q_dot_trajectory=q_dot_traj, 
                desired_trajectory=Td_log, 
                desired_velocities=vd_log, 
                actual_task_velocities=v_act_log,
                segment_indices=segment_indices,
                time_log=time_log
            )

    # 6. Real Robot Execution
    if config['real_robot']['execute']:
        print("\n--- Real Robot Execution ---")
        try:
            port = config['real_robot']['port']
            # Instantiate GrblRobotControl
            grbl = GrblRobotControl(port=port)
            
            # Reset to Home (Optional, but good practice to ensure state)
            print("Homing internal state...")
            grbl.send_command("G92 X0 Y0 Z0 A0 B0")
            grbl.simulated_mpos = np.zeros(5)

            print("Note: Ensure robot is physically at HOME position before starting.")
            input("Press Enter to execute trajectory on REAL ROBOT...")
            
            # Stream
            # Note: GrblRobotControl.stream_trajectory expects q_traj in radians
            grbl.stream_trajectory(q_traj, time_log)
            
            print("Real Robot Execution Done.")
            
        except Exception as e:
            print(f"Error executing on real robot: {e}")
    

if __name__ == "__main__":
    main()
