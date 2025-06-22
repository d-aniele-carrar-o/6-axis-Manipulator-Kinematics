import matplotlib
matplotlib.use('TkAgg')  # Ensure interactive backend
import matplotlib.pyplot as plt
import pandas as pd
import numpy as np
import argparse
from scipy.spatial.transform import Rotation as R
from scipy.signal import savgol_filter

def plot_robot_motion(csv_file, time_limit=None, show_error_plots=False):
    """
    Create comprehensive plots of robot motion data from CSV file
    
    Parameters:
    -----------
    csv_file : str
        Path to the CSV motion file
    time_limit : float, optional
        Time limit for x-axis in seconds (default: use full data range)
    show_error_plots : bool, optional
        Whether to show position error plots (default: False)
    """
    # Set plot style
    plt.rcParams['font.family'] = 'Times New Roman'
    plt.rcParams['xtick.direction'] = 'in'
    plt.rcParams['ytick.direction'] = 'in'
    
    # Check if file has headers and add them if missing
    try:
        # Try loading with headers first
        df_test = pd.read_csv(csv_file, nrows=1)
        if df_test.columns[0].isdigit() or 'Unnamed' in df_test.columns[0]:
            # No headers detected, add them
            header = "control_type,timestamp,x1,xdot1,fx1,y1,ydot1,fy1,z1,zdot1,fz1,rx1,rx_dot1,tau_x1,ry1,ry_dot1,tau_y1,rz1,rz_dot1,tau_z1,x2,xdot2,fx2,y2,ydot2,fy2,z2,zdot2,fz2,rx2,rx_dot2,tau_x2,ry2,ry_dot2,tau_y2,rz2,rz_dot2,tau_z2,x3,xdot3,fx3,y3,ydot3,fy3,z3,zdot3,fz3,rx3,rx_dot3,tau_x3,ry3,ry_dot3,tau_y3,rz3,rz_dot3,tau_z3,x4,xdot4,fx4,y4,ydot4,fy4,z4,zdot4,fz4,rx4,rx_dot4,tau_x4,ry4,ry_dot4,tau_y4,rz4,rz_dot4,tau_z4"
            with open(csv_file, 'r') as f:
                content = f.read()
            with open(csv_file, 'w') as f:
                f.write(header + '\n' + content)
        
        # Now load with headers
        df = pd.read_csv(csv_file, skipinitialspace=True)
        
        # Create time index for new format
        time = np.arange(len(df))
        
        # Check if joint data is available
        has_joint_data = 'q1_1' in df.columns
        
    except Exception as e:
        print(f"Error processing CSV file: {e}")
        return
    
    # Set x-axis limits based on data or user-provided time_limit
    if time_limit is None:
        xlim = [0, time[-1]]
    else:
        xlim = [0, time_limit]
    
    # Always have 4 robots (2 leader-follower pairs)
    num_robots = 4
    
    # Clean noisy data for robot 4 only using moving average
    # Apply smoothing to force data with window size 16
    force_cols = ['fx4', 'fy4', 'fz4', 'tau_x4', 'tau_y4', 'tau_z4']
    for col in force_cols:
        if col in df.columns:
            df[col] = np.convolve(df[col], np.ones(16)/16, mode='same')
    
    # Apply smoothing to velocity data with window size 20
    velocity_cols = ['xdot4', 'ydot4', 'zdot4', 'rx_dot4', 'ry_dot4', 'rz_dot4']
    for col in velocity_cols:
        if col in df.columns:
            df[col] = np.convolve(df[col], np.ones(20)/20, mode='same')
    
    # Apply velocity smoothing to robots 1&2 with window size 14
    velocity_cols_12 = ['xdot1', 'ydot1', 'zdot1', 'rx_dot1', 'ry_dot1', 'rz_dot1',
                        'xdot2', 'ydot2', 'zdot2', 'rx_dot2', 'ry_dot2', 'rz_dot2']
    for col in velocity_cols_12:
        if col in df.columns:
            df[col] = np.convolve(df[col], np.ones(14)/14, mode='same')
    
    # Apply velocity smoothing to robot 3 with window size 5
    velocity_cols_3 = ['xdot3', 'ydot3', 'zdot3', 'rx_dot3', 'ry_dot3', 'rz_dot3']
    for col in velocity_cols_3:
        if col in df.columns:
            df[col] = np.convolve(df[col], np.ones(5)/5, mode='same')
    
    # Fix force/torque scaling - robots 1&2 have wrong scale (too high)
    scale_factor = 10 / 400  # Scale down from ~400N to ~10N range
    for robot_idx in [1, 2]:
        force_cols = [f'fx{robot_idx}', f'fy{robot_idx}', f'fz{robot_idx}']
        torque_cols = [f'tau_x{robot_idx}', f'tau_y{robot_idx}', f'tau_z{robot_idx}']
        
        for col in force_cols + torque_cols:
            if col in df.columns:
                df[col] = df[col] * scale_factor
    
    # Create column mappings for each robot with named columns
    robot_cols = {}
    for i in range(1, num_robots + 1):
        robot_cols[i] = {
            'x': f'x{i}', 'y': f'y{i}', 'z': f'z{i}',
            'rx': f'rx{i}', 'ry': f'ry{i}', 'rz': f'rz{i}',
            'xdot': f'xdot{i}', 'ydot': f'ydot{i}', 'zdot': f'zdot{i}',
            'rx_dot': f'rx_dot{i}', 'ry_dot': f'ry_dot{i}', 'rz_dot': f'rz_dot{i}',
            'fx': f'fx{i}', 'fy': f'fy{i}', 'fz': f'fz{i}',
            'tau_x': f'tau_x{i}', 'tau_y': f'tau_y{i}', 'tau_z': f'tau_z{i}'
        }
    
    # Calculate orientation differences for each leader-follower pair
    orientation_data = {}
    
    # Process both leader-follower pairs (1-2 and 3-4)
    for pair_idx in range(2):
        leader_idx = pair_idx * 2 + 1
        follower_idx = leader_idx + 1
        
        # Extract rotation vectors
        leader_vec = np.column_stack((
            df[robot_cols[leader_idx]['rx']], 
            df[robot_cols[leader_idx]['ry']], 
            df[robot_cols[leader_idx]['rz']]
        ))
        follower_vec = np.column_stack((
            df[robot_cols[follower_idx]['rx']], 
            df[robot_cols[follower_idx]['ry']], 
            df[robot_cols[follower_idx]['rz']]
        ))
        
        # Get initial rotation vectors and matrices
        leader_init_vec = leader_vec[0]
        follower_init_vec = follower_vec[0]
        leader_init_mat = R.from_rotvec(leader_init_vec).as_matrix()
        follower_init_mat = R.from_rotvec(follower_init_vec).as_matrix()
        
        # Initialize arrays
        data_len = len(leader_vec)
        dev_rot_vec = np.zeros((data_len, 3))
        leader_offset_vec_list = np.zeros((data_len, 3))
        follower_offset_vec_list = np.zeros((data_len, 3))
        
        # Calculate orientation differences
        for i in range(data_len):
            leader_mat = R.from_rotvec(leader_vec[i]).as_matrix()
            follower_mat = R.from_rotvec(follower_vec[i]).as_matrix()
            
            # Calculate relative orientation
            leader_offset_mat = leader_mat @ leader_init_mat.T
            follower_offset_mat = follower_mat @ follower_init_mat.T
            
            l_minus_f_mat = leader_offset_mat @ follower_offset_mat.T
            l_minus_f_vec = R.from_matrix(l_minus_f_mat).as_rotvec()
            
            dev_rot_vec[i,:] = l_minus_f_vec
            leader_offset_vec_list[i,:] = R.from_matrix(leader_offset_mat).as_rotvec()
            follower_offset_vec_list[i,:] = R.from_matrix(follower_offset_mat).as_rotvec()
        
        # Store results for this pair
        orientation_data[pair_idx] = {
            'dev_rot_vec': dev_rot_vec,
            'leader_offset_vec_list': leader_offset_vec_list,
            'follower_offset_vec_list': follower_offset_vec_list
        }
    
    # Create plots for each leader-follower pair
    for pair_idx in range(2):
        leader_idx = pair_idx * 2 + 1
        follower_idx = leader_idx + 1
        
        # Get orientation data for this pair
        dev_rot_vec = orientation_data[pair_idx]['dev_rot_vec']
        leader_offset_vec_list = orientation_data[pair_idx]['leader_offset_vec_list']
        follower_offset_vec_list = orientation_data[pair_idx]['follower_offset_vec_list']
        
        # Create figure for this robot pair
        fig, axs = plt.subplots(3, 2, figsize=(14, 10))
        fig.suptitle(f'Robot Motion Data - Robots {leader_idx} & {follower_idx}', fontsize=16)
        
        # Apply time limit if specified
        if time_limit is not None:
            # Find the index corresponding to the time limit
            time_idx = np.searchsorted(time, time_limit)
            # Limit the time array and all data arrays to this index
            time = time[:time_idx]
        
        # Get data from dataframe
        x1 = df[robot_cols[leader_idx]['x']]
        y1 = df[robot_cols[leader_idx]['y']]
        z1 = df[robot_cols[leader_idx]['z']]
        x2 = df[robot_cols[follower_idx]['x']]
        y2 = df[robot_cols[follower_idx]['y']]
        z2 = df[robot_cols[follower_idx]['z']]
        
        vx1 = df[robot_cols[leader_idx]['xdot']]
        vy1 = df[robot_cols[leader_idx]['ydot']]
        vz1 = df[robot_cols[leader_idx]['zdot']]
        vx2 = df[robot_cols[follower_idx]['xdot']]
        vy2 = df[robot_cols[follower_idx]['ydot']]
        vz2 = df[robot_cols[follower_idx]['zdot']]
        
        wx1 = df[robot_cols[leader_idx]['rx_dot']]
        wy1 = df[robot_cols[leader_idx]['ry_dot']]
        wz1 = df[robot_cols[leader_idx]['rz_dot']]
        wx2 = df[robot_cols[follower_idx]['rx_dot']]
        wy2 = df[robot_cols[follower_idx]['ry_dot']]
        wz2 = df[robot_cols[follower_idx]['rz_dot']]
        
        fx1 = df[robot_cols[leader_idx]['fx']]
        fy1 = df[robot_cols[leader_idx]['fy']]
        fz1 = df[robot_cols[leader_idx]['fz']]
        fx2 = df[robot_cols[follower_idx]['fx']]
        fy2 = df[robot_cols[follower_idx]['fy']]
        fz2 = df[robot_cols[follower_idx]['fz']]
        
        tx1 = df[robot_cols[leader_idx]['tau_x']]
        ty1 = df[robot_cols[leader_idx]['tau_y']]
        tz1 = df[robot_cols[leader_idx]['tau_z']]
        tx2 = df[robot_cols[follower_idx]['tau_x']]
        ty2 = df[robot_cols[follower_idx]['tau_y']]
        tz2 = df[robot_cols[follower_idx]['tau_z']]
        
        # Position plots
        axs[0, 0].plot(time, x1[:len(time)], 'r-', label=f'Robot {leader_idx} X')
        axs[0, 0].plot(time, y1[:len(time)], 'g-', label=f'Robot {leader_idx} Y')
        axs[0, 0].plot(time, z1[:len(time)], 'b-', label=f'Robot {leader_idx} Z')
        axs[0, 0].plot(time, x2[:len(time)], 'r--', label=f'Robot {follower_idx} X')
        axs[0, 0].plot(time, y2[:len(time)], 'g--', label=f'Robot {follower_idx} Y')
        axs[0, 0].plot(time, z2[:len(time)], 'b--', label=f'Robot {follower_idx} Z')
        axs[0, 0].set_title('Position')
        axs[0, 0].set_ylabel('Position (m)')
        axs[0, 0].set_xlim(xlim)
        axs[0, 0].legend()
        
        # Rotation plots
        axs[0, 1].plot(time, leader_offset_vec_list[:len(time),0], 'r-', label=f'Robot {leader_idx} RX')
        axs[0, 1].plot(time, leader_offset_vec_list[:len(time),1], 'g-', label=f'Robot {leader_idx} RY')
        axs[0, 1].plot(time, leader_offset_vec_list[:len(time),2], 'b-', label=f'Robot {leader_idx} RZ')
        axs[0, 1].plot(time, follower_offset_vec_list[:len(time),0], 'r--', label=f'Robot {follower_idx} RX')
        axs[0, 1].plot(time, follower_offset_vec_list[:len(time),1], 'g--', label=f'Robot {follower_idx} RY')
        axs[0, 1].plot(time, follower_offset_vec_list[:len(time),2], 'b--', label=f'Robot {follower_idx} RZ')
        axs[0, 1].set_title('Rotation')
        axs[0, 1].set_ylabel('Rotation (rad)')
        axs[0, 1].set_xlim(xlim)
        axs[0, 1].legend()
        
        # Velocity plots
        axs[1, 0].plot(time, vx1[:len(time)], 'r-', label=f'Robot {leader_idx} X')
        axs[1, 0].plot(time, vy1[:len(time)], 'g-', label=f'Robot {leader_idx} Y')
        axs[1, 0].plot(time, vz1[:len(time)], 'b-', label=f'Robot {leader_idx} Z')
        axs[1, 0].plot(time, vx2[:len(time)], 'r--', label=f'Robot {follower_idx} X')
        axs[1, 0].plot(time, vy2[:len(time)], 'g--', label=f'Robot {follower_idx} Y')
        axs[1, 0].plot(time, vz2[:len(time)], 'b--', label=f'Robot {follower_idx} Z')
        axs[1, 0].set_title('Linear Velocity')
        axs[1, 0].set_ylabel('Velocity (m/s)')
        axs[1, 0].set_xlim(xlim)
        axs[1, 0].legend()
        
        # Angular velocity plots
        axs[1, 1].plot(time, wx1[:len(time)], 'r-', label=f'Robot {leader_idx} RX')
        axs[1, 1].plot(time, wy1[:len(time)], 'g-', label=f'Robot {leader_idx} RY')
        axs[1, 1].plot(time, wz1[:len(time)], 'b-', label=f'Robot {leader_idx} RZ')
        axs[1, 1].plot(time, wx2[:len(time)], 'r--', label=f'Robot {follower_idx} RX')
        axs[1, 1].plot(time, wy2[:len(time)], 'g--', label=f'Robot {follower_idx} RY')
        axs[1, 1].plot(time, wz2[:len(time)], 'b--', label=f'Robot {follower_idx} RZ')
        axs[1, 1].set_title('Angular Velocity')
        axs[1, 1].set_ylabel('Angular Velocity (rad/s)')
        axs[1, 1].set_xlim(xlim)
        axs[1, 1].legend()
        
        # Force plots
        axs[2, 0].plot(time, fx1[:len(time)], 'r-', label=f'Robot {leader_idx} FX')
        axs[2, 0].plot(time, fy1[:len(time)], 'g-', label=f'Robot {leader_idx} FY')
        axs[2, 0].plot(time, fz1[:len(time)], 'b-', label=f'Robot {leader_idx} FZ')
        axs[2, 0].plot(time, fx2[:len(time)], 'r--', label=f'Robot {follower_idx} FX')
        axs[2, 0].plot(time, fy2[:len(time)], 'g--', label=f'Robot {follower_idx} FY')
        axs[2, 0].plot(time, fz2[:len(time)], 'b--', label=f'Robot {follower_idx} FZ')
        axs[2, 0].set_title('Forces')
        axs[2, 0].set_ylabel('Force (N)')
        axs[2, 0].set_xlabel('Time (samples)')
        axs[2, 0].set_xlim(xlim)
        axs[2, 0].legend()
        
        # Torque plots
        axs[2, 1].plot(time, tx1[:len(time)], 'r-', label=f'Robot {leader_idx} X')
        axs[2, 1].plot(time, ty1[:len(time)], 'g-', label=f'Robot {leader_idx} Y')
        axs[2, 1].plot(time, tz1[:len(time)], 'b-', label=f'Robot {leader_idx} Z')
        axs[2, 1].plot(time, tx2[:len(time)], 'r--', label=f'Robot {follower_idx} X')
        axs[2, 1].plot(time, ty2[:len(time)], 'g--', label=f'Robot {follower_idx} Y')
        axs[2, 1].plot(time, tz2[:len(time)], 'b--', label=f'Robot {follower_idx} Z')
        axs[2, 1].set_title('Torques')
        axs[2, 1].set_ylabel('Torque (Nm)')
        axs[2, 1].set_xlabel('Time (samples)')
        axs[2, 1].set_xlim(xlim)
        axs[2, 1].legend()
        
        plt.tight_layout(rect=[0, 0, 1, 0.96])
        
        # Add force and torque derivative plots for robots 1 & 2 only
        if pair_idx == 0:  # Only for the first pair (robots 1 & 2)
            # Smooth the force and torque data before calculating derivatives
            window_length = min(51, len(time) // 10 * 2 + 1)  # Adaptive window size, must be odd
            fx1_smooth = savgol_filter(fx1[:len(time)], window_length, 3)
            fy1_smooth = savgol_filter(fy1[:len(time)], window_length, 3)
            fz1_smooth = savgol_filter(fz1[:len(time)], window_length, 3)
            fx2_smooth = savgol_filter(fx2[:len(time)], window_length, 3)
            fy2_smooth = savgol_filter(fy2[:len(time)], window_length, 3)
            fz2_smooth = savgol_filter(fz2[:len(time)], window_length, 3)
            
            tx1_smooth = savgol_filter(tx1[:len(time)], window_length, 3)
            ty1_smooth = savgol_filter(ty1[:len(time)], window_length, 3)
            tz1_smooth = savgol_filter(tz1[:len(time)], window_length, 3)
            tx2_smooth = savgol_filter(tx2[:len(time)], window_length, 3)
            ty2_smooth = savgol_filter(ty2[:len(time)], window_length, 3)
            tz2_smooth = savgol_filter(tz2[:len(time)], window_length, 3)
            
            # Calculate derivatives using multi-sample finite difference (100 samples to match oscillation period)
            n_samples = 100
            dfx1_dt = np.zeros_like(fx1_smooth)
            dfy1_dt = np.zeros_like(fy1_smooth)
            dfz1_dt = np.zeros_like(fz1_smooth)
            dfx2_dt = np.zeros_like(fx2_smooth)
            dfy2_dt = np.zeros_like(fy2_smooth)
            dfz2_dt = np.zeros_like(fz2_smooth)
            
            dtx1_dt = np.zeros_like(tx1_smooth)
            dty1_dt = np.zeros_like(ty1_smooth)
            dtz1_dt = np.zeros_like(tz1_smooth)
            dtx2_dt = np.zeros_like(tx2_smooth)
            dty2_dt = np.zeros_like(ty2_smooth)
            dtz2_dt = np.zeros_like(tz2_smooth)
            
            for i in range(n_samples, len(time)):
                dt = time[i] - time[i-n_samples]
                dfx1_dt[i] = (fx1_smooth[i] - fx1_smooth[i-n_samples]) / dt
                dfy1_dt[i] = (fy1_smooth[i] - fy1_smooth[i-n_samples]) / dt
                dfz1_dt[i] = (fz1_smooth[i] - fz1_smooth[i-n_samples]) / dt
                dfx2_dt[i] = (fx2_smooth[i] - fx2_smooth[i-n_samples]) / dt
                dfy2_dt[i] = (fy2_smooth[i] - fy2_smooth[i-n_samples]) / dt
                dfz2_dt[i] = (fz2_smooth[i] - fz2_smooth[i-n_samples]) / dt
                
                dtx1_dt[i] = (tx1_smooth[i] - tx1_smooth[i-n_samples]) / dt
                dty1_dt[i] = (ty1_smooth[i] - ty1_smooth[i-n_samples]) / dt
                dtz1_dt[i] = (tz1_smooth[i] - tz1_smooth[i-n_samples]) / dt
                dtx2_dt[i] = (tx2_smooth[i] - tx2_smooth[i-n_samples]) / dt
                dty2_dt[i] = (ty2_smooth[i] - ty2_smooth[i-n_samples]) / dt
                dtz2_dt[i] = (tz2_smooth[i] - tz2_smooth[i-n_samples]) / dt
            
            # Apply additional smoothing to derivatives
            deriv_window = min(31, len(time) // 20 * 2 + 1)
            dfx1_dt = savgol_filter(dfx1_dt, deriv_window, 2)
            dfy1_dt = savgol_filter(dfy1_dt, deriv_window, 2)
            dfz1_dt = savgol_filter(dfz1_dt, deriv_window, 2)
            dfx2_dt = savgol_filter(dfx2_dt, deriv_window, 2)
            dfy2_dt = savgol_filter(dfy2_dt, deriv_window, 2)
            dfz2_dt = savgol_filter(dfz2_dt, deriv_window, 2)
            
            dtx1_dt = savgol_filter(dtx1_dt, deriv_window, 2)
            dty1_dt = savgol_filter(dty1_dt, deriv_window, 2)
            dtz1_dt = savgol_filter(dtz1_dt, deriv_window, 2)
            dtx2_dt = savgol_filter(dtx2_dt, deriv_window, 2)
            dty2_dt = savgol_filter(dty2_dt, deriv_window, 2)
            dtz2_dt = savgol_filter(dtz2_dt, deriv_window, 2)
            
            # Apply moving average to reduce sample-to-sample oscillations (50 samples for half oscillation period)
            avg_window = 50
            dfx1_dt = np.convolve(dfx1_dt, np.ones(avg_window)/avg_window, mode='same')
            dfy1_dt = np.convolve(dfy1_dt, np.ones(avg_window)/avg_window, mode='same')
            dfz1_dt = np.convolve(dfz1_dt, np.ones(avg_window)/avg_window, mode='same')
            dfx2_dt = np.convolve(dfx2_dt, np.ones(avg_window)/avg_window, mode='same')
            dfy2_dt = np.convolve(dfy2_dt, np.ones(avg_window)/avg_window, mode='same')
            dfz2_dt = np.convolve(dfz2_dt, np.ones(avg_window)/avg_window, mode='same')
            
            dtx1_dt = np.convolve(dtx1_dt, np.ones(avg_window)/avg_window, mode='same')
            dty1_dt = np.convolve(dty1_dt, np.ones(avg_window)/avg_window, mode='same')
            dtz1_dt = np.convolve(dtz1_dt, np.ones(avg_window)/avg_window, mode='same')
            dtx2_dt = np.convolve(dtx2_dt, np.ones(avg_window)/avg_window, mode='same')
            dty2_dt = np.convolve(dty2_dt, np.ones(avg_window)/avg_window, mode='same')
            dtz2_dt = np.convolve(dtz2_dt, np.ones(avg_window)/avg_window, mode='same')
            
            # Create figure for force and torque derivatives
            fig_deriv, axs_deriv = plt.subplots(1, 2, figsize=(14, 6))
            fig_deriv.suptitle(f'Force and Torque Derivatives - Robots {leader_idx} & {follower_idx}', fontsize=16)
            
            # Force derivative plots
            axs_deriv[0].plot(time, dfx1_dt, 'r-', label=f'Robot {leader_idx} dFX/dt')
            axs_deriv[0].plot(time, dfy1_dt, 'g-', label=f'Robot {leader_idx} dFY/dt')
            axs_deriv[0].plot(time, dfz1_dt, 'b-', label=f'Robot {leader_idx} dFZ/dt')
            axs_deriv[0].plot(time, dfx2_dt, 'r--', label=f'Robot {follower_idx} dFX/dt')
            axs_deriv[0].plot(time, dfy2_dt, 'g--', label=f'Robot {follower_idx} dFY/dt')
            axs_deriv[0].plot(time, dfz2_dt, 'b--', label=f'Robot {follower_idx} dFZ/dt')
            axs_deriv[0].set_title('Force Derivatives')
            axs_deriv[0].set_ylabel('Force Derivative (N/s)')
            axs_deriv[0].set_xlabel('Time (samples)')
            axs_deriv[0].set_xlim(xlim)
            axs_deriv[0].legend()
            
            # Torque derivative plots
            axs_deriv[1].plot(time, dtx1_dt, 'r-', label=f'Robot {leader_idx} dTX/dt')
            axs_deriv[1].plot(time, dty1_dt, 'g-', label=f'Robot {leader_idx} dTY/dt')
            axs_deriv[1].plot(time, dtz1_dt, 'b-', label=f'Robot {leader_idx} dTZ/dt')
            axs_deriv[1].plot(time, dtx2_dt, 'r--', label=f'Robot {follower_idx} dTX/dt')
            axs_deriv[1].plot(time, dty2_dt, 'g--', label=f'Robot {follower_idx} dTY/dt')
            axs_deriv[1].plot(time, dtz2_dt, 'b--', label=f'Robot {follower_idx} dTZ/dt')
            axs_deriv[1].set_title('Torque Derivatives')
            axs_deriv[1].set_ylabel('Torque Derivative (Nm/s)')
            axs_deriv[1].set_xlabel('Time (samples)')
            axs_deriv[1].set_xlim(xlim)
            axs_deriv[1].legend()
            
            plt.tight_layout(rect=[0, 0, 1, 0.96])
        
        # Add position error plots if requested
        if show_error_plots:
            fig_err, axs_err = plt.subplots(3, 1, figsize=(14, 10))
            fig_err.suptitle(f'Position Error - Robots {leader_idx} & {follower_idx}', fontsize=16)
            
            # X position error
            axs_err[0].plot(time, (x1 - x2)[:len(time)], 'r-', label='X Error')
            axs_err[0].set_title('X Position Error')
            axs_err[0].set_ylabel('Error (m)')
            axs_err[0].set_xlim(xlim)
            axs_err[0].legend()
            
            # Y position error
            axs_err[1].plot(time, (y1 - y2)[:len(time)], 'g-', label='Y Error')
            axs_err[1].set_title('Y Position Error')
            axs_err[1].set_ylabel('Error (m)')
            axs_err[1].set_xlim(xlim)
            axs_err[1].legend()
            
            # Z position error
            axs_err[2].plot(time, (z1 - z2)[:len(time)], 'b-', label='Z Error')
            axs_err[2].set_title('Z Position Error')
            axs_err[2].set_ylabel('Error (m)')
            axs_err[2].set_xlabel('Time (samples)')
            axs_err[2].set_xlim(xlim)
            axs_err[2].legend()
            
            plt.tight_layout(rect=[0, 0, 1, 0.96])
    
    # Save figures
    filename_base = csv_file.split('.')[0]
    plt.savefig(f"{filename_base}_motion.png")
    
    plt.show()

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Plot robot motion data from CSV file')
    parser.add_argument('csv_file', type=str, help='Path to the CSV motion file')
    parser.add_argument('--time_limit', type=float, default=None, 
                        help='Time limit for x-axis in seconds (default: use full data range)')
    parser.add_argument('--show_error_plots', action='store_true',
                        help='Show position error plots (default: False)')
    args = parser.parse_args()

    plot_robot_motion(args.csv_file, args.time_limit, args.show_error_plots)
