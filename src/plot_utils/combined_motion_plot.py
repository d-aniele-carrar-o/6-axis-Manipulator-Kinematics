import matplotlib.pyplot as plt
import pandas as pd
import numpy as np
import argparse
from scipy.spatial.transform import Rotation as R

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
    
    # Check if file is in old or new format
    try:
        # Try loading as new format with column headers
        df = pd.read_csv(csv_file, skipinitialspace=True)
        new_format = True
        
        # Create time index for new format
        time = np.arange(len(df))
        
        # Check if joint data is available
        has_joint_data = 'q1_1' in df.columns
        
    except Exception:
        # Fall back to old format without headers
        new_format = False
        df = pd.read_csv(csv_file, skipfooter=1, header=None, engine="python")
        data = df.to_numpy()
        
        # Convert time to seconds for old format
        time = (data[:,1] - data[0,1])/1000000.0
        
        has_joint_data = False
    
    # Set x-axis limits based on data or user-provided time_limit
    if time_limit is None:
        xlim = [0, time[-1]]
    else:
        xlim = [0, time_limit]
    
    # Always have 4 robots (2 leader-follower pairs)
    num_robots = 4
    
    # Define column mappings based on format
    if new_format:
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
        
        # Add joint data mappings if available
        robot_joint_cols = {}
        if has_joint_data:
            for i in range(1, num_robots + 1):
                robot_joint_cols[i] = {
                    'q': [f'q1_{i}', f'q2_{i}', f'q3_{i}', f'q4_{i}', f'q5_{i}', f'q6_{i}'],
                    'qdot': [f'qdot1_{i}', f'qdot2_{i}', f'qdot3_{i}', f'qdot4_{i}', f'qdot5_{i}', f'qdot6_{i}'],
                    'tau': [f'tau1_{i}', f'tau2_{i}', f'tau3_{i}', f'tau4_{i}', f'tau5_{i}', f'tau6_{i}']
                }
    else:
        # For old format without headers, use column indices
        # Each robot has 18 columns (x, xdot, fx, y, ydot, fy, z, zdot, fz, rx, rx_dot, tau_x, ry, ry_dot, tau_y, rz, rz_dot, tau_z)
        robot_cols = {}
        for i in range(1, num_robots + 1):
            offset = 2 + (i-1) * 18  # Starting column for each robot (after control_mode, timestamp)
            robot_cols[i] = {
                'x': offset + 0, 'xdot': offset + 1, 'fx': offset + 2,
                'y': offset + 3, 'ydot': offset + 4, 'fy': offset + 5,
                'z': offset + 6, 'zdot': offset + 7, 'fz': offset + 8,
                'rx': offset + 9, 'rx_dot': offset + 10, 'tau_x': offset + 11,
                'ry': offset + 12, 'ry_dot': offset + 13, 'tau_y': offset + 14,
                'rz': offset + 15, 'rz_dot': offset + 16, 'tau_z': offset + 17
            }
        
        # No joint data in old format
        robot_joint_cols = {}
    
    # Calculate orientation differences for each leader-follower pair
    orientation_data = {}
    
    # Process both leader-follower pairs (1-2 and 3-4)
    for pair_idx in range(2):
        leader_idx = pair_idx * 2 + 1
        follower_idx = leader_idx + 1
        
        # Extract rotation vectors
        if new_format:
            # For new format with headers
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
        else:
            # For old format without headers
            leader_rx = robot_cols[leader_idx]['rx']
            leader_ry = robot_cols[leader_idx]['ry']
            leader_rz = robot_cols[leader_idx]['rz']
            follower_rx = robot_cols[follower_idx]['rx']
            follower_ry = robot_cols[follower_idx]['ry']
            follower_rz = robot_cols[follower_idx]['rz']
            
            leader_vec = data[:, (leader_rx, leader_ry, leader_rz)]
            follower_vec = data[:, (follower_rx, follower_ry, follower_rz)]
        
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
        
        # Get data based on format
        if new_format:
            # For new format, get data from dataframe
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
        else:
            # For old format, get data from numpy array
            x1 = data[:, robot_cols[leader_idx]['x']]
            y1 = data[:, robot_cols[leader_idx]['y']]
            z1 = data[:, robot_cols[leader_idx]['z']]
            x2 = data[:, robot_cols[follower_idx]['x']]
            y2 = data[:, robot_cols[follower_idx]['y']]
            z2 = data[:, robot_cols[follower_idx]['z']]
            
            vx1 = data[:, robot_cols[leader_idx]['xdot']]
            vy1 = data[:, robot_cols[leader_idx]['ydot']]
            vz1 = data[:, robot_cols[leader_idx]['zdot']]
            vx2 = data[:, robot_cols[follower_idx]['xdot']]
            vy2 = data[:, robot_cols[follower_idx]['ydot']]
            vz2 = data[:, robot_cols[follower_idx]['zdot']]
            
            wx1 = data[:, robot_cols[leader_idx]['rx_dot']]
            wy1 = data[:, robot_cols[leader_idx]['ry_dot']]
            wz1 = data[:, robot_cols[leader_idx]['rz_dot']]
            wx2 = data[:, robot_cols[follower_idx]['rx_dot']]
            wy2 = data[:, robot_cols[follower_idx]['ry_dot']]
            wz2 = data[:, robot_cols[follower_idx]['rz_dot']]
            
            fx1 = data[:, robot_cols[leader_idx]['fx']]
            fy1 = data[:, robot_cols[leader_idx]['fy']]
            fz1 = data[:, robot_cols[leader_idx]['fz']]
            fx2 = data[:, robot_cols[follower_idx]['fx']]
            fy2 = data[:, robot_cols[follower_idx]['fy']]
            fz2 = data[:, robot_cols[follower_idx]['fz']]
            
            tx1 = data[:, robot_cols[leader_idx]['tau_x']]
            ty1 = data[:, robot_cols[leader_idx]['tau_y']]
            tz1 = data[:, robot_cols[leader_idx]['tau_z']]
            tx2 = data[:, robot_cols[follower_idx]['tau_x']]
            ty2 = data[:, robot_cols[follower_idx]['tau_y']]
            tz2 = data[:, robot_cols[follower_idx]['tau_z']]
        
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
    
    # Plot joint data if available
    if has_joint_data:
        # Define contrasting colors for the 6 joints
        joint_colors = ['#e41a1c', '#377eb8', "#2bb326", "#4F095A", "#ab7125", "#e004c3"]
        
        for pair_idx in range(2):
            leader_idx = pair_idx * 2 + 1
            follower_idx = leader_idx + 1
            
            # Create figure for joint data
            fig_joint, axs_joint = plt.subplots(3, 2, figsize=(14, 10))
            fig_joint.suptitle(f'Joint Space Data - Robots {leader_idx} & {follower_idx}', fontsize=16)
            
            # Joint positions
            for i in range(6):
                axs_joint[0, 0].plot(time, df[robot_joint_cols[leader_idx]['q'][i]][:len(time)], color=joint_colors[i], label=f'Joint {i+1}')
            axs_joint[0, 0].set_title(f'Robot {leader_idx} Joint Positions')
            axs_joint[0, 0].set_ylabel('Position (rad)')
            axs_joint[0, 0].set_xlim(xlim)
            axs_joint[0, 0].legend()
            
            for i in range(6):
                axs_joint[0, 1].plot(time, df[robot_joint_cols[follower_idx]['q'][i]][:len(time)], color=joint_colors[i], label=f'Joint {i+1}')
            axs_joint[0, 1].set_title(f'Robot {follower_idx} Joint Positions')
            axs_joint[0, 1].set_ylabel('Position (rad)')
            axs_joint[0, 1].set_xlim(xlim)
            axs_joint[0, 1].legend()
            
            # Joint velocities
            for i in range(6):
                axs_joint[1, 0].plot(time, df[robot_joint_cols[leader_idx]['qdot'][i]][:len(time)], color=joint_colors[i], label=f'Joint {i+1}')
            axs_joint[1, 0].set_title(f'Robot {leader_idx} Joint Velocities')
            axs_joint[1, 0].set_ylabel('Velocity (rad/s)')
            axs_joint[1, 0].set_xlim(xlim)
            axs_joint[1, 0].legend()
            
            for i in range(6):
                axs_joint[1, 1].plot(time, df[robot_joint_cols[follower_idx]['qdot'][i]][:len(time)], color=joint_colors[i], label=f'Joint {i+1}')
            axs_joint[1, 1].set_title(f'Robot {follower_idx} Joint Velocities')
            axs_joint[1, 1].set_ylabel('Velocity (rad/s)')
            axs_joint[1, 1].set_xlim(xlim)
            axs_joint[1, 1].legend()
            
            # Joint torques
            for i in range(6):
                axs_joint[2, 0].plot(time, df[robot_joint_cols[leader_idx]['tau'][i]][:len(time)], color=joint_colors[i], label=f'Joint {i+1}')
            axs_joint[2, 0].set_title(f'Robot {leader_idx} Joint Torques')
            axs_joint[2, 0].set_ylabel('Torque (Nm)')
            axs_joint[2, 0].set_xlabel('Time (samples)')
            axs_joint[2, 0].set_xlim(xlim)
            axs_joint[2, 0].legend()
            
            for i in range(6):
                axs_joint[2, 1].plot(time, df[robot_joint_cols[follower_idx]['tau'][i]][:len(time)], color=joint_colors[i], label=f'Joint {i+1}')
            axs_joint[2, 1].set_title(f'Robot {follower_idx} Joint Torques')
            axs_joint[2, 1].set_ylabel('Torque (Nm)')
            axs_joint[2, 1].set_xlabel('Time (samples)')
            axs_joint[2, 1].set_xlim(xlim)
            axs_joint[2, 1].legend()
            
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
