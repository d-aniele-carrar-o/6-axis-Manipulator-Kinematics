import pandas as pd
import numpy as np
from scipy.signal import savgol_filter, find_peaks
from scipy.spatial.transform import Rotation, Slerp

def parse_and_restructure_data(filepath: str) -> dict:
    """
    Parses the flat CSV file and restructures it into a dictionary of
    DataFrames, one for each robot.

    Args:
        filepath: Path to the motion.csv file.

    Returns:
        A dictionary where keys are robot IDs (1-4) and values are their
        corresponding trajectory DataFrames.
    """
    # Load the raw data (with spaces after commas in header)
    raw_df = pd.read_csv(filepath, skipinitialspace=True)
    
    # Identify the base columns for a single robot
    base_columns = [
        'x', 'xdot', 'fx', 'y', 'ydot', 'fy', 'z', 'zdot', 'fz',
        'rx', 'rx_dot', 'tau_x', 'ry', 'ry_dot', 'tau_y', 'rz', 'rz_dot', 'tau_z'
    ]
    
    # The provided CSV has a slightly different order, let's define the mapping
    # based on the actual header in 1749729939_motion.csv
    header_map = [
        'x', 'xdot', 'fx', 'y', 'ydot', 'fy', 'z', 'zdot', 'fz',
        'rx', 'rx_dot', 'tau_x', 'ry', 'ry_dot', 'tau_y', 'rz', 'rz_dot', 'tau_z'
    ]

    robot_data = {}
    for i in range(1, 5):
        robot_cols = {f'{col}{i}': col_map for col, col_map in zip(base_columns, header_map)}
        
        # Reorder the keys of robot_cols to match the order in raw_df
        ordered_robot_cols = {}
        for raw_col_name in raw_df.columns:
            if raw_col_name in robot_cols:
                ordered_robot_cols[raw_col_name] = robot_cols[raw_col_name]

        # Create a copy with timestamp column
        df = raw_df[list(ordered_robot_cols.keys())].copy()
        df['timestamp'] = raw_df['timestamp']
        df.rename(columns=ordered_robot_cols, inplace=True)
        robot_data[i] = df

    return robot_data

def preprocess_robot_data(df: pd.DataFrame, params: dict) -> pd.DataFrame:
    """
    Applies preprocessing steps to a single robot's trajectory data.
    This includes filtering, derivative calculation, and feature engineering.
    """
    # Convert timestamp to seconds from start
    df['time_sec'] = (df['timestamp'] - df['timestamp'].iloc[0]) / 1e6 # Assuming microseconds
    # Ensure dt is never zero to avoid division by zero in gradient calculations
    dt = df['time_sec'].diff().fillna(0.001).values
    dt[dt < 0.001] = 0.001  # Replace very small dt values with a minimum threshold

    # Define columns for processing
    pos_cols = ['x', 'y', 'z']
    vel_cols = ['xdot', 'ydot', 'zdot']
    force_cols = ['fx', 'fy', 'fz']
    torque_cols = ['tau_x', 'tau_y', 'tau_z']
    angle_axis_cols = ['rx', 'ry', 'rz']

    # --- 1. Noise Filtering (Savitzky-Golay) ---
    for col in pos_cols + vel_cols + force_cols + torque_cols + angle_axis_cols:
        df[f'{col}_filtered'] = savgol_filter(
            df[col],
            window_length=params['savgol_window'],
            polyorder=params['savgol_polyorder']
        )

    # --- 2. Orientation Conversion (Angle-Axis to Quaternion) ---
    angle_axis_vecs = df[[f'{c}_filtered' for c in angle_axis_cols]].values
    # Handle zero-magnitude vectors which cause errors in Rotation.from_rotvec
    non_zero_mask = np.linalg.norm(angle_axis_vecs, axis=1) > 1e-8
    quaternions = np.zeros((len(df), 4))
    quaternions[~non_zero_mask] = np.array([0, 0, 0, 1])  # Identity quaternion
    if np.any(non_zero_mask):
        quaternions[non_zero_mask] = Rotation.from_rotvec(angle_axis_vecs[non_zero_mask]).as_quat()
    df['quaternion'] = list(quaternions)

    # --- 3. Kinematic Derivative Calculation ---
    # Use filtered velocity data
    velocity_filtered = df[[f'{c}_filtered' for c in vel_cols]].values
    
    # Acceleration - use a safer approach with np.gradient
    try:
        # First try with standard gradient
        acceleration = np.gradient(velocity_filtered, dt, axis=0, edge_order=1)
        # Replace any NaN or inf values with zeros
        acceleration = np.nan_to_num(acceleration, nan=0.0, posinf=0.0, neginf=0.0)
        df[['ax', 'ay', 'az']] = acceleration
        
        # Jerk - use a safer approach
        jerk = np.gradient(acceleration, dt, axis=0, edge_order=1)
        # Replace any NaN or inf values with zeros
        jerk = np.nan_to_num(jerk, nan=0.0, posinf=0.0, neginf=0.0)
        df[['jx', 'jy', 'jz']] = jerk
    except Exception as e:
        print(f"Warning: Error in gradient calculation: {e}")
        # Fallback to simple finite differences if gradient fails
        acceleration = np.zeros_like(velocity_filtered)
        acceleration[1:] = (velocity_filtered[1:] - velocity_filtered[:-1]) / dt[:-1, np.newaxis]
        df[['ax', 'ay', 'az']] = acceleration
        
        jerk = np.zeros_like(acceleration)
        jerk[1:] = (acceleration[1:] - acceleration[:-1]) / dt[:-1, np.newaxis]
        df[['jx', 'jy', 'jz']] = jerk

    # --- 4. Magnitude Computations ---
    df['vel_mag'] = np.linalg.norm(df[[f'{c}_filtered' for c in vel_cols]].values, axis=1)
    df['accel_mag'] = np.linalg.norm(acceleration, axis=1)
    df['jerk_mag'] = np.linalg.norm(jerk, axis=1)
    df['force_mag'] = np.linalg.norm(df[[f'{c}_filtered' for c in force_cols]].values, axis=1)
    df['torque_mag'] = np.linalg.norm(df[[f'{c}_filtered' for c in torque_cols]].values, axis=1)

    return df

def douglas_peucker_6d(points_pos, points_ori, epsilon_pos, epsilon_ori, w_pos=0.5, w_ori=0.5):
    """
    Douglas-Peucker algorithm adapted for 6D poses (position + orientation).
    
    Args:
        points_pos: Numpy array of 3D positions.
        points_ori: List/array of Scipy Rotation objects (quaternions).
        epsilon_pos: Tolerance for position.
        epsilon_ori: Tolerance for orientation (in radians).
        w_pos: Weight for positional error.
        w_ori: Weight for orientational error.

    Returns:
        A list of indices of the points to be kept.
    """
    if len(points_pos) < 2:
        return [0]

    def _dp_recursive(start_idx, end_idx):
        if end_idx <= start_idx + 1:
            return []

        max_dist = -1.0
        farthest_idx = -1

        start_pos, end_pos = points_pos[start_idx], points_pos[end_idx]
        start_ori, end_ori = points_ori[start_idx], points_ori[end_idx]
        
        # Create a slerp object for orientation interpolation
        key_rots = Rotation.concatenate([start_ori, end_ori])
        key_times = [0, 1]
        slerp = Slerp(key_times, key_rots)

        for i in range(start_idx + 1, end_idx):
            # Parameter for projection onto the line segment
            t = (i - start_idx) / (end_idx - start_idx)
            
            # Positional distance
            interp_pos = start_pos + t * (end_pos - start_pos)
            dist_pos = np.linalg.norm(points_pos[i] - interp_pos)
            
            # Orientational distance
            interp_ori = slerp([t])
            # Angular distance between two quaternions
            rot_diff = points_ori[i].inv() * interp_ori
            dist_ori = rot_diff.magnitude()

            # Weighted combined distance (normalized by epsilon)
            # This is a heuristic to combine the two error types
            dist = np.sqrt(w_pos * (dist_pos / epsilon_pos)**2 + w_ori * (dist_ori / epsilon_ori)**2)

            if dist > max_dist:
                max_dist = dist
                farthest_idx = i
        
        # If max distance is greater than 1 (since we normalized by epsilon), recurse
        if max_dist > 1.0:
            res1 = _dp_recursive(start_idx, farthest_idx)
            res2 = _dp_recursive(farthest_idx, end_idx)
            return res1 + [farthest_idx] + res2
        else:
            return []

    indices = [0] + _dp_recursive(0, len(points_pos) - 1) + [len(points_pos) - 1]
    return sorted(list(set(indices)))

def get_kinematic_extrema(df: pd.DataFrame, params: dict) -> list:
    """Finds keyframes based on kinematic extrema (velocity minima, jerk maxima)."""
    # Velocity minima (pauses or transitions)
    # find_peaks with a negative signal finds minima
    vel_minima, _ = find_peaks(-df['vel_mag'], prominence=params['vel_min_prominence'])
    
    # Jerk maxima (abrupt changes)
    jerk_maxima, _ = find_peaks(df['jerk_mag'], height=params['jerk_peak_height'])
    
    return list(vel_minima) + list(jerk_maxima)

def get_dynamic_events(df: pd.DataFrame, params: dict) -> list:
    """Finds keyframes based on dynamic events (force/torque thresholds)."""
    force_contact_mask = df['force_mag'] > params['force_contact_threshold']
    
    # Find where the contact state changes
    contact_changes = np.where(np.diff(force_contact_mask.astype(int)))
    
    return list(contact_changes[0])


def main():
    """
    Main function to run the keyframe extraction pipeline.
    """
    # --- Parameters ---
    # These parameters can be tuned to change the algorithm's behavior.
    PARAMS = {
        # Preprocessing
        'savgol_window': 51,  # Must be odd
        'savgol_polyorder': 3,
        # Douglas-Peucker
        'dp_epsilon_pos': 0.01,  # meters
        'dp_epsilon_ori': 0.1,   # radians (~5.7 degrees)
        'dp_weight_pos': 0.5,
        'dp_weight_ori': 0.5,
        # Kinematic Extrema
        'vel_min_prominence': 0.01, # m/s
        'jerk_peak_height': 5.0,    # m/s^3
        # Dynamic Events
        'force_contact_threshold': 5.0, # Newtons
        # Final Filtering
        'min_time_separation_sec': 0.25 # seconds
    }
    
    # --- Load and Preprocess Data ---
    filepath = '1749729939_motion.csv'
    print(f"Loading and restructuring data from {filepath}...")
    all_robot_data_raw = parse_and_restructure_data(filepath)
    
    all_robot_data_processed = {}
    print("Preprocessing data for each robot...")
    for robot_id, df_raw in all_robot_data_raw.items():
        all_robot_data_processed[robot_id] = preprocess_robot_data(df_raw, PARAMS)
        print(f"  - Robot {robot_id} processed.")

    # --- Candidate Keyframe Generation ---
    candidate_indices = set()
    
    # Always include the first and last frame
    num_frames = len(all_robot_data_processed[1])
    candidate_indices.add(0)
    candidate_indices.add(num_frames - 1)
    
    print("\nGenerating candidate keyframes from all methods...")
    for robot_id, df in all_robot_data_processed.items():
        print(f"  Processing Robot {robot_id}...")
        
        # A. Geometric Simplification (Douglas-Peucker)
        pos_points = df[['x_filtered', 'y_filtered', 'z_filtered']].values
        ori_points = [Rotation.from_quat(q) for q in df['quaternion']]
        dp_indices = douglas_peucker_6d(
            pos_points, ori_points,
            PARAMS['dp_epsilon_pos'], PARAMS['dp_epsilon_ori'],
            PARAMS['dp_weight_pos'], PARAMS['dp_weight_ori']
        )
        candidate_indices.update(dp_indices)
        print(f"    - DP found {len(dp_indices)} keyframes.")

        # B. Kinematic Extrema
        kinematic_indices = get_kinematic_extrema(df, PARAMS)
        candidate_indices.update(kinematic_indices)
        print(f"    - Kinematics found {len(kinematic_indices)} keyframes.")

        # C. Dynamic Events
        dynamic_indices = get_dynamic_events(df, PARAMS)
        candidate_indices.update(dynamic_indices)
        print(f"    - Dynamics found {len(dynamic_indices)} keyframes.")

    # --- Synchronization and Final Filtering ---
    print(f"\nTotal unique candidate keyframes before filtering: {len(candidate_indices)}")
    
    # Sort all candidate indices chronologically
    sorted_indices = sorted(list(candidate_indices))
    
    # Filter out keyframes that are too close in time
    final_keyframes = []
    if sorted_indices:
        last_keyframe_time = -np.inf
        time_vector = all_robot_data_processed[1]['time_sec'].values
        
        for idx in sorted_indices:
            current_time = time_vector[idx]
            if current_time - last_keyframe_time >= PARAMS['min_time_separation_sec']:
                final_keyframes.append(idx)
                last_keyframe_time = current_time
    
    # Ensure the last frame is included if it was filtered out
    if num_frames - 1 not in final_keyframes:
        # Check if it's far enough from the last added keyframe
        if time_vector[num_frames - 1] - time_vector[final_keyframes[-1]] >= PARAMS['min_time_separation_sec']:
             final_keyframes.append(num_frames - 1)
        else: # Replace the last one if it's too close
            final_keyframes[-1] = num_frames - 1

    print(f"\nFinal refined keyframes after filtering: {len(final_keyframes)}")
    print("\n--- FINAL KEYFRAME INDICES ---")
    print(sorted(final_keyframes))
    
    # Save keyframes to CSV file for MATLAB
    keyframe_indices = sorted(final_keyframes)
    
    # Method 1: Save just the indices to a simple CSV file
    with open('keyframe_indices.csv', 'w') as f:
        f.write('keyframe_idx\n')  # Header
        for idx in keyframe_indices:
            f.write(f'{idx}\n')
    print(f"\nKeyframe indices saved to keyframe_indices.csv")
    
    # Method 2: Add a keyframe column to the original CSV
    try:
        # Read the original CSV
        orig_df = pd.read_csv(filepath, skipinitialspace=True)
        
        # Create a new column with 0s (not a keyframe)
        orig_df['is_keyframe'] = 0
        
        # Set 1 for keyframe indices
        orig_df.loc[keyframe_indices, 'is_keyframe'] = 1
        
        # Save to a new CSV file
        output_filepath = filepath.replace('.csv', '_with_keyframes.csv')
        orig_df.to_csv(output_filepath, index=False)
        print(f"Original CSV with keyframe column saved to {output_filepath}")
    except Exception as e:
        print(f"Error adding keyframe column to original CSV: {e}")


if __name__ == '__main__':
    main()
