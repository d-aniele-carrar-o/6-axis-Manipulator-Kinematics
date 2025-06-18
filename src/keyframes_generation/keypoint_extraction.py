import pandas as pd
import numpy as np
from scipy.signal import savgol_filter, find_peaks
from scipy.spatial.transform import Rotation, Slerp
import os


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
        # print(f"Warning: Error in gradient calculation: {e}")
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

def filter_similar_keyframes(indices, pos_data, ori_data, pos_threshold=0.01, ori_threshold=0.1):
    """
    Filter out keyframes that are too similar to their neighbors based on position and orientation changes.
    
    Args:
        indices: List of keyframe indices
        pos_data: Position data for all frames (numpy array)
        ori_data: Orientation data for all frames (list of Rotation objects)
        pos_threshold: Threshold for position change (in meters)
        ori_threshold: Threshold for orientation change (in radians)
        
    Returns:
        List of filtered keyframe indices
    """
    if len(indices) <= 2:
        return indices  # Keep at least first and last frame
    
    # Always keep first and last indices
    filtered_indices = [indices[0]]
    last_kept_pos = pos_data[indices[0]]
    last_kept_ori = ori_data[indices[0]]
    
    # Check each keyframe against the last kept one
    for i in range(1, len(indices) - 1):
        idx = indices[i]
        current_pos = pos_data[idx]
        current_ori = ori_data[idx]
        
        try:
            # Calculate position and orientation differences
            pos_diff = np.linalg.norm(current_pos - last_kept_pos)
            ori_diff = (current_ori.inv() * last_kept_ori).magnitude()
            
            # Keep this keyframe if either position or orientation changed significantly
            if pos_diff > pos_threshold or ori_diff > ori_threshold:
                filtered_indices.append(idx)
                last_kept_pos = current_pos
                last_kept_ori = current_ori
        except Exception as e:
            print(f"Warning: Error calculating differences for index {idx}: {e}")
            # If there's an error, keep the keyframe to be safe
            filtered_indices.append(idx)
            last_kept_pos = current_pos
            last_kept_ori = current_ori
    
    # Always add the last index
    filtered_indices.append(indices[-1])
    
    return filtered_indices

def main(filepath=None, params=None):
    """
    Main function to run the keyframe extraction pipeline.
    
    Args:
        filepath: Path to the motion CSV file. If None, uses default.
        params: Dictionary of parameters to override defaults.
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
        'min_time_separation_sec': 0.6, # seconds
        # Similarity Filtering
        'pos_similarity_threshold': 0.02,  # meters (5cm)
        'ori_similarity_threshold': 0.2,   # radians (~17.1 degrees)
        # Method Selection (True to use, False to ignore)
        'use_geometric': True,      # Use geometric simplification (Douglas-Peucker)
        'use_kinematic': True,      # Use kinematic extrema (velocity minima, jerk maxima)
        'use_dynamic': True         # Use dynamic events (force/torque thresholds)
    }
    
    # Override parameters if provided
    if params:
        for key, value in params.items():
            if key in PARAMS:
                PARAMS[key] = value
    
    # --- Load and Preprocess Data ---
    if filepath is None:
        filepath = '/Users/danielecarraro/Documents/GITHUB/6-axis-Manipulator-Kinematics/good/data/1750265649_motion_move_unbalanced.csv'
    print(f"Loading and restructuring data from {filepath}...")
    all_robot_data_raw = parse_and_restructure_data(filepath)
    
    all_robot_data_processed = {}
    print("Preprocessing data for each robot...")
    for robot_id, df_raw in all_robot_data_raw.items():
        all_robot_data_processed[robot_id] = preprocess_robot_data(df_raw, PARAMS)
        print(f"  - Robot {robot_id} processed.")

    # --- Candidate Keyframe Generation ---
    candidate_indices = set()
    # Dictionary to track which methods found each keyframe
    keyframe_methods = {}
    
    # Always include the first and last frame
    num_frames = len(all_robot_data_processed[1])
    candidate_indices.add(0)
    candidate_indices.add(num_frames - 1)
    keyframe_methods[0] = "boundary"
    keyframe_methods[num_frames - 1] = "boundary"
    
    print("\nGenerating candidate keyframes from selected methods...")
    active_methods = []
    if PARAMS['use_geometric']:
        active_methods.append("Geometric (Douglas-Peucker)")
    if PARAMS['use_kinematic']:
        active_methods.append("Kinematic extrema")
    if PARAMS['use_dynamic']:
        active_methods.append("Dynamic events")
    print(f"  Active methods: {', '.join(active_methods)}")
    
    for robot_id, df in all_robot_data_processed.items():
        print(f"  Processing Robot {robot_id}...")
        
        # A. Geometric Simplification (Douglas-Peucker)
        if PARAMS['use_geometric']:
            pos_points = df[['x_filtered', 'y_filtered', 'z_filtered']].values
            ori_points = [Rotation.from_quat(q) for q in df['quaternion']]
            dp_indices = douglas_peucker_6d(
                pos_points, ori_points,
                PARAMS['dp_epsilon_pos'], PARAMS['dp_epsilon_ori'],
                PARAMS['dp_weight_pos'], PARAMS['dp_weight_ori']
            )
            for idx in dp_indices:
                candidate_indices.add(idx)
                if idx not in keyframe_methods:  # First method for this keyframe
                    keyframe_methods[idx] = "geometric"
                elif keyframe_methods[idx] != "boundary":  # Don't modify boundary frames
                    if "geometric" not in keyframe_methods[idx]:  # Avoid duplicates
                        keyframe_methods[idx] += "/geometric"
            print(f"    - DP found {len(dp_indices)} keyframes.")
        else:
            print(f"    - Geometric method disabled.")

        # B. Kinematic Extrema
        if PARAMS['use_kinematic']:
            kinematic_indices = get_kinematic_extrema(df, PARAMS)
            for idx in kinematic_indices:
                candidate_indices.add(idx)
                if idx not in keyframe_methods:  # First method for this keyframe
                    keyframe_methods[idx] = "kinematic"
                elif keyframe_methods[idx] != "boundary":  # Don't modify boundary frames
                    if "kinematic" not in keyframe_methods[idx]:  # Avoid duplicates
                        keyframe_methods[idx] += "/kinematic"
            print(f"    - Kinematics found {len(kinematic_indices)} keyframes.")
        else:
            print(f"    - Kinematic method disabled.")

        # C. Dynamic Events
        if PARAMS['use_dynamic']:
            dynamic_indices = get_dynamic_events(df, PARAMS)
            for idx in dynamic_indices:
                candidate_indices.add(idx)
                if idx not in keyframe_methods:  # First method for this keyframe
                    keyframe_methods[idx] = "dynamic"
                elif keyframe_methods[idx] != "boundary":  # Don't modify boundary frames
                    if "dynamic" not in keyframe_methods[idx]:  # Avoid duplicates
                        keyframe_methods[idx] += "/dynamic"
            print(f"    - Dynamics found {len(dynamic_indices)} keyframes.")
        else:
            print(f"    - Dynamic method disabled.")

    # --- Synchronization and Final Filtering ---
    print(f"\nTotal unique candidate keyframes before filtering: {len(candidate_indices)}")
    
    # Filter keyframes based on selected methods
    filtered_indices = []
    for idx in candidate_indices:
        # Always include boundary frames
        if idx == 0 or idx == num_frames - 1:
            filtered_indices.append(idx)
            continue
            
        method = keyframe_methods.get(idx, "")
        methods_list = method.split('/')
        
        # Check if any of the active methods identified this keyframe
        keep = False
        if PARAMS['use_geometric'] and "geometric" in methods_list:
            keep = True
        elif PARAMS['use_kinematic'] and "kinematic" in methods_list:
            keep = True
        elif PARAMS['use_dynamic'] and "dynamic" in methods_list:
            keep = True
            
        if keep:
            filtered_indices.append(idx)
    
    # Sort all candidate indices chronologically
    sorted_indices = sorted(filtered_indices)
    
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
            
    # Make sure the method for the last frame is preserved
    if num_frames - 1 in final_keyframes and num_frames - 1 not in keyframe_methods:
        keyframe_methods[num_frames - 1] = "boundary"
    
    # Store the count after time-based filtering        
    time_filtered_count = len(final_keyframes)
    print(f"\nKeyframes after time-based filtering: {time_filtered_count}")
    
    # Add additional filtering for similar keyframes
    # Get position and orientation data for robot 1 (could use any robot)
    pos_data = all_robot_data_processed[1][['x_filtered', 'y_filtered', 'z_filtered']].values
    
    # Convert quaternions to Rotation objects with error handling
    ori_data = []
    for q in all_robot_data_processed[1]['quaternion']:
        try:
            # Ensure quaternion is valid
            q_array = np.array(q)
            if len(q_array) != 4 or np.isnan(q_array).any():
                # Use identity quaternion for invalid data
                ori_data.append(Rotation.from_quat([0, 0, 0, 1]))
            else:
                ori_data.append(Rotation.from_quat(q_array))
        except Exception as e:
            print(f"Warning: Error converting quaternion {q}: {e}")
            # Use identity quaternion as fallback
            ori_data.append(Rotation.from_quat([0, 0, 0, 1]))
    
    # Filter out keyframes with minimal movement
    similarity_filtered_keyframes = filter_similar_keyframes(
        final_keyframes, 
        pos_data, 
        ori_data,
        PARAMS['pos_similarity_threshold'],
        PARAMS['ori_similarity_threshold']
    )
    
    # Update final keyframes with the similarity filtered ones
    final_keyframes = similarity_filtered_keyframes

    print(f"Final keyframes after similarity filtering: {len(final_keyframes)}")
    print("\n--- FINAL KEYFRAME INDICES ---")
    print(sorted(final_keyframes))
    
    # Save keyframes to CSV file with extraction method
    keyframe_indices = sorted(final_keyframes)
    
    # Create a list of methods for the final keyframes
    # If a keyframe was filtered out during time or similarity filtering, its method won't be in the final list
    final_methods = []
    for idx in keyframe_indices:
        method = keyframe_methods.get(idx, "unknown")
        # If this is a keyframe that survived filtering, add its method
        final_methods.append(method)
        
    # Print statistics about methods used
    multi_method_count = sum(1 for method in final_methods if "/" in method)
    print(f"\nKeyframes identified by multiple methods: {multi_method_count} ({(multi_method_count/len(final_methods))*100:.1f}%)")
    
    # Count keyframes by method
    method_counts = {
        "boundary": 0,
        "geometric": 0,
        "kinematic": 0,
        "dynamic": 0
    }
    
    for method in final_methods:
        methods_list = method.split('/')
        for m in methods_list:
            if m in method_counts:
                method_counts[m] += 1
    
    # Print method statistics
    print("\nKeyframes by method:")
    for method, count in method_counts.items():
        if method == "boundary":
            print(f"  - {method}: {count} (always included)")
        else:
            if PARAMS[f'use_{method}']:
                print(f"  - {method}: {count} ({(count/len(final_methods))*100:.1f}%)")
            else:
                print(f"  - {method}: 0 (disabled)")
    
    # Save indices and methods to a CSV file in the same directory as the original file
    original_dir = os.path.dirname(filepath)
    original_filename = os.path.basename(filepath)
    output_filename = os.path.splitext(original_filename)[0] + "_keyframe_indxs.csv"
    output_kf_filepath = os.path.join(original_dir, output_filename)
    
    # Save as DataFrame for better formatting
    kf_df = pd.DataFrame({
        'keyframe_idx': keyframe_indices,
        'extraction_method': final_methods
    })
    kf_df.to_csv(output_kf_filepath, index=False)
    print(f"\nKeyframe indices and methods saved to {output_kf_filepath}")
    
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
    import argparse
    
    # Parse command line arguments
    parser = argparse.ArgumentParser(description='Extract keyframes from motion data.')
    parser.add_argument('--filepath', type=str, help='Path to the motion CSV file')
    parser.add_argument('--methods', type=str, default='all', 
                        help='Comma-separated list of methods to use: geometric,kinematic,dynamic or "all"')
    
    args = parser.parse_args()
    
    # Prepare parameters
    params = {}
    
    # Override methods based on command line arguments
    if args.methods != 'all':
        # Set all methods to False by default
        params['use_geometric'] = False
        params['use_kinematic'] = False
        params['use_dynamic'] = False
        
        # Enable only the specified methods
        methods = args.methods.split(',')
        for method in methods:
            method = method.strip().lower()
            if method in ['geometric', 'kinematic', 'dynamic']:
                params[f'use_{method}'] = True
    
    main(filepath=args.filepath, params=params)