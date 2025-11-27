import numpy as np
import matplotlib.pyplot as plt
from typing import List, Tuple, Dict, Optional
from scipy.spatial.transform import Rotation as R
from scipy.spatial.transform import Slerp
from abc import ABC, abstractmethod
from matplotlib.animation import FuncAnimation

from robot_kinematics import RobotKinematics


# --- Modular Velocity Profile System ---

class VelocityProfile(ABC):
    """Abstract base class for motion profiles."""
    @abstractmethod
    def generate(self, distance: float, v_max: float, a_max: float, dt: float) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
        """Returns time, position, velocity arrays."""
        pass

class LSPBProfile(VelocityProfile):
    """
    Linear Segment with Parabolic Blend (Trapezoidal Velocity Profile).
    Standard in industrial CNCs.
    """
    def generate(self, distance: float, v_max: float, a_max: float, dt: float):
        # Handle very small distances
        if distance < 1e-6:
            return np.array([0.0]), np.array([0.0]), np.array([0.0])
        
        # 1. Check if triangular profile is needed (short distance)
        # Time to reach v_max
        t_acc = v_max / a_max
        dist_acc = 0.5 * a_max * t_acc**2
        
        if 2 * dist_acc > distance:
            # Triangle Profile (Cannot reach v_max)
            t_acc = np.sqrt(distance / a_max)
            v_peak = a_max * t_acc
            total_time = 2 * t_acc
            # Adjust v_max for the generation loop
            v_cruise = v_peak # Reached peak but no cruise
            t_cruise = 0
            dist_cruise = 0  # No cruise distance in triangular profile
        else:
            # Trapezoidal Profile
            dist_cruise = distance - 2 * dist_acc
            t_cruise = dist_cruise / v_max if v_max > 1e-6 else 0
            total_time = 2 * t_acc + t_cruise
            v_cruise = v_max

        # Generate points (ensure at least start and end points)
        times = np.arange(0, total_time + dt, dt)
        if len(times) < 2:
            times = np.array([0.0, total_time])
        s = []    # Position along path
        s_dot = [] # Velocity along path
        
        for t in times:
            if t <= t_acc:
                # Acceleration phase
                s.append(0.5 * a_max * t**2)
                s_dot.append(a_max * t)
            elif t <= t_acc + t_cruise:
                # Cruise phase
                s.append(dist_acc + v_cruise * (t - t_acc))
                s_dot.append(v_cruise)
            else:
                # Deceleration phase
                t_dec = t - (t_acc + t_cruise)
                rem_dist = v_cruise * t_dec - 0.5 * a_max * t_dec**2
                s.append(dist_acc + dist_cruise + rem_dist)
                s_dot.append(v_cruise - a_max * t_dec)
                
        return times, np.array(s), np.array(s_dot)


# --- CNC Interpolation & Planning ---

class CartesianSegment:
    """
    Represents a single motion segment from P_start to P_end
    interpolated in Cartesian space.
    """
    def __init__(self, start_pose: np.ndarray, end_pose: np.ndarray, 
                 v_max: float, a_max: float, dt: float, mode: str = 'exact_stop'):
        self.start_pose = start_pose
        self.end_pose = end_pose
        self.dt = dt
        self.mode = mode
        
        # Decompose Poses
        self.p_start = start_pose[:3, 3]
        self.p_end = end_pose[:3, 3]
        self.R_start = start_pose[:3, :3]
        self.R_end = end_pose[:3, :3]
        
        # 1. Geometry Calculation
        self.dist_vec = self.p_end - self.p_start
        self.linear_dist = np.linalg.norm(self.dist_vec)
        self.direction = self.dist_vec / self.linear_dist if self.linear_dist > 1e-6 else np.zeros(3)
        
        # Orientation Difference (Global)
        # R_end = R_diff * R_start  => R_diff = R_end * R_start^T
        self.R_diff = self.R_end @ self.R_start.T
        self.rot_vec_total = R.from_matrix(self.R_diff).as_rotvec()
        self.total_angle = np.linalg.norm(self.rot_vec_total)
        
        # Slerp Setup
        # We use scipy Slerp which takes a list of rotations and a list of times (0..1)
        self.key_rots = R.from_matrix([self.R_start, self.R_end])
        self.slerp = Slerp([0, 1], self.key_rots)
        
        # 2. Velocity Profile Generation (Time Law)
        # We drive the motion based on the linear distance
        # If linear distance is zero (pure rotation), we should drive based on angle?
        # For simplicity in this CNC, we assume there is linear motion or we use angle as "distance"
        
        if self.linear_dist < 1e-6:
            # Pure rotation or no motion
            motion_mag = self.total_angle
            self.is_pure_rotation = True
        else:
            motion_mag = self.linear_dist
            self.is_pure_rotation = False
            
        self.profile = LSPBProfile()
        self.times, self.s, self.s_dot = self.profile.generate(motion_mag, v_max, a_max, dt)
        
    def get_state(self, index: int) -> Tuple[np.ndarray, np.ndarray, bool]:
        """
        Get the Cartesian state (Pose, Velocity) at time step 'index'.
        Returns: (T_des, v_des, is_done)
        """
        if index >= len(self.times):
            return self.end_pose, np.zeros(6), True
        
        s = self.s[index]
        s_dot = self.s_dot[index] # Speed along path
        
        # Normalized path parameter u in [0, 1]
        motion_len = self.linear_dist if not self.is_pure_rotation else self.total_angle
        u = s / motion_len if motion_len > 1e-6 else 1.0
        u = np.clip(u, 0, 1)
        
        # Interpolate Position
        if self.is_pure_rotation:
            p_curr = self.p_start
            v_lin = np.zeros(3)
            u_dot = s_dot / motion_len if motion_len > 1e-6 else 0
        else:
            # Use clamped u to prevent overshoot beyond P_end
            p_curr = self.p_start + self.direction * (u * motion_len)
            v_lin = self.direction * s_dot
            u_dot = s_dot / motion_len
            
        # Interpolate Orientation (SLERP)
        r_curr = self.slerp(u)
        
        # Calculate Angular Velocity
        # Angular velocity vector w such that R_dot = w x R
        # For Slerp/Geodesic: w = axis_of_rotation * angle_rate
        # The axis of rotation (global) is fixed for a geodesic between two orientations.
        # It is exactly the axis of R_diff = R_end * R_start^T
        # The total angle is |rot_vec_total|. 
        # The current angle is theta(t) = total_angle * u(t)
        # So theta_dot = total_angle * u_dot
        
        w_vec = self.rot_vec_total * u_dot
        
        # Construct Result
        T_curr = np.eye(4)
        T_curr[:3, 3] = p_curr
        T_curr[:3, :3] = r_curr.as_matrix()
        
        v_curr = np.concatenate([v_lin, w_vec])
        
        return T_curr, v_curr, False


class CNCPlanner:
    """
    Simplified Path Planner.
    Accepts via-points and generates a sequence of Cartesian Segments.
    """
    def __init__(self):
        self.via_points = [] # List of (Pose, v_max, a_max, mode)
        
    def add_move(self, position: np.ndarray, euler_rpy: List[float], 
                 v_max: float = 0.1, a_max: float = 0.5, mode: str = 'exact_stop'):
        """
        Add a target waypoint.
        mode: 'exact_stop' (G61) or 'continuous' (G64)
        """
        R_mat = R.from_euler('xyz', euler_rpy).as_matrix()
        T = np.eye(4)
        T[:3, :3] = R_mat
        T[:3, 3] = position
        self.via_points.append({'pose': T, 'v': v_max, 'a': a_max, 'mode': mode})

    def plan(self, start_pose: np.ndarray, dt: float) -> List[CartesianSegment]:
        """
        Generate motion segments from start_pose through all via_points.
        """
        segments = []
        current_pose = start_pose.copy()
        
        for wp in self.via_points:
            target_pose = wp['pose']
            v = wp['v']
            a = wp['a']
            mode = wp['mode']
            
            segment = CartesianSegment(current_pose, target_pose, v, a, dt, mode)
            segments.append(segment)
            
            current_pose = target_pose
            
        return segments


class CNCController:
    """
    Executes the generated segments on the robot using Differential IK.
    Simulates the real-time control loop.
    """
    def __init__(self, robot: RobotKinematics):
        self.robot = robot
        
    def execute(self, segments: List[CartesianSegment], start_q: np.ndarray) -> Tuple:
        """
        Run the control loop.
        """
        time_log = []
        q_log = []
        q_dot_log = []
        vd_log = []     # Desired task velocities
        v_act_log = []  # Actual task velocities
        pd_log = []     # Desired position
        rpy_d_log = []  # Desired orientation (RPY)
        segment_end_times = [] # Times when each segment ends
        
        current_q = start_q.copy()
        current_time = 0.0
        
        print(f"Starting CNC execution. Processing {len(segments)} blocks.")
        
        for i, segment in enumerate(segments):
            print(f"  > Block {i+1}: Dist={segment.linear_dist:.3f}m, Planned Time={segment.times[-1]:.2f}s")
            
            idx = 0
            while True:
                # 1. Get Setpoint (Interpolator)
                # If idx exceeds profile length, returns P_end, v=0, is_done=True
                Td, vd, is_done_profile = segment.get_state(idx)
                
                # 2. Check Execution Status
                Te, _ = self.robot.forward_kinematics(current_q)
                
                # Calculate Error relative to the CURRENT setpoint
                p_err = np.linalg.norm(Td[:3, 3] - Te[:3, 3])
                
                # Orientation Error for exit condition
                R_err = Td[:3, :3] @ Te[:3, :3].T
                rot_vec = R.from_matrix(R_err).as_rotvec()
                angle_err = np.linalg.norm(rot_vec)
                
                # Exit Condition: Profile Finished AND Physical Error Small (Exact Stop)
                if is_done_profile:
                    # Record the time when the segment finishes
                    segment_end_times.append(current_time)
                    
                    # G64: Continuous Path (Approximate) - Don't wait for settle
                    if segment.mode == 'continuous':
                         print(f"    Continuous transition (G64). Pos Error: {p_err*1000:.3f}mm, Orien Error: {np.rad2deg(angle_err):.3f}deg")
                         break
                    
                    # G61: Exact Stop - Wait for settle
                    # Thresholds: 0.1mm, 0.01deg
                    if p_err < 1e-4 and angle_err < np.radians(0.01):
                        break
                
                # 3. Control Step (Differential IK)
                dt = segment.dt
                next_q, q_dot = self.robot.differential_inverse_kinematics_step(current_q, Td, vd, dt)
                
                # Calculate Actual Task Velocity
                J = self.robot.jacobian(current_q)
                v_actual = J @ q_dot
                
                current_q = next_q
                
                # Logging
                time_log.append(current_time)
                q_log.append(current_q)
                q_dot_log.append(q_dot)
                vd_log.append(vd)
                v_act_log.append(v_actual)
                
                # Desired Pose Logging
                pd_log.append(Td[:3, 3])
                rpy_d = R.from_matrix(Td[:3, :3]).as_euler('xyz')
                rpy_d_log.append(rpy_d)
                
                current_time += dt
                idx += 1
                
        return (np.array(time_log), np.array(q_log), np.array(q_dot_log),
                np.array(vd_log), np.array(v_act_log), 
                np.array(pd_log), np.array(rpy_d_log), segment_end_times)


# --- Visualization ---

class TrajectoryVisualizer:
    """Handles all visualization for robot trajectories."""
    
    def __init__(self, robot: RobotKinematics,
                 frame_scale: float = 0.05,
                 animation_interval: int = 100,
                 figsize: Tuple[int, int] = (14, 10)):
        self.robot = robot
        self.frame_scale = frame_scale
        self.animation_interval = animation_interval
        self.figsize = figsize
    
    def plot_frame(self, ax, T: np.ndarray, scale: Optional[float] = None):
        if scale is None:
            scale = self.frame_scale
            
        origin = T[:3, 3]
        R_mat = T[:3, :3]
        
        colors = ['r', 'g', 'b']
        labels = ['X', 'Y', 'Z']
        for i in range(3):
            axis_vec = R_mat[:, i] * scale
            ax.quiver(origin[0], origin[1], origin[2], 
                     axis_vec[0], axis_vec[1], axis_vec[2], 
                     color=colors[i], arrow_length_ratio=0.2, label=labels[i] if i == 0 else "")
    
    def _precompute_trajectory_data(self, q_trajectory: List[np.ndarray], 
                                   desired_trajectory: List[np.ndarray]) -> Dict:
        all_joint_positions = []
        all_ee_transforms = []
        actual_ee_positions = []
        
        for q in q_trajectory:
            Te, transforms = self.robot.forward_kinematics(q)
            joint_pos_list = [np.zeros(3)] + [T[:3, 3] for T in transforms]
            all_joint_positions.append(np.array(joint_pos_list))
            all_ee_transforms.append(Te)
            actual_ee_positions.append(Te[:3, 3])
        
        # Interpolate desired positions to match trajectory length
        desired_pos_raw = np.array([T[:3, 3] for T in desired_trajectory])
        n_traj = len(q_trajectory)
        n_desired = len(desired_pos_raw)
        
        if n_desired != n_traj:
            # Linear interpolation to match trajectory length
            t_desired = np.linspace(0, 1, n_desired)
            t_traj = np.linspace(0, 1, n_traj)
            desired_ee_positions = np.array([
                np.interp(t_traj, t_desired, desired_pos_raw[:, i]) 
                for i in range(3)
            ]).T
        else:
            desired_ee_positions = desired_pos_raw
            
        actual_ee_positions = np.array(actual_ee_positions)
        
        # Calculate bounds
        all_points = np.vstack([
            np.vstack(all_joint_positions).reshape(-1, 3),
            desired_ee_positions
        ])
        
        margin = 0.1
        min_bounds = all_points.min(axis=0) - margin
        max_bounds = all_points.max(axis=0) + margin
        
        return {
            'joint_positions': all_joint_positions,
            'ee_transforms': all_ee_transforms,
            'desired_positions': desired_ee_positions,
            'actual_positions': actual_ee_positions,
            'min_bounds': min_bounds,
            'max_bounds': max_bounds
        }
    
    def animate_trajectory(self, q_trajectory: List[np.ndarray], 
                          desired_trajectory: List[np.ndarray]) -> FuncAnimation:
        data = self._precompute_trajectory_data(q_trajectory, desired_trajectory)
        
        fig = plt.figure(figsize=self.figsize)
        ax = fig.add_subplot(111, projection='3d')
        
        def update(frame):
            ax.clear()
            
            # Plot Robot
            joints = data['joint_positions'][frame]
            ax.plot(joints[:, 0], joints[:, 1], joints[:, 2], 
                   'bo-', linewidth=3, markersize=8, label='Robot')
            
            # Plot Trajectories
            ax.plot(data['desired_positions'][:, 0], 
                   data['desired_positions'][:, 1], 
                   data['desired_positions'][:, 2], 
                   'g--', alpha=0.5, linewidth=2, label='Desired Path')
            
            trail_len = min(frame + 1, len(data['actual_positions']))
            ax.plot(data['actual_positions'][:trail_len, 0], 
                   data['actual_positions'][:trail_len, 1], 
                   data['actual_positions'][:trail_len, 2], 
                   'r-', alpha=0.7, linewidth=2, label='Actual Path')
            
            # Current EE frame
            self.plot_frame(ax, data['ee_transforms'][frame])
            
            # Error line
            curr_des = data['desired_positions'][frame]
            curr_act = data['actual_positions'][frame]
            ax.plot([curr_des[0], curr_act[0]], 
                   [curr_des[1], curr_act[1]], 
                   [curr_des[2], curr_act[2]], 
                   'k:', linewidth=2, alpha=0.7, label='Error')
            
            # Settings
            ax.set_xlim(data['min_bounds'][0], data['max_bounds'][0])
            ax.set_ylim(data['min_bounds'][1], data['max_bounds'][1])
            ax.set_zlim(data['min_bounds'][2], data['max_bounds'][2])
            ax.set_xlabel('X (m)')
            ax.set_ylabel('Y (m)')
            ax.set_zlabel('Z (m)')
            ax.set_title(f'Frame {frame + 1}/{len(q_trajectory)}')
            ax.legend(loc='upper right')
            ax.grid(True)
            
            # Error text
            err_val = np.linalg.norm(curr_des - curr_act)
            ax.text2D(0.05, 0.95, f"Error: {err_val*1000:.1f} mm", 
                     transform=ax.transAxes,
                     bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.8))
        
        anim = FuncAnimation(fig, update, frames=len(q_trajectory), 
                           interval=self.animation_interval, repeat=True)
        plt.tight_layout()
        plt.show()
        return anim


# --- Main Execution ---

if __name__ == "__main__":
    robot = RobotKinematics("3Dprinted", home_config=np.array([-np.pi/4, np.pi/2, 0.0, 0.0, -np.pi/2, 0.0]))
    planner = CNCPlanner()
    controller = CNCController(robot)
    
    # 1. Define Trajectory (G-Code like commands)
    planner.add_move(
        position=robot.get_home_pose()[:3, 3] + np.array([0.0, 0.1, 0.0]), 
        euler_rpy=R.from_matrix(robot.get_home_pose()[:3, :3]).as_euler('xyz'),
        v_max=0.1, a_max=0.5,
        mode='exact_stop'
    )
    # planner.add_move(
    #     position=np.array([0.15, -0.1, 0.3]), 
    #     euler_rpy=[0, np.radians(20), 0], 
    #     v_max=0.15, a_max=0.5,
    #     mode='exact_stop'
    # )
    # planner.add_move(
    #     position=np.array([0.15, -0.1, 0.15]), 
    #     euler_rpy=[0, 0, 0], 
    #     v_max=0.1, a_max=0.5,
    #     mode='exact_stop'
    # )
    
    # 2. Plan (Generate Segments)
    print("Generating CNC Path...")
    start_T, _ = robot.forward_kinematics(robot.home_config)
    segments = planner.plan(start_T, dt=0.01)
    
    # 3. Execute (Control Loop)
    time, q_traj, q_dot_traj, vd_log, v_act_log, pd_log, rpy_d_log, segment_end_times = controller.execute(segments, robot.home_config)
    
    # 4. Visualization
    if len(time) == 0:
        print("Error: No trajectory generated.")
        exit(1)
        
    # Reconstruct Actual Path (Pose)
    p_act_log = []
    rpy_act_log = []
    for q in q_traj:
        T, _ = robot.forward_kinematics(q)
        p_act_log.append(T[:3, 3])
        rpy_act_log.append(R.from_matrix(T[:3, :3]).as_euler('xyz'))
    p_act_log = np.array(p_act_log)
    rpy_act_log = np.array(rpy_act_log)

    # Prepare desired trajectory for visualization
    # We reconstruct it from segments for accuracy
    desired_trajectory = []
    # Start
    desired_trajectory.append(start_T)
    # Ends of each segment
    for seg in segments:
        desired_trajectory.append(seg.end_pose)
    
    visualizer = TrajectoryVisualizer(robot)
    
    # Animate
    q_traj_list = [q_traj[i] for i in range(len(q_traj))]
    anim = visualizer.animate_trajectory(q_traj_list, desired_trajectory)
    
    # Plots
    fig = plt.figure(figsize=(16, 12))
    
    # Helper to add via-point lines
    def add_viapoints(ax):
        for t_via in segment_end_times:
            ax.axvline(x=t_via, color='k', linestyle=':', alpha=0.5, linewidth=1.5)
    
    # --- 1,1 (Top-Left): Task EE Position + Orientation ---
    ax1 = fig.add_subplot(2, 2, 1)
    
    # Position (Left Axis)
    colors_pos = ['r', 'g', 'b']
    labels_pos = ['X', 'Y', 'Z']
    lns1 = []
    for i in range(3):
        l1, = ax1.plot(time, pd_log[:, i], color=colors_pos[i], linestyle='--', label=f'{labels_pos[i]} des')
        l2, = ax1.plot(time, p_act_log[:, i], color=colors_pos[i], linestyle='-', label=f'{labels_pos[i]} act')
        lns1.extend([l1, l2])
        
    ax1.set_title("Task Position & Orientation")
    ax1.set_ylabel("Position (m)")
    ax1.grid(True)
    add_viapoints(ax1)
    
    # Orientation (Right Axis)
    ax1b = ax1.twinx()
    colors_ori = ['c', 'm', 'y'] # Cyan, Magenta, Yellow for distinctness
    labels_ori = ['R', 'P', 'Y']
    # Unwrap angles
    rpy_d_unwrapped = np.unwrap(rpy_d_log, axis=0)
    rpy_act_unwrapped = np.unwrap(rpy_act_log, axis=0)
    
    lns2 = []
    for i in range(3):
        l1, = ax1b.plot(time, np.rad2deg(rpy_d_unwrapped[:, i]), color=colors_ori[i], linestyle='--', label=f'{labels_ori[i]} des')
        l2, = ax1b.plot(time, np.rad2deg(rpy_act_unwrapped[:, i]), color=colors_ori[i], linestyle='-', label=f'{labels_ori[i]} act')
        lns2.extend([l1, l2])
        
    ax1b.set_ylabel("Orientation (deg)")
    
    # Combined Legend
    lns = lns1 + lns2
    labs = [l.get_label() for l in lns]
    ax1.legend(lns, labs, loc='center left', bbox_to_anchor=(1.1, 0.5), fontsize='x-small')


    # --- 1,2 (Top-Right): Joint Space Configurations ---
    ax2 = fig.add_subplot(2, 2, 2)
    for i in range(6):
        ax2.plot(time, np.rad2deg(q_traj[:, i]), label=f'q{i+1}')
    ax2.set_title("Joint Angles (deg)")
    ax2.set_ylabel("Angle (deg)")
    ax2.legend(ncol=3, fontsize='small')
    ax2.grid(True)
    add_viapoints(ax2)


    # --- 2,1 (Bottom-Left): Task Velocities (Linear + Angular) ---
    ax3 = fig.add_subplot(2, 2, 3)
    
    # Linear Velocity (Left Axis)
    lns3 = []
    for i in range(3):
        l1, = ax3.plot(time, vd_log[:, i], color=colors_pos[i], linestyle='--', label=f'v{labels_pos[i].lower()} des')
        l2, = ax3.plot(time, v_act_log[:, i], color=colors_pos[i], linestyle='-', label=f'v{labels_pos[i].lower()} act')
        lns3.extend([l1, l2])
        
    ax3.set_title("Task Velocities")
    ax3.set_xlabel("Time (s)")
    ax3.set_ylabel("Linear Vel (m/s)")
    ax3.grid(True)
    add_viapoints(ax3)
    
    # Angular Velocity (Right Axis)
    ax3b = ax3.twinx()
    lns4 = []
    labels_w = ['wx', 'wy', 'wz']
    for i in range(3):
        l1, = ax3b.plot(time, vd_log[:, i+3], color=colors_ori[i], linestyle='--', label=f'{labels_w[i]} des')
        l2, = ax3b.plot(time, v_act_log[:, i+3], color=colors_ori[i], linestyle='-', label=f'{labels_w[i]} act')
        lns4.extend([l1, l2])
        
    ax3b.set_ylabel("Angular Vel (rad/s)")
    
    # Combined Legend
    lns_v = lns3 + lns4
    labs_v = [l.get_label() for l in lns_v]
    ax3.legend(lns_v, labs_v, loc='center left', bbox_to_anchor=(1.1, 0.5), fontsize='x-small')


    # --- 2,2 (Bottom-Right): Joint Space Velocities ---
    ax4 = fig.add_subplot(2, 2, 4)
    for i in range(6):
        ax4.plot(time, np.rad2deg(q_dot_traj[:, i]), label=f'dq{i+1}')
    ax4.set_title("Joint Velocities (deg/s)")
    ax4.set_xlabel("Time (s)")
    ax4.set_ylabel("Velocity (deg/s)")
    ax4.legend(ncol=3, fontsize='small')
    ax4.grid(True)
    add_viapoints(ax4)
    
    plt.tight_layout()
    plt.show()
