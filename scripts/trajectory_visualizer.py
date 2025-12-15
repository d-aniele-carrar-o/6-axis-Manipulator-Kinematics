
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from scipy.spatial.transform import Rotation as R
from typing import Dict, Tuple, List, Optional

from robot_kinematics import RobotKinematics
from CNC import CartesianSegment


def unwrap_euler(euler_angles: np.ndarray, discont: float = np.pi) -> np.ndarray:
    """
    Unwrap Euler angles to prevent jumps when crossing +/- 180 degrees.
    Works by checking differences between consecutive frames.
    """
    unwrapped = euler_angles.copy()
    for i in range(3):
        unwrapped[:, i] = np.unwrap(unwrapped[:, i], discont=discont)
    return unwrapped


class TrajectoryVisualizer:
    """Handles all visualization for robot trajectories."""
    
    def __init__(self, robot: RobotKinematics, 
                 frame_scale: float = 0.05,
                 animation_interval: int = 100,
                 speedup: int = 1,
                 figsize: Tuple[int, int] = (16, 12)):
        """
        Initialize trajectory visualizer.
        
        Args:
            robot: RobotKinematics instance
            frame_scale: Scale factor for coordinate frame visualization
            animation_interval: Animation frame interval in milliseconds
            speedup: Frame skipping factor (1 = show all frames, 2 = show every 2nd, etc.)
            figsize: Figure size for plots
        """
        self.robot = robot
        self.frame_scale = frame_scale
        self.animation_interval = animation_interval
        self.speedup = max(1, int(speedup))
        self.figsize = figsize
    
    def plot_frame(self, ax, T: np.ndarray, scale: Optional[float] = None):
        """
        Plot coordinate frame axes.
        
        Args:
            ax: 3D matplotlib axis
            T: 4x4 transformation matrix
            scale: Optional scale override
        """
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
        """
        Precompute all data needed for visualization.
        
        Args:
            q_trajectory: List of joint angle arrays
            desired_trajectory: List of desired end-effector transforms
            
        Returns:
            Dictionary containing precomputed data
        """
        all_joint_positions = []
        all_ee_transforms = []
        actual_ee_positions = []
        
        for q in q_trajectory:
            transforms = self.robot.forward_kinematics_full(q)
            joint_pos_list = [np.zeros(3)] + [T[:3, 3] for T in transforms]
            all_joint_positions.append(np.array(joint_pos_list))
            all_ee_transforms.append(transforms[-1])
            actual_ee_positions.append(transforms[-1][:3, 3])
        
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
                           segments: List[CartesianSegment]
                           ) -> FuncAnimation:
        """
        Animate robot trajectory with visualization of desired vs actual path.
        
        Args:
            q_trajectory: List of joint angle arrays
            segments: List of CartesianSegment objects
            
        Returns:
            FuncAnimation object
        """
        # Apply speedup
        q_display = q_trajectory[::self.speedup]

        # Prepare desired trajectory for visualization
        desired_trajectory = []
        desired_trajectory.append(self.robot.get_end_effector_pose())
        for seg in segments:
            desired_trajectory.append(seg.end_pose)
        
        data = self._precompute_trajectory_data(q_display, desired_trajectory)
        
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
            
            # Show real frame index
            real_frame = frame * self.speedup
            ax.set_title(f'Frame {real_frame}/{len(q_trajectory)} (Speedup: {self.speedup}x)')
            ax.legend(loc='upper right')
            ax.grid(True)
            
            # Error text
            err_val = np.linalg.norm(curr_des - curr_act)
            ax.text2D(0.05, 0.95, f"Error: {err_val*1000:.1f} mm", 
                     transform=ax.transAxes,
                     bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.8))
            
            return []
        
        anim = FuncAnimation(fig, update, frames=len(q_display), 
                           interval=self.animation_interval, repeat=True)
        plt.tight_layout()
        plt.show()
        return anim
    
    def plot_analysis(self, q_trajectory: List[np.ndarray], 
                     q_dot_trajectory: List[np.ndarray],
                     desired_trajectory: List[np.ndarray],
                     desired_velocities: Optional[np.ndarray],
                     actual_task_velocities: np.ndarray,
                     segment_indices: Optional[List[int]] = None,
                     time_log: Optional[np.ndarray] = None):
        """
        Plot detailed analysis of the trajectory including velocities.
        
        Args:
            q_trajectory: List of joint angle arrays
            q_dot_trajectory: List of joint velocity arrays
            desired_trajectory: List of desired end-effector transforms
            desired_velocities: Array of desired task velocities (N, 6)
            actual_task_velocities: Array of actual task velocities (N, 6)
            segment_indices: Optional list of indices where new segments start/end
            time_log: Optional array of time stamps for x-axis
        """
        # Convert desired trajectory to arrays
        desired_pos = np.array([T[:3, 3] for T in desired_trajectory])
        
        desired_rpy = []
        for T in desired_trajectory:
            r = R.from_matrix(T[:3, :3])
            # Convention used across the project:
            # - Waypoints specify RPY = [roll, pitch, yaw] in radians.
            # - Rotation is built as R = Rz(yaw) * Ry(pitch) * Rx(roll).
            # - In SciPy, this corresponds to extrinsic 'xyz' with angles [roll, pitch, yaw].
            euler = r.as_euler('xyz') 
            desired_rpy.append(euler)
        desired_rpy = np.array(desired_rpy)
        desired_rpy = unwrap_euler(desired_rpy) # Unwrap desired
        
        # Compute Actual EE Position & Orientation from q_trajectory
        actual_ee_positions = []
        actual_rpy = []
        for q in q_trajectory:
            transforms = self.robot.forward_kinematics_full(q)
            actual_ee_positions.append(transforms[-1][:3, 3])
            r = R.from_matrix(transforms[-1][:3, :3])
            euler = r.as_euler('xyz')
            actual_rpy.append(euler)
            
        actual_ee_positions = np.array(actual_ee_positions)
        actual_rpy = np.array(actual_rpy)
        actual_rpy = unwrap_euler(actual_rpy) # Unwrap actual
        
        # Prepare X-axis data
        if time_log is not None:
            x_data = time_log
            x_label = 'Time (s)'
        else:
            x_data = np.arange(len(q_trajectory))
            x_label = 'Sample'
            
        # Helper to plot vertical lines
        def plot_segment_lines(ax):
            if segment_indices:
                for idx in segment_indices:
                    # If using time, map index to time
                    if time_log is not None and idx < len(time_log):
                        val = time_log[idx]
                    else:
                        val = idx
                    ax.axvline(x=val, color='k', linestyle=':', alpha=0.5)

        # Create 2x2 subplots
        fig, axs = plt.subplots(2, 2, figsize=(16, 10))
        (ax1, ax2), (ax3, ax4) = axs
        
        # 1. Top-Left: Joint Angles
        q_array = np.array(q_trajectory)
        for i in range(6):
            ax1.plot(x_data, np.rad2deg(q_array[:, i]), label=f'Joint {i+1}', linewidth=2)
        plot_segment_lines(ax1)
        ax1.set_title('Joint Angles', fontsize=12)
        ax1.set_ylabel('Angle (deg)', fontsize=10)
        ax1.set_xlabel(x_label, fontsize=10)
        ax1.legend(loc='best', fontsize='x-small', ncol=3)
        ax1.grid(True, alpha=0.3)
        
        # 2. Bottom-Left: Joint Velocities
        q_dot_array = np.array(q_dot_trajectory)
        for i in range(6):
            ax3.plot(x_data, np.rad2deg(q_dot_array[:, i]), label=f'Joint {i+1}', linewidth=2)
        plot_segment_lines(ax3)
        ax3.set_title('Joint Velocities', fontsize=12)
        ax3.set_ylabel('Velocity (deg/s)', fontsize=10)
        ax3.set_xlabel(x_label, fontsize=10)
        ax3.legend(loc='best', fontsize='x-small', ncol=3)
        ax3.grid(True, alpha=0.3)
        
        # 3. Top-Right: End-effector Position & Orientation
        colors_light = ['r', 'g', 'b']  # For XYZ
        colors_dark = ['darkred', 'darkgreen', 'darkblue']  # For RPY
        pos_labels = ['X', 'Y', 'Z']
        rpy_labels = ['Roll', 'Pitch', 'Yaw']
        
        for i in range(3):
            ax2.plot(x_data, desired_pos[:, i], '--', color=colors_light[i], 
                    linewidth=1.5, alpha=0.6, label=f'Des {pos_labels[i]}')
            ax2.plot(x_data, actual_ee_positions[:, i], '-', color=colors_light[i], 
                    linewidth=2, label=f'Act {pos_labels[i]}')
        
        ax2_twin = ax2.twinx()
        for i in range(3):
            ax2_twin.plot(x_data, np.rad2deg(desired_rpy[:, i]), ':', color=colors_dark[i], 
                         linewidth=1.5, alpha=0.6, label=f'Des {rpy_labels[i]}')
            ax2_twin.plot(x_data, np.rad2deg(actual_rpy[:, i]), '-.', color=colors_dark[i], 
                         linewidth=2, label=f'Act {rpy_labels[i]}')
        
        plot_segment_lines(ax2)
        ax2.set_title('EE Position & Orientation', fontsize=12)
        ax2.set_ylabel('Position (m)', fontsize=10)
        ax2_twin.set_ylabel('Orientation (deg)', fontsize=10)
        ax2.set_xlabel(x_label, fontsize=10)
        ax2.grid(True, alpha=0.3)
        
        # 4. Bottom-Right: Task Velocities
        # Linear (v) and Angular (w)
        # We can plot magnitude or components. Let's plot components.
        
        # If desired velocities are available
        if desired_velocities is not None:
             # Just plot Linear Velocity Norm and Angular Velocity Norm to avoid clutter?
             # Or plot components? Let's plot components for Linear (X,Y,Z) and Angular (Wx, Wy, Wz)
             
             # Plot Linear
             for i in range(3):
                 ax4.plot(x_data, desired_velocities[:, i], '--', color=colors_light[i], 
                         linewidth=1.5, alpha=0.6, label=f'Des V{pos_labels[i].lower()}')
                 ax4.plot(x_data, actual_task_velocities[:, i], '-', color=colors_light[i], 
                         linewidth=2, label=f'Act V{pos_labels[i].lower()}')
             
             # Plot Angular on secondary axis
             ax4_twin = ax4.twinx()
             for i in range(3):
                 ax4_twin.plot(x_data, desired_velocities[:, i+3], ':', color=colors_dark[i], 
                              linewidth=1.5, alpha=0.6, label=f'Des W{pos_labels[i].lower()}')
                 ax4_twin.plot(x_data, actual_task_velocities[:, i+3], '-.', color=colors_dark[i], 
                              linewidth=2, label=f'Act W{pos_labels[i].lower()}')
             
             ax4_twin.set_ylabel('Angular Vel (rad/s)', fontsize=10)
        else:
             # Just plot Actual
             for i in range(3):
                 ax4.plot(x_data, actual_task_velocities[:, i], '-', color=colors_light[i], 
                         linewidth=2, label=f'Act V{pos_labels[i].lower()}')
             
             ax4_twin = ax4.twinx()
             for i in range(3):
                 ax4_twin.plot(x_data, actual_task_velocities[:, i+3], '-.', color=colors_dark[i], 
                              linewidth=2, label=f'Act W{pos_labels[i].lower()}')
             ax4_twin.set_ylabel('Angular Vel (rad/s)', fontsize=10)

        plot_segment_lines(ax4)
        ax4.set_title('Task Velocities', fontsize=12)
        ax4.set_ylabel('Linear Vel (m/s)', fontsize=10)
        ax4.set_xlabel(x_label, fontsize=10)
        ax4.grid(True, alpha=0.3)
        
        # Combine legends for ax2 and ax4
        # (Simplified to avoid too many labels, but show at least types)
        
        plt.tight_layout()
        plt.show()
