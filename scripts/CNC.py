import numpy as np
from typing import List, Optional, Tuple
from scipy.spatial.transform import Rotation as R
from numpy import pi

from robot_kinematics import RobotKinematics
from velocity_profiles import LSPBProfile


# --- CNC Interpolation & Planning ---
class CartesianSegment:
    """
    Represents a single motion segment from P_start to P_end
    interpolated in Cartesian space.
    """
    def __init__(self, start_pose: np.ndarray, end_pose: np.ndarray, 
                 vel: float, mode: str = 'exact_stop'):
        self.start_pose = start_pose
        self.end_pose = end_pose
        self.mode = mode
        
        # Decompose Poses
        self.p_start = start_pose[:3,3]
        self.p_end = end_pose[:3,3]
        self.R_start = start_pose[:3,:3]
        self.R_end = end_pose[:3,:3]
        
        # Geometry Calculation
        self.dist_vec = self.p_end - self.p_start
        self.linear_dist = np.linalg.norm(self.dist_vec)
        self.direction = self.dist_vec / self.linear_dist if self.linear_dist > 1e-6 else np.zeros(3)
        
        # Orientation Difference (Global)
        # R_end = R_diff * R_start  => R_diff = R_end * R_start^T
        self.R_diff = self.R_end @ self.R_start.T
        self.rot_vec_total = R.from_matrix(self.R_diff).as_rotvec()
        self.angular_dist = np.linalg.norm(self.rot_vec_total)
        self.axis = self.rot_vec_total / self.angular_dist if self.angular_dist > 1e-6 else np.zeros(3)
        
        self.profile = LSPBProfile()
        self.profile.generate(np.array([self.linear_dist, self.angular_dist]), vel)
        
    def get_state(self, t: float) -> Tuple[np.ndarray, np.ndarray]:
        """
        Get the Cartesian state (Pose, Velocity) at time step.
        Returns: (T_des, v_des, is_done)
        """
        s, s_dot = self.profile.s(t)
        
        # Calculate Linear Position
        p_curr = self.p_start + s[0] * self.direction
        v = s_dot[0] * self.direction

        # Calculate Angular Velocity
        r_curr = self.R_start @ R.from_rotvec(s[1] * self.axis).as_matrix()
        w = s_dot[1] * self.axis

        # Construct Result
        T_curr = np.eye(4)
        T_curr[:3,3] = p_curr
        T_curr[:3,:3] = r_curr
        
        v_curr = np.concatenate([v, w])
        
        return T_curr, v_curr

class CNCPlanner:
    """
    Simplified Path Planner.
    Accepts via-points and generates a sequence of Cartesian Segments.
    """
    def __init__(self):
        self.via_points = [] # List of (Pose, v_max, a_max, mode)
        
    def add_move(self, position: np.ndarray, euler_rpy: List[float], 
                 vel: float = 0.1, mode: str = 'exact_stop'):
        """
        Add a target waypoint.
        mode: 'exact_stop' (G61) or 'continuous' (G64)
        """
        R_mat = R.from_euler('xyz', euler_rpy).as_matrix()
        T = np.eye(4)
        T[:3, :3] = R_mat
        T[:3, 3] = position
        self.via_points.append({'pose': T, 'v': vel, 'mode': mode})

    def plan(self, start_pose: np.ndarray) -> List[CartesianSegment]:
        """
        Generate motion segments from start_pose through all via_points.
        """
        segments = []
        current_pose = start_pose.copy()
        
        for wp in self.via_points:
            target_pose = wp['pose']
            v = wp['v']
            mode = wp['mode']
            
            segment = CartesianSegment(current_pose, target_pose, v, mode)
            segments.append(segment)
            
            current_pose = target_pose
            
        return segments

class CNCController:
    """
    Executes the generated segments on the robot using Differential IK.
    Simulates the real-time control loop.
    """
    def __init__(self, robot: RobotKinematics, dt: float = 0.01):
        self.robot = robot
        self.dt = dt
        
    def execute(self, segments: List[CartesianSegment], start_q: Optional[np.ndarray] = None) -> Tuple:
        """
        Run the control loop.
        """
        # Get initial state from first segment
        T0, vd0 = segments[0].get_state(0)
        
        # Initialize logs with STARTING state (t=0)
        time_log = [0.0]
        q_log = [self.robot.home_config if start_q is None else start_q.copy()]
        q_dot_log = [np.zeros(6)]
        Td_log = [T0]
        vd_log = [vd0]
        v_act_log = [np.zeros(6)]
        
        segment_indices = []
        
        current_q = self.robot.home_config if start_q is None else start_q.copy()
        current_time = 0.0
        
        print(f"Starting CNC execution. Processing {len(segments)} blocks.")
        
        for segment in segments:
            
            t = self.dt
            while True:
                # 1. Get Setpoint (Interpolator)
                Td, vd = segment.get_state(t)
                
                # 2. Check Execution Status
                Te, _ = self.robot.forward_kinematics(current_q)
                
                # Calculate Error relative to the CURRENT setpoint
                p_err = np.linalg.norm(Td[:3,3] - Te[:3,3])
                
                # Orientation Error for exit condition
                R_err = Td[:3,:3] @ Te[:3,:3].T
                rot_vec = R.from_matrix(R_err).as_rotvec()
                angle_err = np.linalg.norm(rot_vec)
                
                # Exit Condition
                if t > segment.profile.T:
                    if segment.mode == 'continuous':
                         print(f"    Continuous transition (G64). Pos Error: {p_err*1000:.3f}mm")
                         segment_indices.append(len(time_log) - 1)
                         break
                    if p_err < 1e-4 and angle_err < np.radians(0.01):
                        segment_indices.append(len(time_log) - 1)
                        break
                
                # 3. Control Step (Differential IK)
                next_q, q_dot = self.robot.differential_inverse_kinematics_step(current_q, Td, vd, self.dt)
                
                # Calculate Actual Task Velocity
                J = self.robot.jacobian(current_q)
                v_actual = J @ q_dot
                
                current_q = next_q
                current_time += self.dt
                t += self.dt
                
                # Logging
                time_log.append(current_time)
                q_log.append(current_q)
                q_dot_log.append(q_dot)
                vd_log.append(vd)
                v_act_log.append(v_actual)
                Td_log.append(Td.copy())
                
        return (np.array(time_log), np.array(q_log), np.array(q_dot_log),
                np.array(vd_log), np.array(v_act_log), Td_log, segment_indices)


# --- Simple Geometric Trajectories ---
class GeometryTrajectory:
    @staticmethod
    def circle(robot: RobotKinematics, planner: CNCPlanner, radius, n_points):
        """
        Generate points on a circle in the YZ plane.
        """
        center = robot.get_home_pose()[:3,3] - np.array([0, 0, radius])
        orientation = [0, pi, pi]
        for i in range(n_points):
            angle = 2 * np.pi * i / n_points
            x = center[0]
            y = center[1] + radius * np.cos(angle)
            z = center[2] + radius * np.sin(angle)
            planner.add_move(
                position=np.array([x, y, z]),
                euler_rpy=orientation,
                vel=0.1, mode='continuous'
            )
    
    @staticmethod
    def rectangle(robot: RobotKinematics, planner: CNCPlanner, width, height):
        """
        Generate rectangle waypoints.
        """
        start_pos = robot.get_home_pose()[:3,3]
        positions = np.array([start_pos + np.array([0, -width/2, 0]),
                         start_pos + np.array([0, -width/2, -height]),
                         start_pos + np.array([0,  width/2, -height]),
                         start_pos + np.array([0,  width/2, 0]),
                         start_pos])
        orientation = [0, pi, pi]
        for pos in positions:
            planner.add_move(
                position=pos,
                euler_rpy=orientation,
                vel=0.1, mode='exact_stop'
            )


if __name__ == "__main__":
    import matplotlib.pyplot as plt
    import warnings
    warnings.filterwarnings('ignore', message='Gimbal lock detected')
    
    # Create realistic example segment
    T_start = np.eye(4)
    T_start[:3, :3] = R.from_euler('xyz', [0, pi/2, 0]).as_matrix()
    T_start[:3, 3] = np.array([0.5, 0.2, 0.6])
    
    T_end = np.eye(4)
    T_end[:3, :3] = R.from_euler('xyz', [pi/4, pi/3, pi/6]).as_matrix()
    T_end[:3, 3] = np.array([0.7, 0.5, 0.5])
    
    segment = CartesianSegment(T_start, T_end, vel=1.0, mode='exact_stop')
    
    # Sample trajectory
    dt = 0.02
    n_samples = int(segment.profile.T / dt) + 1
    t_vals = np.linspace(0, segment.profile.T, n_samples)
    positions = np.zeros((n_samples, 3))
    velocities = np.zeros((n_samples, 3))
    orientations = np.zeros((n_samples, 3))
    angular_vels = np.zeros((n_samples, 3))
    
    for i, t in enumerate(t_vals):
        T, v = segment.get_state(t)
        positions[i] = T[:3, 3]
        velocities[i] = v[:3]
        orientations[i] = R.from_matrix(T[:3, :3]).as_euler('xyz')
        angular_vels[i] = v[3:]
    
    # Create comprehensive visualization
    fig = plt.figure(figsize=(15, 10))
    
    # 3D trajectory
    ax1 = fig.add_subplot(2, 3, 1, projection='3d')
    ax1.plot(positions[:, 0], positions[:, 1], positions[:, 2], 'b-', linewidth=2)
    ax1.scatter(*segment.p_start, c='g', s=100, marker='o', label='Start')
    ax1.scatter(*segment.p_end, c='r', s=100, marker='s', label='End')
    ax1.set_xlabel('X [m]')
    ax1.set_ylabel('Y [m]')
    ax1.set_zlabel('Z [m]')
    ax1.set_title('3D Cartesian Path')
    ax1.legend()
    ax1.grid(True)
    
    # Position vs time
    ax2 = fig.add_subplot(2, 3, 2)
    ax2.plot(t_vals, positions[:, 0], c='r', label='X')
    ax2.plot(t_vals, positions[:, 1], c='g', label='Y')
    ax2.plot(t_vals, positions[:, 2], c='b', label='Z')
    ax2.set_xlabel('Time [s]')
    ax2.set_ylabel('Position [m]')
    ax2.set_title('Position vs Time')
    ax2.legend()
    ax2.grid(True)
    
    # Linear velocity
    ax3 = fig.add_subplot(2, 3, 3)
    ax3.plot(t_vals, velocities[:, 0], c='r', label='Vx')
    ax3.plot(t_vals, velocities[:, 1], c='g', label='Vy')
    ax3.plot(t_vals, velocities[:, 2], c='b', label='Vz')
    ax3.plot(t_vals, np.linalg.norm(velocities, axis=1), 'k--', label='|V|', linewidth=2)
    ax3.set_xlabel('Time [s]')
    ax3.set_ylabel('Velocity [m/s]')
    ax3.set_title('Linear Velocity Profile')
    ax3.legend()
    ax3.grid(True)
    
    # Orientation
    ax4 = fig.add_subplot(2, 3, 4)
    ax4.plot(t_vals, np.degrees(orientations[:, 0]), c='r', label='Roll')
    ax4.plot(t_vals, np.degrees(orientations[:, 1]), c='g', label='Pitch')
    ax4.plot(t_vals, np.degrees(orientations[:, 2]), c='b', label='Yaw')
    ax4.set_xlabel('Time [s]')
    ax4.set_ylabel('Angle [deg]')
    ax4.set_title('Orientation (Euler XYZ)')
    ax4.legend()
    ax4.grid(True)
    
    # Angular velocity
    ax5 = fig.add_subplot(2, 3, 5)
    ax5.plot(t_vals, angular_vels[:, 0], c='r', label='ωx')
    ax5.plot(t_vals, angular_vels[:, 1], c='g', label='ωy')
    ax5.plot(t_vals, angular_vels[:, 2], c='b', label='ωz')
    ax5.plot(t_vals, np.linalg.norm(angular_vels, axis=1), 'k--', label='|ω|', linewidth=2)
    ax5.set_xlabel('Time [s]')
    ax5.set_ylabel('Angular Velocity [rad/s]')
    ax5.set_title('Angular Velocity Profile')
    ax5.legend()
    ax5.grid(True)
    
    # Segment info
    ax6 = fig.add_subplot(2, 3, 6)
    ax6.axis('off')
    info_text = f"""Segment Information:
    
Mode: {segment.mode}
Linear Distance: {segment.linear_dist:.4f} m
Angular Distance: {np.degrees(segment.angular_dist):.2f}°
Duration: {segment.profile.T:.3f} s
Max Velocity: {0.2:.3f} m/s

Direction: [{segment.direction[0]:.3f}, {segment.direction[1]:.3f}, {segment.direction[2]:.3f}]
Rotation Axis: [{segment.axis[0]:.3f}, {segment.axis[1]:.3f}, {segment.axis[2]:.3f}]

Start: [{segment.p_start[0]:.3f}, {segment.p_start[1]:.3f}, {segment.p_start[2]:.3f}]
End: [{segment.p_end[0]:.3f}, {segment.p_end[1]:.3f}, {segment.p_end[2]:.3f}]
    """
    ax6.text(0.1, 0.5, info_text, fontsize=10, family='monospace', verticalalignment='center')
    ax6.set_title('Segment Parameters', fontweight='bold')
    
    plt.tight_layout()
    plt.show()
    
    print("\n=== CartesianSegment Debug Visualization ===")
    print(f"Linear distance: {segment.linear_dist:.4f} m")
    print(f"Angular distance: {np.degrees(segment.angular_dist):.2f}°")
    print(f"Total duration: {segment.profile.T:.3f} s")
    print(f"Direction vector: {segment.direction}")
    print(f"Rotation axis: {segment.axis}")
    
    # === CNCPlanner Tests ===
    print("\n=== CNCPlanner Tests ===")
    
    planner = CNCPlanner()
    start = np.array([0.5, 0.2, 0.6])
    
    # Add waypoints for L-shaped path
    planner.add_move(start + np.array([0.2, 0, 0]), [0, pi/2, 0], vel=0.15, mode='exact_stop')
    planner.add_move(start + np.array([0.2, 0.3, 0]), [pi/4, pi/2, 0], vel=0.2, mode='continuous')
    planner.add_move(start + np.array([0.2, 0.3, -0.2]), [pi/4, pi/2, pi/6], vel=0.1, mode='exact_stop')
    
    T_start_plan = np.eye(4)
    T_start_plan[:3, :3] = R.from_euler('xyz', [0, pi/2, 0]).as_matrix()
    T_start_plan[:3, 3] = start
    
    segments = planner.plan(T_start_plan)
    
    print(f"Generated {len(segments)} segments")
    for i, seg in enumerate(segments):
        print(f"  Segment {i+1}: {seg.linear_dist:.3f}m, {np.degrees(seg.angular_dist):.1f}°, {seg.profile.T:.2f}s, mode={seg.mode}")
    
    # Sample all segments
    colors = ['b', 'g', 'orange']
    all_pos, all_vel, all_ori, all_ang_vel, all_t = [], [], [], [], []
    t_cumulative = 0
    
    for seg in segments:
        t_seg = np.linspace(0, seg.profile.T, 50)
        for t in t_seg:
            T, v = seg.get_state(t)
            all_pos.append(T[:3, 3])
            all_vel.append(v[:3])
            all_ori.append(R.from_matrix(T[:3, :3]).as_euler('xyz'))
            all_ang_vel.append(v[3:])
            all_t.append(t_cumulative + t)
        t_cumulative += seg.profile.T
    
    all_pos = np.array(all_pos)
    all_vel = np.array(all_vel)
    all_ori = np.array(all_ori)
    all_ang_vel = np.array(all_ang_vel)
    all_t = np.array(all_t)
    
    # Create detailed visualization
    fig2 = plt.figure(figsize=(18, 10))
    
    # 3D path
    ax1 = fig2.add_subplot(2, 4, 1, projection='3d')
    t_cum = 0
    for i, seg in enumerate(segments):
        t_seg = np.linspace(0, seg.profile.T, 50)
        pos_seg = np.array([seg.get_state(t)[0][:3, 3] for t in t_seg])
        ax1.plot(pos_seg[:, 0], pos_seg[:, 1], pos_seg[:, 2], c=colors[i], linewidth=2, label=f'S{i+1}')
        ax1.scatter(*seg.p_start, c=colors[i], s=80, marker='o')
    ax1.scatter(*segments[-1].p_end, c='r', s=100, marker='s')
    ax1.set_xlabel('X [m]')
    ax1.set_ylabel('Y [m]')
    ax1.set_zlabel('Z [m]')
    ax1.set_title('3D Path')
    ax1.legend()
    ax1.grid(True)
    
    # XYZ Position
    ax2 = fig2.add_subplot(2, 4, 2)
    ax2.plot(all_t, all_pos[:, 0], 'r-', label='X', linewidth=1.5)
    ax2.plot(all_t, all_pos[:, 1], 'g-', label='Y', linewidth=1.5)
    ax2.plot(all_t, all_pos[:, 2], 'b-', label='Z', linewidth=1.5)
    t_cum = 0
    for seg in segments:
        t_cum += seg.profile.T
        ax2.axvline(t_cum, c='k', ls='--', alpha=0.3, linewidth=0.8)
    ax2.set_xlabel('Time [s]')
    ax2.set_ylabel('Position [m]')
    ax2.set_title('XYZ Position')
    ax2.legend()
    ax2.grid(True, alpha=0.3)
    
    # XYZ Velocity
    ax3 = fig2.add_subplot(2, 4, 3)
    ax3.plot(all_t, all_vel[:, 0], 'r-', label='Vx', linewidth=1.5)
    ax3.plot(all_t, all_vel[:, 1], 'g-', label='Vy', linewidth=1.5)
    ax3.plot(all_t, all_vel[:, 2], 'b-', label='Vz', linewidth=1.5)
    ax3.plot(all_t, np.linalg.norm(all_vel, axis=1), 'k--', label='|V|', linewidth=2)
    t_cum = 0
    for seg in segments:
        t_cum += seg.profile.T
        ax3.axvline(t_cum, c='k', ls='--', alpha=0.3, linewidth=0.8)
    ax3.set_xlabel('Time [s]')
    ax3.set_ylabel('Velocity [m/s]')
    ax3.set_title('Linear Velocity')
    ax3.legend()
    ax3.grid(True, alpha=0.3)
    
    # Velocity magnitude detail
    ax4 = fig2.add_subplot(2, 4, 4)
    t_cum = 0
    for i, seg in enumerate(segments):
        t_seg = np.linspace(0, seg.profile.T, 50)
        v_mag = np.array([np.linalg.norm(seg.get_state(t)[1][:3]) for t in t_seg])
        ax4.plot(t_cum + t_seg, v_mag, c=colors[i], linewidth=2, label=f'S{i+1}')
        ax4.axvline(t_cum, c='k', ls='--', alpha=0.3, linewidth=0.8)
        t_cum += seg.profile.T
    ax4.set_xlabel('Time [s]')
    ax4.set_ylabel('|V| [m/s]')
    ax4.set_title('Velocity Magnitude (LSPB)')
    ax4.legend()
    ax4.grid(True, alpha=0.3)
    
    # Orientation (Euler)
    ax5 = fig2.add_subplot(2, 4, 5)
    ax5.plot(all_t, np.degrees(all_ori[:, 0]), 'r-', label='Roll', linewidth=1.5)
    ax5.plot(all_t, np.degrees(all_ori[:, 1]), 'g-', label='Pitch', linewidth=1.5)
    ax5.plot(all_t, np.degrees(all_ori[:, 2]), 'b-', label='Yaw', linewidth=1.5)
    t_cum = 0
    for seg in segments:
        t_cum += seg.profile.T
        ax5.axvline(t_cum, c='k', ls='--', alpha=0.3, linewidth=0.8)
    ax5.set_xlabel('Time [s]')
    ax5.set_ylabel('Angle [deg]')
    ax5.set_title('Orientation (RPY)')
    ax5.legend()
    ax5.grid(True, alpha=0.3)
    
    # Angular Velocity
    ax6 = fig2.add_subplot(2, 4, 6)
    ax6.plot(all_t, all_ang_vel[:, 0], 'r-', label='ωx', linewidth=1.5)
    ax6.plot(all_t, all_ang_vel[:, 1], 'g-', label='ωy', linewidth=1.5)
    ax6.plot(all_t, all_ang_vel[:, 2], 'b-', label='ωz', linewidth=1.5)
    ax6.plot(all_t, np.linalg.norm(all_ang_vel, axis=1), 'k--', label='|ω|', linewidth=2)
    t_cum = 0
    for seg in segments:
        t_cum += seg.profile.T
        ax6.axvline(t_cum, c='k', ls='--', alpha=0.3, linewidth=0.8)
    ax6.set_xlabel('Time [s]')
    ax6.set_ylabel('Angular Vel [rad/s]')
    ax6.set_title('Angular Velocity')
    ax6.legend()
    ax6.grid(True, alpha=0.3)
    
    # Angular velocity magnitude
    ax7 = fig2.add_subplot(2, 4, 7)
    t_cum = 0
    for i, seg in enumerate(segments):
        t_seg = np.linspace(0, seg.profile.T, 50)
        w_mag = np.array([np.linalg.norm(seg.get_state(t)[1][3:]) for t in t_seg])
        ax7.plot(t_cum + t_seg, w_mag, c=colors[i], linewidth=2, label=f'S{i+1}')
        ax7.axvline(t_cum, c='k', ls='--', alpha=0.3, linewidth=0.8)
        t_cum += seg.profile.T
    ax7.set_xlabel('Time [s]')
    ax7.set_ylabel('|ω| [rad/s]')
    ax7.set_title('Angular Velocity Magnitude')
    ax7.legend()
    ax7.grid(True, alpha=0.3)
    
    # Summary info
    ax8 = fig2.add_subplot(2, 4, 8)
    ax8.axis('off')
    total_dist = sum(s.linear_dist for s in segments)
    total_ang = sum(s.angular_dist for s in segments)
    total_time = sum(s.profile.T for s in segments)
    info = f"""Path Summary:

Segments: {len(segments)}
Total Distance: {total_dist*1000:.1f} mm
Total Rotation: {np.degrees(total_ang):.1f}°
Total Time: {total_time:.3f} s
Avg Velocity: {total_dist/total_time:.3f} m/s

Per Segment:
"""
    for i, seg in enumerate(segments):
        info += f"\n{i+1}. {seg.linear_dist*1000:.1f}mm, {np.degrees(seg.angular_dist):.1f}°"
        v_max = np.max(seg.profile.v_cruise) if hasattr(seg.profile.v_cruise, '__len__') else seg.profile.v_cruise
        info += f"\n   v={v_max:.2f}m/s, T={seg.profile.T:.2f}s"
        info += f"\n   mode: {seg.mode}"
    ax8.text(0.05, 0.5, info, fontsize=9, family='monospace', verticalalignment='center')
    ax8.set_title('Statistics', fontweight='bold')
    
    plt.tight_layout()
    plt.show()
    
    print(f"\nTotal path: {total_dist*1000:.1f}mm, {np.degrees(total_ang):.1f}° in {total_time:.3f}s")
