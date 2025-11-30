import numpy as np
from typing import List, Optional, Tuple
from scipy.spatial.transform import Slerp, Rotation as R
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
        # Use the dominant motion (linear or angular) for the time law
        
        if self.linear_dist < 1e-6:
            # Pure rotation or no motion
            motion_mag = self.total_angle
            self.is_pure_rotation = True
        else:
            motion_mag = self.linear_dist
            self.is_pure_rotation = False
            
        self.profile = LSPBProfile()
        self.times, self.s, self.s_dot, _ = self.profile.generate(float(motion_mag), v_max, a_max, dt)
        
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
        else:
            # Use clamped u to prevent overshoot beyond P_end
            p_curr = self.p_start + self.direction * (u * motion_len)
            v_lin = self.direction * s_dot
            
        # Interpolate Orientation (SLERP)
        r_curr = self.slerp(u)
        
        # Calculate Angular Velocity
        # For mixed motion: scale angular velocity by the same profile
        # Angular velocity = rotation_axis * angular_speed
        if self.total_angle > 1e-6:
            if self.is_pure_rotation:
                # Pure rotation: use s_dot directly as angular speed
                angular_speed = s_dot
            else:
                # Mixed motion: scale angular motion by linear velocity profile
                angular_speed = s_dot * (self.total_angle / self.linear_dist)
            
            # Normalize rotation vector to get axis, then scale by speed
            rotation_axis = self.rot_vec_total / self.total_angle
            w_vec = rotation_axis * angular_speed
        else:
            w_vec = np.zeros(3)
        
        # Construct Result
        T_curr = np.eye(4)
        T_curr[:3,3] = p_curr
        T_curr[:3,:3] = r_curr.as_matrix()
        
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
        
    def execute(self, segments: List[CartesianSegment], start_q: Optional[np.ndarray] = None) -> Tuple:
        """
        Run the control loop.
        """
        # Get initial state from first segment
        T0, vd0, _ = segments[0].get_state(0)
        
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
        
        for i, segment in enumerate(segments):
            print(f"  > Block {i+1}: Dist={segment.linear_dist:.3f}m, Planned Time={segment.times[-1]:.2f}s")
            
            idx = 1
            while True:
                # 1. Get Setpoint (Interpolator)
                Td, vd, is_done_profile = segment.get_state(idx)
                
                # 2. Check Execution Status
                Te, _ = self.robot.forward_kinematics(current_q)
                
                # Calculate Error relative to the CURRENT setpoint
                p_err = np.linalg.norm(Td[:3,3] - Te[:3,3])
                
                # Orientation Error for exit condition
                R_err = Td[:3,:3] @ Te[:3,:3].T
                rot_vec = R.from_matrix(R_err).as_rotvec()
                angle_err = np.linalg.norm(rot_vec)
                
                # Exit Condition
                if is_done_profile:
                    if segment.mode == 'continuous':
                         print(f"    Continuous transition (G64). Pos Error: {p_err*1000:.3f}mm")
                         segment_indices.append(len(time_log) - 1)
                         break
                    if p_err < 1e-4 and angle_err < np.radians(0.01):
                        segment_indices.append(len(time_log) - 1)
                        break
                
                # 3. Control Step (Differential IK)
                dt = segment.dt
                next_q, q_dot = self.robot.differential_inverse_kinematics_step(current_q, Td, vd, dt)
                
                # Calculate Actual Task Velocity
                J = self.robot.jacobian(current_q)
                v_actual = J @ q_dot
                
                current_q = next_q
                current_time += dt
                
                # Logging
                time_log.append(current_time)
                q_log.append(current_q)
                q_dot_log.append(q_dot)
                vd_log.append(vd)
                v_act_log.append(v_actual)
                
                Td_log.append(Td.copy())
                
                idx += 1
                
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
                v_max=0.1, a_max=0.5,
                mode='continuous'
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
                v_max=0.1, a_max=0.5,
                mode='exact_stop'
            )
