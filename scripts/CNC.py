import numpy as np
from typing import List, Optional, Tuple
from scipy.spatial.transform import Rotation as R
from numpy import pi

from robot_kinematics import RobotKinematics
from velocity_profiles import LSPBProfile, SCurveProfile


# --- CNC Interpolation & Planning ---
class CartesianSegment:
    """
    Represents a single motion segment from P_start to P_end
    interpolated in Cartesian space.
    """
    def __init__(self, start_pose: np.ndarray, end_pose: np.ndarray, 
                 vel: List[float], velocity_profile: str = 'lspb',
                 mode: str = 'exact_stop'):
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
        
        if velocity_profile == 'lspb':
            self.profile = LSPBProfile()
        elif velocity_profile == 's-curve':
            self.profile = SCurveProfile()
        else:
            raise ValueError(f"Invalid velocity profile: {velocity_profile}")
        self.profile.generate([self.linear_dist, self.angular_dist], vel)
        
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
        r_curr = R.from_rotvec(s[1] * self.axis).as_matrix() @ self.R_start
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
    def __init__(self, velocity_profile: str = 'lspb'):
        self.via_points = [] # List of (Pose, v, mode)
        self.velocity_profile = velocity_profile
        
    def add_move(self, position: np.ndarray, euler_rpy: List[float], 
                 vel: List[float], mode: str = 'exact_stop'):
        """
        Add a target waypoint.
        mode: 'exact_stop' (G61) or 'continuous' (G64)
        """
        R_mat = R.from_euler('xyz', euler_rpy).as_matrix()
        T = np.eye(4)
        T[:3, :3] = R_mat
        T[:3, 3] = position
        self.via_points.append({'pose': T, 'vel': vel, 'mode': mode})

    def plan(self, start_pose: np.ndarray) -> List[CartesianSegment]:
        """
        Generate motion segments from start_pose through all via_points.
        """
        segments = []
        current_pose = start_pose.copy()
        
        for wp in self.via_points:
            target_pose = wp['pose']
            vel = wp['vel']
            mode = wp['mode']
            
            segment = CartesianSegment(current_pose, target_pose, vel, self.velocity_profile, mode)
            segments.append(segment)
            
            current_pose = target_pose
            
        return segments

class CNCController:
    """
    Executes the generated segments on the robot using Differential IK.
    Simulates the real-time control loop.
    """
    def __init__(self, robot: RobotKinematics, dt: float = 0.01, precision: dict = {'position': 0.001, 'orientation': 0.01}):
        self.robot = robot
        self.dt = dt
        self.precision = precision
        
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
                Te = self.robot.forward_kinematics(current_q)
                
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
                    if p_err < self.precision['position'] and angle_err < self.precision['orientation']:
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
                vel=[0.1, 1.0], mode='continuous'
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
                vel=[0.1, 1.0], mode='exact_stop'
            )
