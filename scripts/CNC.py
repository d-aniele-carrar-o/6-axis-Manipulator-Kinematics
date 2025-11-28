import time
import serial
import serial.tools.list_ports
import numpy as np
import matplotlib.pyplot as plt
from typing import List, Tuple, Dict, Optional
from scipy.spatial.transform import Slerp, Rotation as R
from abc import ABC, abstractmethod
from matplotlib.animation import FuncAnimation

from numpy import pi
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
        # Initialize logs with STARTING state (t=0)
        time_log = [0.0]
        q_log = [start_q.copy()]
        q_dot_log = [np.zeros(6)]
        
        # Initial Desired State (from start of first segment)
        if segments:
            p0 = segments[0].start_pose[:3, 3]
            r0 = R.from_matrix(segments[0].start_pose[:3, :3]).as_euler('xyz')
        else:
            p0 = np.zeros(3)
            r0 = np.zeros(3)
            
        pd_log = [p0]
        rpy_d_log = [r0]
        vd_log = [np.zeros(6)]
        v_act_log = [np.zeros(6)]
        
        segment_end_times = []
        
        current_q = start_q.copy()
        current_time = 0.0
        
        print(f"Starting CNC execution. Processing {len(segments)} blocks.")
        
        for i, segment in enumerate(segments):
            print(f"  > Block {i+1}: Dist={segment.linear_dist:.3f}m, Planned Time={segment.times[-1]:.2f}s")
            
            idx = 0
            while True:
                # 1. Get Setpoint (Interpolator)
                Td, vd, is_done_profile = segment.get_state(idx)
                
                # 2. Check Execution Status
                Te, _ = self.robot.forward_kinematics(current_q)
                
                # Calculate Error relative to the CURRENT setpoint
                p_err = np.linalg.norm(Td[:3, 3] - Te[:3, 3])
                
                # Orientation Error for exit condition
                R_err = Td[:3, :3] @ Te[:3, :3].T
                rot_vec = R.from_matrix(R_err).as_rotvec()
                angle_err = np.linalg.norm(rot_vec)
                
                # Exit Condition
                if is_done_profile:
                    segment_end_times.append(current_time)
                    if segment.mode == 'continuous':
                         print(f"    Continuous transition (G64). Pos Error: {p_err*1000:.3f}mm")
                         break
                    if p_err < 1e-4 and angle_err < np.radians(0.01):
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
                
                pd_log.append(Td[:3, 3])
                rpy_d = R.from_matrix(Td[:3, :3]).as_euler('xyz')
                rpy_d_log.append(rpy_d)
                
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

# --- Real Robot Interface ---
class RealRobotInterface:
    def __init__(self, port=None, baud=115200):
        if port is None:
            ports = list(serial.tools.list_ports.comports())
            if not ports:
                raise Exception("No serial ports found!")
            print("Available ports:")
            for i, p in enumerate(ports):
                print(f" {i}: {p.device}")
            
            if len(ports) == 1:
                port = ports[0].device
                print(f"Selecting {port}")
            else:
                try:
                    choice = int(input(f"Select port (0-{len(ports)-1}): "))
                    port = ports[choice].device
                    print(f"Selected {port}")
                except (ValueError, IndexError):
                    port = ports[0].device
                    print(f"Invalid selection, using {port}")
            
        self.ser = serial.Serial(port, baud, timeout=1)
        time.sleep(2) # Wait for Arduino reset
        print("Connected to Real Robot.")
        
        # Calibration Data (Steps per 90 degrees)
        self.steps_90_deg = np.array([99000, 146000, 139500, 170000, 138000])
        self.steps_per_rad = self.steps_90_deg / (np.pi / 2.0)
        
        # Buffer Tracking
        self.BUFFER_SIZE = 64
        self.buffer_slots = self.BUFFER_SIZE
        
    def clear_buffer(self):
        """Clear any remaining data in serial buffer"""
        self.ser.reset_input_buffer()
        while self.ser.in_waiting:
            self.ser.read()
            
    def rad_to_steps(self, q_rad: np.ndarray, start_q: np.ndarray) -> np.ndarray:
        """
        Convert joint angles to steps. 
        Assumes robot starts at 'start_q' which corresponds to 0 steps.
        Only returns first 5 axes.
        """
        # Delta from start position
        q_delta = q_rad[:5] - start_q[:5]
        steps = q_delta * self.steps_per_rad
        
        # Reverse direction for joints 3 and 5 (indices 2 and 4)
        steps[2] = -steps[2]  # Joint 3 (Z axis)
        steps[4] = -steps[4]  # Joint 5 (B axis)
        
        return steps.astype(int)

    def wait_for_ready(self):
        """Wait for Arduino to send READY string"""
        print("Waiting for Arduino...")
        
        # First, try to read any existing messages
        start_time = time.time()
        ready_received = False
        
        while time.time() - start_time < 5.0:  # 5 second timeout
            if self.ser.in_waiting:
                try:
                    line = self.ser.readline().decode().strip()
                    print(f"Arduino: {line}")
                    if "READY" in line:
                        ready_received = True
                        break
                except:
                    pass
            time.sleep(0.1)
        
        if not ready_received:
            print("No READY message received, sending test command...")
            # Send a test command to verify communication
            self.ser.write(b"G28\n")  # Home command
            time.sleep(0.5)
            
            # Check for OK response
            if self.ser.in_waiting:
                response = self.ser.readline().decode().strip()
                print(f"Test response: {response}")
                if "OK" in response:
                    print("Arduino communication verified.")
                else:
                    print("Warning: Unexpected response from Arduino")
            else:
                print("Warning: No response from Arduino")
        else:
            print("Arduino Ready.")

    def stream_trajectory(self, q_traj: np.ndarray, time_traj: np.ndarray, start_q: np.ndarray):
        """
        Stream the generated trajectory to the Arduino.
        """
        # --- SAFEGUARD ANALYSIS ---
        print("\n--- Trajectory Analysis ---")
        q_traj_5 = q_traj[:, :5]
        min_q = np.min(q_traj_5, axis=0)
        max_q = np.max(q_traj_5, axis=0)
        range_q = max_q - min_q
        
        axes_names = ['X (J1)', 'Y (J2)', 'Z (J3)', 'A (J4)', 'B (J5)']
        for i in range(5):
            steps_range = int(range_q[i] * self.steps_per_rad[i])
            print(f"Axis {axes_names[i]}: Range {np.rad2deg(range_q[i]):.2f} deg ({steps_range} steps)")
            if steps_range > 0:
                 print(f"  -> MOVING")
        
        # Check start point consistency
        start_steps = self.rad_to_steps(q_traj[0], start_q)
        print(f"Start Step Error: {start_steps} (Should be all zeros)")
        if np.any(np.abs(start_steps) > 10):
            print("WARNING: Trajectory does not start at Home configuration!")
        
        print("Streaming trajectory to robot...")
        
        # INCREASED FREQUENCY for smoother motion
        # Target ~50Hz (dt = 0.02s) to reduce stutter
        target_dt = 0.02 
        original_dt = time_traj[1] - time_traj[0] if len(time_traj) > 1 else target_dt
        step = int(target_dt / original_dt)
        if step < 1: step = 1
        
        indices = list(range(0, len(q_traj), step))
        if indices[-1] != len(q_traj) - 1:
            indices.append(len(q_traj) - 1)
            
        print(f"Downsampling: Sending {len(indices)} points (from {len(q_traj)} generated). Target dt={target_dt}s")
        
        total_points = len(indices)
        sent_count = 0
        
        for i in range(len(indices)):
            idx = indices[i]
            q_curr = q_traj[idx]
            steps = self.rad_to_steps(q_curr, start_q)
            
            # Calculate Speed (Feedrate)
            if i < len(indices) - 1:
                idx_next = indices[i+1]
                dt_segment = time_traj[idx_next] - time_traj[idx]
                
                q_next = q_traj[idx_next]
                steps_next = self.rad_to_steps(q_next, start_q)
                
                # Max steps to travel in this segment among all axes
                max_diff = np.max(np.abs(steps_next - steps))
                
                if dt_segment > 0 and max_diff > 0:
                    speed = max_diff / dt_segment
                else:
                    speed = 2000 # Default/Idle speed
            else:
                speed = 2000
                
            # Construct G-Code
            cmd = f"G1 X{steps[0]} Y{steps[1]} Z{steps[2]} A{steps[3]} B{steps[4]} F{speed:.1f}\n"
            
            # Flow Control
            self.send_command(cmd)
            sent_count += 1
            if sent_count % 50 == 0:
                print(f"Sent {sent_count}/{total_points}...")

        print("Trajectory transmission complete.")

    def send_command(self, cmd):
        # We need to ensure we don't overflow the buffer.
        while self.buffer_slots <= 0:
            self.process_incoming()
            time.sleep(0.001)
            
        self.ser.write(cmd.encode())
        self.buffer_slots -= 1
        
        # Wait for "OK" to confirm receipt
        while True:
            line = self.ser.readline().decode().strip()
            if line == "OK":
                break
            elif line == "DONE":
                self.buffer_slots += 1
                if self.buffer_slots > self.BUFFER_SIZE: self.buffer_slots = self.BUFFER_SIZE
            elif line:
                # print(f"Arduino Msg: {line}")
                pass
                
    def process_incoming(self):
        """Read any pending DONE messages to free slots"""
        while self.ser.in_waiting:
            line = self.ser.readline().decode().strip()
            if line == "DONE":
                self.buffer_slots += 1
                if self.buffer_slots > self.BUFFER_SIZE: self.buffer_slots = self.BUFFER_SIZE
            elif line == "OK":
                pass 
            elif line:
                # print(f"Arduino Async: {line}")
                pass


# --- Simple Geometric Trajectories ---
class GeometryTrajectory:
    @staticmethod
    def circle(robot: RobotKinematics, radius, n_points):
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
    def rectangle(robot: RobotKinematics, width, height):
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


# --- Main Execution ---
def main():
    robot = RobotKinematics("3Dprinted", home_config=np.array([0.0, pi/2, 0.0, 0.0, -pi/2, 0.0]))
    planner = CNCPlanner()
    controller = CNCController(robot)
    
    planner.add_move(
        position=np.array([0.187, 0, 0.03]),
        euler_rpy=[0, pi, pi],
        v_max=0.1, a_max=0.5,
        mode='exact_stop'
    )
    planner.add_move(
        position=np.array([0.187, -0.05, 0.03]),
        euler_rpy=[0, pi, pi],
        v_max=0.1, a_max=0.5,
        mode='exact_stop'
    )
    
    # 2. Plan (Generate Segments)
    print("Generating CNC Path...")
    start_T, _ = robot.forward_kinematics(robot.home_config)
    segments = planner.plan(start_T, dt=0.01)
    
    # 3. Execute (Control Loop)
    time_log, q_traj, q_dot_traj, vd_log, v_act_log, pd_log, rpy_d_log, segment_end_times = controller.execute(segments, robot.home_config)
    
    # 4. Visualization
    if len(time_log) == 0:
        print("Error: No trajectory generated.")
        return
    
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
    desired_trajectory = []
    desired_trajectory.append(start_T)
    for seg in segments:
        desired_trajectory.append(seg.end_pose)
    
    # Uncomment to show visualization
    visualizer = TrajectoryVisualizer(robot)
    visualizer.animate_trajectory([q_traj[i] for i in range(len(q_traj))], desired_trajectory)
    
    # 5. Real Robot Execution
    print("\n--- Real Robot Execution ---")
    user_input = input("Connect to real robot and execute? (y/n): ")
    if user_input.lower() == 'y':
        try:
            # Specify port if known, e.g., port='/dev/ttyUSB0'
            rr = RealRobotInterface()
            rr.wait_for_ready()
            
            # Verify Home Position
            print("Note: Ensure robot is physically at HOME position before starting.")
            
            rr.stream_trajectory(q_traj, time_log, robot.home_config)
            
        except Exception as e:
            print(f"Error executing on real robot: {e}")


if __name__ == "__main__":
    main()
