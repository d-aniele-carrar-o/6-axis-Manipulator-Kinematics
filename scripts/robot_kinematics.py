import numpy as np
from scipy.spatial.transform import Rotation as R
from typing import Dict, Tuple, List, Optional


class RobotKinematics:
    """Handles kinematics for 6-DOF robots using Modified DH parameters."""
    
    def __init__(self, robot_type: str = "3Dprinted", home_config: Optional[np.ndarray] = None):
        """
        Initialize robot with DH parameters.
        
        Args:
            robot_type: Name of the robot ("3Dprinted" or "UR3e")
            home_config: Optional home joint configuration. If None, uses default for robot type.
        """
        self.robot_type = robot_type
        self.dh_params = self._get_dh_parameters(robot_type)
        self.joint_limits = np.array([-2*np.pi, 2*np.pi])
        self.home_config = home_config if home_config is not None else self._get_home_config(robot_type)
        
    def _get_dh_parameters(self, robot_type: str) -> Dict[str, np.ndarray]:
        """
        Get DH parameters for different robot types.
        
        Args:
            robot_type: Name of the robot
            
        Returns:
            Dictionary containing DH parameters (AL, A, D, TH)
        """
        if robot_type == "3Dprinted":
            return {
                'AL': np.array([0, np.pi/2, 0, np.pi/2, -np.pi/2, np.pi/2, 0]),
                'A':  np.array([0, 0.03247, 0.14042, 0, 0, 0, 0]),
                'D':  np.array([0, 0.0922, 0, 0, 0.15457, 0, 0.037]),
                'TH': np.array([0, 0, 0, 0, 0, 0, 0])
            }
        elif robot_type == "UR3e":
            return {
                'AL': np.array([0, np.pi/2, 0, 0, np.pi/2, -np.pi/2, 0]),
                'A':  np.array([0, 0, -0.24355, -0.2132, 0, 0, 0]),
                'D':  np.array([0, 0.15185, 0, 0, 0.13105, 0.08535, 0.0921]),
                'TH': np.array([0, 0, 0, 0, 0, 0, 0])
            }
        else:
            raise ValueError(f"Robot type {robot_type} not supported")
    
    def _get_home_config(self, robot_type: str) -> np.ndarray:
        """
        Get default home joint configuration for robot type.
        
        Args:
            robot_type: Name of the robot
            
        Returns:
            Home joint angles (6,)
        """
        if robot_type == "3Dprinted":
            return np.array([0.0, np.pi/2, 0.0, 0.0, np.pi/4, 0.0])
        elif robot_type == "UR3e":
            return np.array([0.0, -np.pi/2, 0.0, -np.pi/2, 0.0, 0.0])
        else:
            return np.zeros(6)
    
    def get_home_pose(self) -> np.ndarray:
        """
        Get the end-effector pose at home configuration.
        
        Returns:
            4x4 transformation matrix of home pose
        """
        T = self.forward_kinematics(self.home_config)
        return T
    
    def get_end_effector_pose(self, q: np.ndarray = None) -> np.ndarray:
        """
        Get the end-effector pose for a given joint configuration.
        
        Args:
            q: Joint angles (6,)
            
        Returns:
            4x4 transformation matrix of end-effector pose
        """
        T = self.forward_kinematics(self.home_config if q is None else q)
        return T
    
    def get_end_effector_position(self, q: np.ndarray = None) -> np.ndarray:
        """
        Get the end-effector position for a given joint configuration.
        
        Args:
            q: Joint angles (6,)
            
        Returns:
            3x1 vector of end-effector position
        """
        T = self.get_end_effector_pose(self.home_config if q is None else q)
        return T[:3, 3]
    
    def get_end_effector_orientation(self, q: np.ndarray = None, mode: str = 'euler') -> np.ndarray:
        """
        Get the end-effector orientation for a given joint configuration.
        
        Args:
            q: Joint angles (6,)
            mode: ['euler' (default), 'matrix', 'quaternion']
        Returns:
            Orientation in the specified mode
            - 'euler': 3x1 vector of Euler angles (roll, pitch, yaw)
            - 'matrix': 3x3 orientation matrix
            - 'quaternion': 4x1 vector of quaternion (w, x, y, z)
        """
        T = self.get_end_effector_pose(self.home_config if q is None else q)
        if mode == 'matrix':
            return T[:3,:3]
        elif mode == 'euler':
            return R.from_matrix(T[:3,:3]).as_euler('xyz')
        elif mode == 'quaternion':
            return R.from_matrix(T[:3,:3]).as_quat()
        else:
            raise ValueError(f"Invalid mode: {mode}")
    
    @staticmethod
    def rot_trans_x(angle: float, offset: float) -> np.ndarray:
        """Rotation and translation about X axis."""
        c, s = np.cos(angle), np.sin(angle)
        return np.array([
            [1, 0, 0, offset],
            [0, c, -s, 0],
            [0, s, c, 0],
            [0, 0, 0, 1]
        ])
    
    @staticmethod
    def rot_trans_z(angle: float, offset: float) -> np.ndarray:
        """Rotation and translation about Z axis."""
        c, s = np.cos(angle), np.sin(angle)
        return np.array([
            [c, -s, 0, 0],
            [s, c, 0, 0],
            [0, 0, 1, offset],
            [0, 0, 0, 1]
        ])
    
    def transform_i_to_i1(self, i: int, qi: float) -> np.ndarray:
        """
        Compute transformation matrix from frame i-1 to frame i.
        
        Uses Modified DH convention:
        T(i-1, i) = Rot_x(alpha_{i-1}) * Trans_x(a_{i-1}) * Rot_z(theta_i) * Trans_z(d_i)
        """
        AL, A, D, TH = self.dh_params['AL'], self.dh_params['A'], self.dh_params['D'], self.dh_params['TH']
        return self.rot_trans_x(AL[i], A[i]) @ self.rot_trans_z(qi + TH[i+1], D[i+1])
    
    def forward_kinematics(self, q: np.ndarray = None) -> np.ndarray:
        """
        Compute forward kinematics.
        
        Args:
            q: Joint angles (6,)
            
        Returns:
            Tuple of (End-effector transformation matrix, List of intermediate transforms)
        """
        q = self.home_config if q is None else q
        T = np.eye(4)
        
        for i in range(6):
            T = T @ self.transform_i_to_i1(i, q[i])
        
        return T
    
    def forward_kinematics_full(self, q: np.ndarray = None) -> np.ndarray:
        """
        Compute forward kinematics and return all intermediate transforms.
        
        Args:
            q: Joint angles (6,)
            
        Returns:
            List of intermediate transforms
        """
        q = self.home_config if q is None else q
        T = np.eye(4)
        transforms = [T]
        for i in range(6):
            T = T @ self.transform_i_to_i1(i, q[i])
            transforms.append(T.copy())
        return transforms
    
    def jacobian(self, q: np.ndarray) -> np.ndarray:
        """
        Compute geometric Jacobian matching MATLAB implementation.
        
        Args:
            q: Joint angles (6,)
            
        Returns:
            Jacobian matrix (6x6)
        """
        T = np.eye(4)
        T_matrices = []
        
        # Calculate T01 through T06
        for i in range(6):
            T = T @ self.transform_i_to_i1(i, q[i])
            T_matrices.append(T)
            
        T06 = T_matrices[5]
        P06 = T06[:3, 3] # End effector position
        
        T01 = T_matrices[0]
        T02 = T_matrices[1]
        T03 = T_matrices[2]
        T04 = T_matrices[3]
        T05 = T_matrices[4]
        T06 = T_matrices[5]
        
        zetas = np.column_stack([T01[:3, 2], T02[:3, 2], T03[:3, 2], T04[:3, 2], T05[:3, 2], T06[:3, 2]])
        dists = np.column_stack([
            P06 - T01[:3, 3],
            P06 - T02[:3, 3],
            P06 - T03[:3, 3],
            P06 - T04[:3, 3],
            P06 - T05[:3, 3],
            P06 - T06[:3, 3]
        ])
        
        J_v = np.cross(zetas, dists, axis=0)
        J_o = zetas
        
        return np.vstack([J_v, J_o])
    
    def numerical_inverse_kinematics(self, q_curr: np.ndarray, Td: np.ndarray, 
                                     max_iter: int = 50, tol: float = 1e-4
                                     ) -> Tuple[np.ndarray, bool]:
        """
        Compute Inverse Kinematics using Damped Levenberg-Marquardt.
        
        Args:
            q_curr: Initial joint guess (6,)
            Td: Desired transformation matrix (4x4)
            max_iter: Maximum iterations
            tol: Tolerance for error
            
        Returns:
            Tuple of (Resulting joint angles, Success boolean)
        """
        q = q_curr.copy()
        success = False
        
        for _ in range(max_iter):
            Te = self.forward_kinematics(q)
            J = self.jacobian(q)
            
            # Position error
            pos_err = Td[:3, 3] - Te[:3, 3]
            
            # Orientation error
            Re = Te[:3, :3]
            Rd = Td[:3, :3]
            Red = Re.T @ Rd
            
            # Convert rotation error matrix to axis-angle
            trace = np.trace(Red)
            dtheta = np.arccos(np.clip((trace - 1) / 2, -1.0, 1.0))
            
            if dtheta < 1e-6:
                r_hat = np.zeros(3)
            else:
                r_hat = (1/(2*np.sin(dtheta))) * np.array([
                    Red[2,1] - Red[1,2],
                    Red[0,2] - Red[2,0],
                    Red[1,0] - Red[0,1]
                ])
            
            # Transform rotation error to base frame
            o_err = Re @ (dtheta * r_hat)
            
            e = np.concatenate([pos_err, o_err])
            error_norm = np.linalg.norm(e)
            
            if error_norm < tol:
                success = True
                break
            
            # Levenberg-Marquardt Damping
            lambda_lm = 0.1 if error_norm > 0.1 else 0.01
            
            E_val = 0.5 * error_norm
            Wn = lambda_lm * E_val * np.eye(6)
            
            # Using linalg.solve is more stable than inv()
            delta_q = np.linalg.solve(J.T @ J + Wn, J.T @ e)
            q = q + delta_q
            
        return q, success

    def differential_inverse_kinematics_step(self, q_curr: np.ndarray, 
                                             Td: np.ndarray, 
                                             vd: np.ndarray, 
                                             dt: float,
                                             max_vel: float = 6.0
                                             ) -> Tuple[np.ndarray, np.ndarray]:
        """
        Compute one step of Differential Inverse Kinematics.
        
        Args:
            q_curr: Current joint configuration
            Td: Desired end-effector pose (4x4)
            vd: Desired end-effector velocity (6,) [vx, vy, vz, wx, wy, wz]
            dt: Time step
            max_vel: Maximum joint velocity limit
            
        Returns:
            Tuple of (next joint configuration, joint velocities)
        """
        # 1. Compute current pose and Jacobian
        Te = self.forward_kinematics(q_curr)
        J = self.jacobian(q_curr)
        
        # 2. Compute error twist
        # Position error
        pos_err = Td[:3, 3] - Te[:3, 3]
        
        # Orientation error
        Re = Te[:3, :3]
        Rd = Td[:3, :3]
        
        # Rotation error vector in base frame
        # R_err = Rd * Re^T
        R_err = Rd @ Re.T
        r_rot = R.from_matrix(R_err)
        rot_vec = r_rot.as_rotvec()
        
        # Combine errors
        e = np.concatenate([pos_err, rot_vec])
        
        # 3. Control gains (Kp = Ko = 3*max_vel usually)
        gain = 3.0 * max_vel
        
        # 4. Compute effective velocity: v_eff = v_d + K * e
        v_eff = vd + gain * e
        
        # 5. Solve for q_dot using Damped Least Squares
        # q_dot = (J^T * J + lambda^2 * I)^-1 * J^T * v_eff
        lambda_dls = 0.005
        
        # To avoid issues when J is singular
        J_T = J.T
        A = J_T @ J + (lambda_dls**2) * np.eye(6)
        b = J_T @ v_eff
        
        try:
            q_dot = np.linalg.solve(A, b)
        except np.linalg.LinAlgError:
            # Fallback to pinv
            q_dot = np.linalg.pinv(J) @ v_eff
        
        # 6. Velocity scaling
        max_q_dot = np.max(np.abs(q_dot))
        if max_q_dot > max_vel:
            q_dot = q_dot * (max_vel / max_q_dot)
        
        # 7. Integration
        q_next = q_curr + q_dot * dt
        
        return q_next, q_dot
