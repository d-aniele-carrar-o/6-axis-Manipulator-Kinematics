import numpy as np
from abc import ABC, abstractmethod
from typing import List, Tuple

# --- Modular Velocity Profile System ---
class VelocityProfile(ABC):
    """Abstract base class for motion profiles."""
    @abstractmethod
    def generate(self, distance: List[float], vel: List[float], a_max: float = 1.) -> Tuple[np.ndarray, np.ndarray]:
        """Returns position and velocity arrays for a synchronized LSPB."""
        pass

class LSPBProfile(VelocityProfile):
    """
    Linear Segment with Parabolic Blend (Trapezoidal Velocity Profile).
    Improved with Multi-Axis Synchronization and Vector Projection.
    """
    def generate(self, distance: List[float], vel: List[float], a_max: float = 1.):
        """
        Calculates synchronized LSPB parameters.
        Args:
            distance: Target distance for each axis.
            vel: Cruise velocity limit for each axis.
            a_max: Max acceleration (scalar or list per axis).
        """
        # 1. Convert inputs to numpy arrays
        self.dist_axes = np.array(distance)
        self.v_limits = np.array(vel)
        
        # Handle scalar vs list inputs for constraints
        if np.ndim(a_max) == 0: a_limits = np.full_like(self.dist_axes, a_max, dtype=float)
        else: a_limits = np.array(a_max, dtype=float)
            
        # 2. Project Constraints to 1D Path
        self.dist_path = np.linalg.norm(self.dist_axes)
        
        if self.dist_path < 1e-9:
            self.u_vec = np.zeros_like(self.dist_axes)
            self.T = 0.0
            return

        # Unit Direction Vector
        self.u_vec = self.dist_axes / self.dist_path
        
        # Active axes mask
        active_mask = np.abs(self.u_vec) > 1e-6
        
        # Calculate Effective Path Limits (Weakest Link Logic)
        self.path_v_max = np.min(self.v_limits[active_mask] / np.abs(self.u_vec[active_mask]))
        self.path_a_max = np.min(a_limits[active_mask] / np.abs(self.u_vec[active_mask]))

        # 3. Robust 1D Solver
        # Determine Blend Time (time to accelerate) assuming we reach v_max
        tb_ideal = self.path_v_max / self.path_a_max
        
        # Check if we are Distance Limited (Triangular Profile)
        # Distance required to reach full speed and stop: D = v^2 / a
        d_full_speed = (self.path_v_max**2) / self.path_a_max
        
        if self.dist_path < d_full_speed:
            # -- Triangular Profile (Short Move) --
            # We never reach path_v_max. We peak at a lower velocity.
            # dist = a * tb^2  =>  tb = sqrt(dist / a)
            self.tb = np.sqrt(self.dist_path / self.path_a_max)
            self.T = 2 * self.tb
            
            # Recalculate actual peak parameters for this specific move
            self.v_cruise_actual = self.path_a_max * self.tb
            self.a_actual = self.path_a_max # We still use max accel
        else:
            # -- Trapezoidal Profile (Long Move) --
            self.tb = tb_ideal
            self.v_cruise_actual = self.path_v_max
            self.a_actual = self.path_a_max
            
            # Distance covered during blends (accel + decel): D_blend = v*tb
            # Remaining distance for cruise: D_cruise = dist - v*tb
            # T_cruise = D_cruise / v
            # T_total = 2*tb + T_cruise = 2*tb + (dist/v - tb) = dist/v + tb
            self.T = (self.dist_path / self.v_cruise_actual) + self.tb

    def s(self, t: float):
        # Piecewise evaluation for scalar path s(t) and v(t)
        if t <= self.tb:
            # Acceleration phase
            s_scalar = 0.5 * self.a_actual * t**2
            v_scalar = self.a_actual * t
        elif t < self.T - self.tb:
            # Cruise phase
            # s = s_accel_end + v * (t - tb)
            # s_accel_end = 0.5 * a * tb^2 = 0.5 * v * tb
            s_accel_end = 0.5 * self.v_cruise_actual * self.tb
            s_scalar = s_accel_end + self.v_cruise_actual * (t - self.tb)
            v_scalar = self.v_cruise_actual
        elif t <= self.T:
            # Deceleration phase
            # easier to calculate from the end backwards
            t_rem = self.T - t
            s_scalar = self.dist_path - 0.5 * self.a_actual * t_rem**2
            v_scalar = self.a_actual * t_rem
        else:
            s_scalar = self.dist_path
            v_scalar = 0.0

        # Scale by direction vector for multi-axis output
        return (s_scalar * self.u_vec, v_scalar * self.u_vec)

class SCurveProfile(VelocityProfile):
    """
    S-Curve (Double S) Velocity Profile with Multi-Axis Synchronization.
    Automatically handles ideal, triangular, and short-move cases.
    """
    def generate(self, distance: List[float], vel: List[float], a_max: float = 7.5, j_max: float = 10.):
        """
        Calculates synchronized S-Curve parameters.
        Args:
            distance: Target distance for each axis.
            vel: Cruise velocity limit for each axis.
            a_max: Max acceleration (scalar or list per axis).
            j_max: Max jerk (scalar or list per axis).
        """
        # 1. Convert inputs to numpy arrays
        self.dist_axes = np.array(distance)
        self.v_limits = np.array(vel)
        
        # Handle scalar vs list inputs for constraints
        if np.ndim(a_max) == 0: a_limits = np.full_like(self.dist_axes, a_max, dtype=float)
        else: a_limits = np.array(a_max, dtype=float)
            
        if np.ndim(j_max) == 0: j_limits = np.full_like(self.dist_axes, j_max, dtype=float)
        else: j_limits = np.array(j_max, dtype=float)

        # 2. Project Constraints to 1D Path
        self.dist_path = np.linalg.norm(self.dist_axes)
        
        if self.dist_path < 1e-9:
            self.u_vec = np.zeros_like(self.dist_axes)
            self.T = 0.0
            return # Zero movement

        # Unit Direction Vector
        self.u_vec = self.dist_axes / self.dist_path
        
        # Active axes mask (avoid division by zero)
        active_mask = np.abs(self.u_vec) > 1e-6
        
        # Calculate Effective Path Limits (Weakest Link Logic)
        self.path_v_max = np.min(self.v_limits[active_mask] / np.abs(self.u_vec[active_mask]))
        self.path_a_max = np.min(a_limits[active_mask] / np.abs(self.u_vec[active_mask]))
        self.path_j_max = np.min(j_limits[active_mask] / np.abs(self.u_vec[active_mask]))

        # 3. Robust 1D Solver
        # A. Theoretical Limits
        v_accel_limit = self.path_a_max**2 / self.path_j_max

        # B. Ideal Times (Infinite Distance)
        if self.path_v_max <= v_accel_limit:
            Ta_ideal = np.sqrt(self.path_v_max / self.path_j_max)
            Tc_ideal = 0.0
        else:
            Ta_ideal = self.path_a_max / self.path_j_max
            Tc_ideal = (self.path_v_max - self.path_a_max * Ta_ideal) / self.path_a_max

        # C. Check Distance Requirement
        d_req_for_cruise = self.path_v_max * (2 * Ta_ideal + Tc_ideal)

        if self.dist_path >= d_req_for_cruise:
            # Scenario: Long Move (Has Cruise)
            self.Ta = Ta_ideal
            self.Tc = Tc_ideal
            d_cruise = self.dist_path - d_req_for_cruise
            self.Tcruise = d_cruise / self.path_v_max
        else:
            # Scenario: Short Move (No Cruise)
            self.Tcruise = 0.0
            d_half = self.dist_path / 2.0
            # Distance to reach peak accel (triangular profile)
            d_amax_triangular = self.path_j_max * (self.path_a_max / self.path_j_max)**3
            
            if d_half < d_amax_triangular:
                # Sub-Case: Double Triangle (Jerk Limited)
                self.Ta = (d_half / self.path_j_max)**(1/3)
                self.Tc = 0.0
            else:
                # Sub-Case: Trapezoidal Accel (Accel Limited)
                self.Ta = self.path_a_max / self.path_j_max
                # Solve quadratic: Tc^2 + 3*Ta*Tc + (2*Ta^2 - 2*d_half/a_max) = 0
                roots_tc = np.roots([1, 3*self.Ta, (2*self.Ta**2 - 2*d_half/self.path_a_max)])
                self.Tc = np.max(roots_tc)

        # 4. Pre-calculate Boundary States
        self.j_use = self.path_j_max
        self.a_use = self.path_j_max * self.Ta  # Actual peak accel (critical for triangle case)
        self.v0 = 0.0
        
        # Switching times
        self.t1 = self.Ta
        self.t2 = self.Ta + self.Tc
        self.t3 = 2*self.Ta + self.Tc
        self.t4 = 2*self.Ta + self.Tc + self.Tcruise
        self.t5 = 3*self.Ta + self.Tc + self.Tcruise
        self.t6 = 3*self.Ta + 2*self.Tc + self.Tcruise
        self.T  = 4*self.Ta + 2*self.Tc + self.Tcruise
        
        # Boundary values (Path Scalar)
        self.v1 = self.v0 + 0.5 * self.j_use * self.Ta**2
        self.s1 = self.v0 * self.Ta + (1/6) * self.j_use * self.Ta**3
        
        self.v2 = self.v1 + self.a_use * self.Tc
        self.s2 = self.s1 + self.v1 * self.Tc + 0.5 * self.a_use * self.Tc**2
        
        self.v_mid = self.v2 + 0.5 * self.j_use * self.Ta**2
        self.s3 = self.s2 + self.v2 * self.Ta + 0.5 * self.a_use * self.Ta**2 - (1/6) * self.j_use * self.Ta**3
        
        self.s4 = self.s3 + self.v_mid * self.Tcruise
        
        self.v5 = self.v_mid - 0.5 * self.j_use * self.Ta**2
        self.s5 = self.s4 + self.v_mid * self.Ta - (1/6) * self.j_use * self.Ta**3
        
        self.v6 = self.v5 - self.a_use * self.Tc
        self.s6 = self.s5 + self.v5 * self.Tc - 0.5 * self.a_use * self.Tc**2

    def s(self, t: float):
        # Piecewise evaluation for scalar path s(t) and v(t) using relative time
        if t < self.t1:
            s_scalar = self.v0*t + (1/6)*self.j_use*t**3
            v_scalar = self.v0 + 0.5 * self.j_use * t**2
        elif t < self.t2:
            dt = t - self.t1
            s_scalar = self.s1 + self.v1*dt + 0.5*self.a_use*dt**2
            v_scalar = self.v1 + self.a_use*dt
        elif t < self.t3:
            dt = t - self.t2
            s_scalar = self.s2 + self.v2*dt + 0.5*self.a_use*dt**2 - (1/6)*self.j_use*dt**3
            v_scalar = self.v2 + self.a_use*dt - 0.5 * self.j_use * dt**2
        elif t < self.t4:
            dt = t - self.t3
            s_scalar = self.s3 + self.v_mid*dt
            v_scalar = self.v_mid
        elif t < self.t5:
            dt = t - self.t4
            s_scalar = self.s4 + self.v_mid*dt - (1/6)*self.j_use*dt**3
            v_scalar = self.v_mid - 0.5 * self.j_use * dt**2
        elif t < self.t6:
            dt = t - self.t5
            s_scalar = self.s5 + self.v5*dt - 0.5*self.a_use*dt**2
            v_scalar = self.v5 - self.a_use*dt
        elif t <= self.T:
            dt = t - self.t6
            s_scalar = self.s6 + self.v6*dt - 0.5*self.a_use*dt**2 + (1/6)*self.j_use*dt**3
            v_scalar = self.v6 - self.a_use*dt + 0.5 * self.j_use * dt**2
        else:
            s_scalar = self.dist_path
            v_scalar = 0.0

        # Scale by direction vector to get multi-axis output
        return (s_scalar * self.u_vec, v_scalar * self.u_vec)


if __name__ == "__main__":
    import matplotlib.pyplot as plt
    
    # Test scenarios
    scenarios = [
        {"name": "Trapezoidal (Long Distance)", "distance": [2., 4.], "vel": [3.0, 5.0]},
        {"name": "Triangular (Short Distance)", "distance": [1.5, 0.1], "vel": [0.2, 1.0]},
    ]
    
    # profile = LSPBProfile()
    profile = SCurveProfile()
    
    # Create separate plots for each scenario
    for i, scenario in enumerate(scenarios):
        params = {k: v for k, v in scenario.items() if k != 'name'}
        profile.generate(**params)
        
        # Create individual plot for this scenario
        fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 8))
        fig.suptitle(f'LSPB Profile: {scenario["name"]}', fontsize=14)
        
        dt = 0.02
        times = np.linspace(0, profile.T, int(profile.T / dt))

        for t in times:
            s, s_dot = profile.s(t)
            ax1.plot(t, s[0], 'bo')
            ax1.plot(t, s[1], 'go')
            ax2.plot(t, s_dot[0], 'bo')
            ax2.plot(t, s_dot[1], 'go')
        
        plt.tight_layout()
        plt.show()
