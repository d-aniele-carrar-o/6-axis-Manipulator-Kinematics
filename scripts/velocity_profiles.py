import numpy as np
from abc import ABC, abstractmethod
from typing import Tuple

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
