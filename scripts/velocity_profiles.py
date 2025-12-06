import numpy as np
from abc import ABC, abstractmethod
from typing import List, Tuple

# --- Modular Velocity Profile System ---
class VelocityProfile(ABC):
    """Abstract base class for motion profiles."""
    @abstractmethod
    def generate(self, distance: np.ndarray, vel: List[float], a_max: float = 2.) -> Tuple[np.ndarray, np.ndarray]:
        """Returns position and velocity arrays for a synchronized LSPB."""
        pass

class LSPBProfile(VelocityProfile):
    """
    Linear Segment with Parabolic Blend (Trapezoidal Velocity Profile).
    """
    def generate(self, distance: np.ndarray, vel: List[float], a_max: float = 2.):
        # Blend time: time to reach vel
        self.t_b = np.max(vel) / a_max
        # Total segment period
        self.distance = np.array(distance)
        self.T = np.max(self.distance / vel) + self.t_b

        # Check if triangular profile is needed (short distance)
        if 2. * 0.5 * a_max * self.t_b**2 > np.max(self.distance):
            # Triangle Profile (Cannot reach v_max)
            self.t_b = np.sqrt(np.max(self.distance) / a_max)
            self.T = 2 * self.t_b
            # Recalculate for triangular profile
            self.a_max = self.distance / (self.t_b**2)
            self.v_cruise = self.a_max * self.t_b
        else:
            # Trapezoidal Profile
            self.v_cruise = self.distance / (self.T - self.t_b)
            self.a_max = self.v_cruise / self.t_b

    def s(self, t: float):
        if t <= self.t_b:
            # Acceleration phase
            return (0.5 * self.a_max * t**2, self.a_max * t)  # s, s_dot
        elif t < self.T - self.t_b:
            # Cruise phase
            return (self.v_cruise * (t - self.t_b/2), self.v_cruise)  # s, s_dot
        elif t < self.T:
            # Deceleration phase
            return (self.distance - 0.5 * self.a_max * (self.T - t)**2, self.a_max * (self.T - t))  # s, s_dot
        else:
            return (self.distance, np.zeros_like(self.distance))  # s, s_dot


if __name__ == "__main__":
    import matplotlib.pyplot as plt
    
    # Test scenarios
    scenarios = [
        {"name": "Trapezoidal (Long Distance)", "distance": [2., 4.], "vel": [3.0, 5.0]},
        {"name": "Triangular (Short Distance)", "distance": [1.5, 0.1], "vel": [2.0, 1.0]},
    ]
    
    profile = LSPBProfile()
    
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
