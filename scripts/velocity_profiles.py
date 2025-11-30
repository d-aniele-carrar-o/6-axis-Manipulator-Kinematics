import numpy as np
from abc import ABC, abstractmethod
from typing import Tuple

# --- Modular Velocity Profile System ---
class VelocityProfile(ABC):
    """Abstract base class for motion profiles."""
    @abstractmethod
    def generate(self, distance: float, v_max: float, a_max: float, dt: float) -> Tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
        """Returns time, position, velocity, acceleration arrays."""
        pass

class LSPBProfile(VelocityProfile):
    """
    Linear Segment with Parabolic Blend (Trapezoidal Velocity Profile).
    Standard in industrial CNCs.
    """
    def generate(self, distance: float, v_max: float, a_max: float, dt: float):
        # Handle very small distances
        if distance < 1e-6:
            return np.array([0.0]), np.array([0.0]), np.array([0.0]), np.array([0.0])
        
        # 1. Check if triangular profile is needed (short distance)
        # Time to reach v_max
        t_acc = v_max / a_max
        dist_acc = 0.5 * a_max * t_acc**2
        
        if 2 * dist_acc > distance:
            # Triangle Profile (Cannot reach v_max)
            t_acc = np.sqrt(distance / a_max)
            v_peak = a_max * t_acc
            total_time = 2 * t_acc
            # Recalculate for triangular profile
            v_cruise = v_peak
            t_cruise = 0
            dist_cruise = 0
            dist_acc = 0.5 * a_max * t_acc**2  # Recalculate with new t_acc
        else:
            # Trapezoidal Profile
            dist_cruise = distance - 2 * dist_acc
            t_cruise = dist_cruise / v_max
            total_time = 2 * t_acc + t_cruise
            v_cruise = v_max

        # Generate points (ensure exact final time is included)
        n_points = int(np.round(total_time / dt)) + 1
        times = np.linspace(0, total_time, n_points)
        s = []    # Position along path
        s_dot = [] # Velocity along path
        s_ddot = [] # Acceleration along path
        
        for t in times:
            if t <= t_acc:
                # Acceleration phase
                s.append(0.5 * a_max * t**2)
                s_dot.append(a_max * t)
                s_ddot.append(a_max)
            elif t < t_acc + t_cruise:
                # Cruise phase
                s.append(dist_acc + v_cruise * (t - t_acc))
                s_dot.append(v_cruise)
                s_ddot.append(0.0)
            else:
                # Deceleration phase
                t_dec = t - (t_acc + t_cruise)
                rem_dist = v_cruise * t_dec - 0.5 * a_max * t_dec**2
                s.append(dist_acc + dist_cruise + rem_dist)
                s_dot.append(v_cruise - a_max * t_dec)
                s_ddot.append(-a_max)
                
        return times, np.array(s), np.array(s_dot), np.array(s_ddot)


if __name__ == "__main__":
    import matplotlib.pyplot as plt
    
    # Test scenarios
    scenarios = [
        {"name": "Trapezoidal (Long Distance)", "distance": 10.0, "v_max": 2.0, "a_max": 1.0, "dt": 0.1},
        {"name": "Triangular (Short Distance)", "distance": 1.0, "v_max": 2.0, "a_max": 1.0, "dt": 0.05},
        {"name": "Very Short Distance", "distance": 0.5, "v_max": 3.0, "a_max": 2.0, "dt": 0.02},
        {"name": "High Acceleration", "distance": 5.0, "v_max": 1.5, "a_max": 5.0, "dt": 0.05}
    ]
    
    profile = LSPBProfile()
    
    # Create separate plots for each scenario
    for i, scenario in enumerate(scenarios):
        params = {k: v for k, v in scenario.items() if k != 'name'}
        times, positions, velocities, accelerations = profile.generate(**params)
        
        # Debug info
        distance = scenario['distance']
        v_max = scenario['v_max']
        a_max = scenario['a_max']
        
        t_acc = v_max / a_max
        dist_acc = 0.5 * a_max * t_acc**2
        is_triangular = 2 * dist_acc > distance
        
        print(f"\n=== {scenario['name']} ===")
        print(f"Distance: {distance:.2f}, v_max: {v_max:.2f}, a_max: {a_max:.2f}")
        print(f"t_acc: {t_acc:.3f}s, dist_acc: {dist_acc:.3f}")
        print(f"Profile type: {'Triangular' if is_triangular else 'Trapezoidal'}")
        print(f"Total time: {times[-1]:.3f}s, Final position: {positions[-1]:.3f}")
        print(f"Max velocity reached: {np.max(velocities):.3f}")
        print(f"Max acceleration: {np.max(np.abs(accelerations)):.3f}")
        
        # Create individual plot for this scenario
        fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 8))
        fig.suptitle(f'LSPB Profile: {scenario["name"]}', fontsize=14)
        
        # Plot position
        ax1.plot(times, positions, 'b-', linewidth=2, label='Position')
        ax1.set_title('Position vs Time')
        ax1.set_xlabel('Time (s)')
        ax1.set_ylabel('Position')
        ax1.grid(True, alpha=0.3)
        ax1.legend()
        
        # Plot velocity and acceleration
        ax2.plot(times, velocities, 'r-', linewidth=2, label='Velocity')
        ax2.plot(times, accelerations, 'g--', linewidth=1.5, label='Acceleration')
        ax2.axhline(y=v_max, color='r', linestyle=':', alpha=0.7, label=f'v_max={v_max}')
        ax2.axhline(y=a_max, color='g', linestyle=':', alpha=0.7, label=f'a_max={a_max}')
        ax2.axhline(y=-a_max, color='g', linestyle=':', alpha=0.7)
        ax2.set_title('Velocity & Acceleration vs Time')
        ax2.set_xlabel('Time (s)')
        ax2.set_ylabel('Velocity / Acceleration')
        ax2.grid(True, alpha=0.3)
        ax2.legend()
        
        plt.tight_layout()
        plt.show()
    
    # Phase analysis plot
    fig, (ax1, ax2, ax3) = plt.subplots(3, 1, figsize=(12, 10))
    fig.suptitle('Detailed Phase Analysis - Trapezoidal Profile', fontsize=14)
    
    # Use trapezoidal scenario for detailed analysis
    scenario = scenarios[0]
    params = {k: v for k, v in scenario.items() if k != 'name'}
    times, positions, velocities, accelerations = profile.generate(**params)
    
    # Calculate phase boundaries
    v_max = scenario['v_max']
    a_max = scenario['a_max']
    t_acc = v_max / a_max
    dist_acc = 0.5 * a_max * t_acc**2
    dist_cruise = scenario['distance'] - 2 * dist_acc
    t_cruise = dist_cruise / v_max
    
    # Mark phases
    phase_colors = ['lightblue', 'lightgreen', 'lightcoral']
    phase_labels = ['Acceleration', 'Cruise', 'Deceleration']
    phase_times = [t_acc, t_acc + t_cruise, times[-1]]
    
    for i, (ax, data, ylabel) in enumerate([(ax1, positions, 'Position'), 
                                           (ax2, velocities, 'Velocity'),
                                           (ax3, accelerations, 'Acceleration')]):
        ax.plot(times, data, 'k-', linewidth=2)
        
        # Color phases
        prev_t = 0
        for j, (t_end, color, label) in enumerate(zip(phase_times, phase_colors, phase_labels)):
            mask = (times >= prev_t) & (times <= t_end)
            if np.any(mask):
                ax.fill_between(times[mask], 0, data[mask], alpha=0.3, color=color, label=label)
            prev_t = t_end
        
        ax.set_ylabel(ylabel)
        ax.grid(True, alpha=0.3)
        ax.legend()
        
        # Add phase boundary lines
        for t_boundary in [t_acc, t_acc + t_cruise]:
            if t_boundary < times[-1]:
                ax.axvline(x=t_boundary, color='red', linestyle='--', alpha=0.7)
    
    ax3.set_xlabel('Time (s)')
    plt.tight_layout()
    plt.show()
