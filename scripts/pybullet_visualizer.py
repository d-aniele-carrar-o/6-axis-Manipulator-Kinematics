import pybullet as p
import pybullet_data
import time
import numpy as np
import os
from typing import List, Optional

class PyBulletVisualizer:
    def __init__(self, urdf_path: str, use_gui: bool = True):
        """
        Initialize the PyBullet visualizer.
        
        Args:
            urdf_path: Path to the robot URDF file.
            use_gui: Whether to launch PyBullet in GUI mode.
        """
        self.urdf_path = urdf_path
        
        # Try connecting with standard GUI, if that fails or crashes, one might need options="--opengl2"
        # We'll use --opengl2 by default as it's more stable on some Linux setups
        # If there's no X server/display, GUI will crash; fall back to DIRECT.
        if use_gui and not os.environ.get("DISPLAY"):
            print("PyBullet GUI requested but $DISPLAY is not set; falling back to DIRECT.")
            use_gui = False

        connection_mode = p.GUI if use_gui else p.DIRECT
        try:
            # On macOS, avoid --opengl2 flag as it can cause crashes
            import platform
            if platform.system() == "Darwin":  # macOS
                self.physics_client = p.connect(connection_mode)
            else:
                # Using --opengl2 often helps with X11 compatibility/drivers on Linux
                self.physics_client = p.connect(connection_mode, options="--opengl2")
        except Exception as e:
            print(f"Failed to connect with GUI mode: {e}")
            print("Falling back to DIRECT mode (no visualization)...")
            self.physics_client = p.connect(p.DIRECT)
        self._closed = False
        
        # Calculate paths
        project_root = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
        src_path = os.path.join(project_root, "src")
        
        # Set search path to src so that "robot_descriptions/..." in URDF can be found
        if os.path.exists(src_path):
            p.setAdditionalSearchPath(src_path)
        
        # Reset simulation
        p.resetSimulation()
        p.setGravity(0, 0, -9.81)
        
        # Load plane (using absolute path from pybullet_data)
        plane_path = os.path.join(pybullet_data.getDataPath(), "plane.urdf")
        self.plane_id = p.loadURDF(plane_path)
        
        # Load robot
        # Calculate the absolute path to the URDF to ensure PyBullet finds it
        if not os.path.isabs(urdf_path):
             # Try to resolve relative to current working directory first
             if not os.path.exists(urdf_path):
                 # Try resolving relative to project root
                 potential_path = os.path.join(project_root, urdf_path)
                 if os.path.exists(potential_path):
                     urdf_path = potential_path
        
        print(f"Loading URDF from: {urdf_path}")
        start_pos = [0, 0, 0]
        start_orientation = p.getQuaternionFromEuler([0, 0, 0])
        # useFixedBase=True is important for manipulator
        self.robot_id = p.loadURDF(urdf_path, start_pos, start_orientation, useFixedBase=True)
        
        # Get joint information
        self.num_joints = p.getNumJoints(self.robot_id)
        self.joint_indices = []
        self.joint_names = []
        
        print(f"Robot loaded with {self.num_joints} joints/links.")
        for i in range(self.num_joints):
            info = p.getJointInfo(self.robot_id, i)
            joint_name = info[1].decode("utf-8")
            joint_type = info[2]
            # Filter for revolute joints (assuming these are the controlled ones)
            if joint_type == p.JOINT_REVOLUTE:
                self.joint_indices.append(i)
                self.joint_names.append(joint_name)
                # print(f"Found revolute joint: {joint_name} (Index: {i})")
        
        print(f"Controlled joints ({len(self.joint_indices)}): {self.joint_names}")

        # Set camera
        p.resetDebugVisualizerCamera(cameraDistance=1.5, cameraYaw=45, cameraPitch=-30, cameraTargetPosition=[0, 0, 0.5])

    def animate_trajectory(self, q_trajectory: List[np.ndarray], time_log: Optional[np.ndarray] = None, speedup: int = 1, loop: bool = False):
        """
        Animate the robot following the provided joint trajectory.
        
        Args:
            q_trajectory: List or array of joint configurations (N x num_joints).
            time_log: Optional time stamps for the trajectory. If None, uses a fixed time step.
            speedup: Frame skipping factor.
            loop: Whether to loop the animation.
        """
        q_traj = np.array(q_trajectory)
        n_frames = len(q_traj)
        
        if n_frames == 0:
            print("Empty trajectory provided.")
            return

        # Apply speedup
        indices = np.arange(0, n_frames, speedup)
        q_display = q_traj[indices]
        
        if time_log is not None:
            t_display = time_log[indices]
        else:
            t_display = None

        print(f"Starting animation with {len(q_display)} frames (Speedup: {speedup}x)...")
        try:
            while True:
                for k, i in enumerate(indices):
                    q = q_traj[i]
                    
                    # Update joint positions
                    for j, joint_idx in enumerate(self.joint_indices):
                        if j < len(q):
                            p.resetJointState(self.robot_id, joint_idx, q[j])
                    
                    p.stepSimulation()
                    
                    # Control timing
                    if t_display is not None and k < len(t_display) - 1:
                        dt = t_display[k+1] - t_display[k]
                        # Real-time waiting might be too slow if speedup is high?
                        # If speedup is 1, dt is actual time step.
                        # If speedup is N, dt is time between frame i and i+N.
                        # So we should wait dt.
                        time.sleep(dt)
                    else:
                        time.sleep(0.01)  # Default small delay
                
                if not loop:
                    break
                time.sleep(1.0) # Pause before looping
                
        except KeyboardInterrupt:
            print("Animation interrupted by user.")
        
    def close(self):
        """
        Disconnect the PyBullet physics client created by this visualizer.
        Safe to call multiple times.
        """
        if getattr(self, "_closed", False):
            return
        try:
            # Disconnect the specific client id we opened (safer than disconnecting "current").
            if self.physics_client is not None and p.isConnected(self.physics_client):
                p.disconnect(self.physics_client)
        finally:
            self._closed = True

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc, tb):
        self.close()
        return False

if __name__ == "__main__":
    # Test stub
    urdf_file = "src/robot_urdf/printed_man.urdf"
    # Basic test if run directly (assumes being run from project root)
    if os.path.exists(urdf_file) or os.path.exists(os.path.join("..", urdf_file)):
        viz = PyBulletVisualizer(urdf_file)
        # Create a dummy trajectory
        q_start = np.zeros(6)
        q_end = np.array([0, np.pi/4, -np.pi/4, 0, np.pi/4, 0])
        traj = np.linspace(q_start, q_end, 100)
        viz.animate_trajectory(traj, loop=True)
        viz.close()
    else:
        print(f"URDF file not found at {urdf_file}. Run from project root.")
