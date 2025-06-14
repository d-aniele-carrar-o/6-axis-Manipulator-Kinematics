import pandas as pd
import matplotlib.pyplot as plt
import numpy as np

# Read the CSV file with proper handling of whitespace in column names
# df_empty_box = pd.read_csv('/Users/danielecarraro/Documents/VSCODE/master-thesis/YOTO/data/1749729939_motion.csv', skipinitialspace=True)
df_unbalanced_box = pd.read_csv('/Users/danielecarraro/Documents/VSCODE/master-thesis/YOTO/data/1749734485_motion.csv', skipinitialspace=True)

df = df_unbalanced_box

# Create time index
time = np.arange(len(df))

# Define column mappings based on the actual column names
# First robot columns
robot1_cols = {
    'x': 'x1', 'y': 'y1', 'z': 'z1',
    'rx': 'rx1', 'ry': 'ry1', 'rz': 'rz1',
    'xdot': 'xdot1', 'ydot': 'ydot1', 'zdot': 'zdot1',
    'rx_dot': 'rx_dot1', 'ry_dot': 'ry_dot1', 'rz_dot': 'rz_dot1',
    'fx': 'fx1', 'fy': 'fy1', 'fz': 'fz1',
    'tau_x': 'tau_x1', 'tau_y': 'tau_y1', 'tau_z': 'tau_z1'
}

# Second robot columns
robot2_cols = {
    'x': 'x2', 'y': 'y2', 'z': 'z2',
    'rx': 'rx2', 'ry': 'ry2', 'rz': 'rz2',
    'xdot': 'xdot2', 'ydot': 'ydot2', 'zdot': 'zdot2',
    'rx_dot': 'rx_dot2', 'ry_dot': 'ry_dot2', 'rz_dot': 'rz_dot2',
    'fx': 'fx2', 'fy': 'fy2', 'fz': 'fz2',
    'tau_x': 'tau_x2', 'tau_y': 'tau_y2', 'tau_z': 'tau_z2'
}

# Third robot columns
robot3_cols = {
    'x': 'x3', 'y': 'y3', 'z': 'z3',
    'rx': 'rx3', 'ry': 'ry3', 'rz': 'rz3',
    'xdot': 'xdot3', 'ydot': 'ydot3', 'zdot': 'zdot3',
    'rx_dot': 'rx_dot3', 'ry_dot': 'ry_dot3', 'rz_dot': 'rz_dot3',
    'fx': 'fx3', 'fy': 'fy3', 'fz': 'fz3',
    'tau_x': 'tau_x3', 'tau_y': 'tau_y3', 'tau_z': 'tau_z3'
}

# Fourth robot columns
robot4_cols = {
    'x': 'x4', 'y': 'y4', 'z': 'z4',
    'rx': 'rx4', 'ry': 'ry4', 'rz': 'rz4',
    'xdot': 'xdot4', 'ydot': 'ydot4', 'zdot': 'zdot4',
    'rx_dot': 'rx_dot4', 'ry_dot': 'ry_dot4', 'rz_dot': 'rz_dot4',
    'fx': 'fx4', 'fy': 'fy4', 'fz': 'fz4',
    'tau_x': 'tau_x4', 'tau_y': 'tau_y4', 'tau_z': 'tau_z4'
}

# Plot first two robots
fig1, axs1 = plt.subplots(3, 2, figsize=(14, 10))
fig1.suptitle('Robot Motion Data - Robots 1 & 2', fontsize=16)

# Position plots
axs1[0, 0].plot(time, df[robot1_cols['x']], 'r-', label='Robot 1 X')
axs1[0, 0].plot(time, df[robot1_cols['y']], 'g-', label='Robot 1 Y')
axs1[0, 0].plot(time, df[robot1_cols['z']], 'b-', label='Robot 1 Z')
axs1[0, 0].plot(time, df[robot2_cols['x']], 'r--', label='Robot 2 X')
axs1[0, 0].plot(time, df[robot2_cols['y']], 'g--', label='Robot 2 Y')
axs1[0, 0].plot(time, df[robot2_cols['z']], 'b--', label='Robot 2 Z')
axs1[0, 0].set_title('Position')
axs1[0, 0].set_ylabel('Position (m)')
axs1[0, 0].legend()

# Rotation plots
axs1[0, 1].plot(time, df[robot1_cols['rx']], 'r-', label='Robot 1 RX')
axs1[0, 1].plot(time, df[robot1_cols['ry']], 'g-', label='Robot 1 RY')
axs1[0, 1].plot(time, df[robot1_cols['rz']], 'b-', label='Robot 1 RZ')
axs1[0, 1].plot(time, df[robot2_cols['rx']], 'r--', label='Robot 2 RX')
axs1[0, 1].plot(time, df[robot2_cols['ry']], 'g--', label='Robot 2 RY')
axs1[0, 1].plot(time, df[robot2_cols['rz']], 'b--', label='Robot 2 RZ')
axs1[0, 1].set_title('Rotation')
axs1[0, 1].set_ylabel('Rotation (rad)')
axs1[0, 1].legend()

# Velocity plots
axs1[1, 0].plot(time, df[robot1_cols['xdot']], 'r-', label='Robot 1 X')
axs1[1, 0].plot(time, df[robot1_cols['ydot']], 'g-', label='Robot 1 Y')
axs1[1, 0].plot(time, df[robot1_cols['zdot']], 'b-', label='Robot 1 Z')
axs1[1, 0].plot(time, df[robot2_cols['xdot']], 'r--', label='Robot 2 X')
axs1[1, 0].plot(time, df[robot2_cols['ydot']], 'g--', label='Robot 2 Y')
axs1[1, 0].plot(time, df[robot2_cols['zdot']], 'b--', label='Robot 2 Z')
axs1[1, 0].set_title('Linear Velocity')
axs1[1, 0].set_ylabel('Velocity (m/s)')
axs1[1, 0].legend()

# Angular velocity plots
axs1[1, 1].plot(time, df[robot1_cols['rx_dot']], 'r-', label='Robot 1 RX')
axs1[1, 1].plot(time, df[robot1_cols['ry_dot']], 'g-', label='Robot 1 RY')
axs1[1, 1].plot(time, df[robot1_cols['rz_dot']], 'b-', label='Robot 1 RZ')
axs1[1, 1].plot(time, df[robot2_cols['rx_dot']], 'r--', label='Robot 2 RX')
axs1[1, 1].plot(time, df[robot2_cols['ry_dot']], 'g--', label='Robot 2 RY')
axs1[1, 1].plot(time, df[robot2_cols['rz_dot']], 'b--', label='Robot 2 RZ')
axs1[1, 1].set_title('Angular Velocity')
axs1[1, 1].set_ylabel('Angular Velocity (rad/s)')
axs1[1, 1].legend()

# Force plots
axs1[2, 0].plot(time, df[robot1_cols['fx']], 'r-', label='Robot 1 FX')
axs1[2, 0].plot(time, df[robot1_cols['fy']], 'g-', label='Robot 1 FY')
axs1[2, 0].plot(time, df[robot1_cols['fz']], 'b-', label='Robot 1 FZ')
axs1[2, 0].plot(time, df[robot2_cols['fx']], 'r--', label='Robot 2 FX')
axs1[2, 0].plot(time, df[robot2_cols['fy']], 'g--', label='Robot 2 FY')
axs1[2, 0].plot(time, df[robot2_cols['fz']], 'b--', label='Robot 2 FZ')
axs1[2, 0].set_title('Forces')
axs1[2, 0].set_ylabel('Force (N)')
axs1[2, 0].set_xlabel('Time (samples)')
axs1[2, 0].legend()

# Torque plots
axs1[2, 1].plot(time, df[robot1_cols['tau_x']], 'r-', label='Robot 1 X')
axs1[2, 1].plot(time, df[robot1_cols['tau_y']], 'g-', label='Robot 1 Y')
axs1[2, 1].plot(time, df[robot1_cols['tau_z']], 'b-', label='Robot 1 Z')
axs1[2, 1].plot(time, df[robot2_cols['tau_x']], 'r--', label='Robot 2 X')
axs1[2, 1].plot(time, df[robot2_cols['tau_y']], 'g--', label='Robot 2 Y')
axs1[2, 1].plot(time, df[robot2_cols['tau_z']], 'b--', label='Robot 2 Z')
axs1[2, 1].set_title('Torques')
axs1[2, 1].set_ylabel('Torque (Nm)')
axs1[2, 1].set_xlabel('Time (samples)')
axs1[2, 1].legend()

plt.tight_layout(rect=[0, 0, 1, 0.96])

# Plot third and fourth robots in a new window
fig2, axs2 = plt.subplots(3, 2, figsize=(14, 10))
fig2.suptitle('Robot Motion Data - Robots 3 & 4', fontsize=16)

# Position plots
axs2[0, 0].plot(time, df[robot3_cols['x']], 'r-', label='Robot 3 X')
axs2[0, 0].plot(time, df[robot3_cols['y']], 'g-', label='Robot 3 Y')
axs2[0, 0].plot(time, df[robot3_cols['z']], 'b-', label='Robot 3 Z')
axs2[0, 0].plot(time, df[robot4_cols['x']], 'r--', label='Robot 4 X')
axs2[0, 0].plot(time, df[robot4_cols['y']], 'g--', label='Robot 4 Y')
axs2[0, 0].plot(time, df[robot4_cols['z']], 'b--', label='Robot 4 Z')
axs2[0, 0].set_title('Position')
axs2[0, 0].set_ylabel('Position (m)')
axs2[0, 0].legend()

# Rotation plots
axs2[0, 1].plot(time, df[robot3_cols['rx']], 'r-', label='Robot 3 RX')
axs2[0, 1].plot(time, df[robot3_cols['ry']], 'g-', label='Robot 3 RY')
axs2[0, 1].plot(time, df[robot3_cols['rz']], 'b-', label='Robot 3 RZ')
axs2[0, 1].plot(time, df[robot4_cols['rx']], 'r--', label='Robot 4 RX')
axs2[0, 1].plot(time, df[robot4_cols['ry']], 'g--', label='Robot 4 RY')
axs2[0, 1].plot(time, df[robot4_cols['rz']], 'b--', label='Robot 4 RZ')
axs2[0, 1].set_title('Rotation')
axs2[0, 1].set_ylabel('Rotation (rad)')
axs2[0, 1].legend()

# Velocity plots
axs2[1, 0].plot(time, df[robot3_cols['xdot']], 'r-', label='Robot 3 X')
axs2[1, 0].plot(time, df[robot3_cols['ydot']], 'g-', label='Robot 3 Y')
axs2[1, 0].plot(time, df[robot3_cols['zdot']], 'b-', label='Robot 3 Z')
axs2[1, 0].plot(time, df[robot4_cols['xdot']], 'r--', label='Robot 4 X')
axs2[1, 0].plot(time, df[robot4_cols['ydot']], 'g--', label='Robot 4 Y')
axs2[1, 0].plot(time, df[robot4_cols['zdot']], 'b--', label='Robot 4 Z')
axs2[1, 0].set_title('Linear Velocity')
axs2[1, 0].set_ylabel('Velocity (m/s)')
axs2[1, 0].legend()

# Angular velocity plots
axs2[1, 1].plot(time, df[robot3_cols['rx_dot']], 'r-', label='Robot 3 RX')
axs2[1, 1].plot(time, df[robot3_cols['ry_dot']], 'g-', label='Robot 3 RY')
axs2[1, 1].plot(time, df[robot3_cols['rz_dot']], 'b-', label='Robot 3 RZ')
axs2[1, 1].plot(time, df[robot4_cols['rx_dot']], 'r--', label='Robot 4 RX')
axs2[1, 1].plot(time, df[robot4_cols['ry_dot']], 'g--', label='Robot 4 RY')
axs2[1, 1].plot(time, df[robot4_cols['rz_dot']], 'b--', label='Robot 4 RZ')
axs2[1, 1].set_title('Angular Velocity')
axs2[1, 1].set_ylabel('Angular Velocity (rad/s)')
axs2[1, 1].legend()

# Force plots
axs2[2, 0].plot(time, df[robot3_cols['fx']], 'r-', label='Robot 3 FX')
axs2[2, 0].plot(time, df[robot3_cols['fy']], 'g-', label='Robot 3 FY')
axs2[2, 0].plot(time, df[robot3_cols['fz']], 'b-', label='Robot 3 FZ')
axs2[2, 0].plot(time, df[robot4_cols['fx']], 'r--', label='Robot 4 FX')
axs2[2, 0].plot(time, df[robot4_cols['fy']], 'g--', label='Robot 4 FY')
axs2[2, 0].plot(time, df[robot4_cols['fz']], 'b--', label='Robot 4 FZ')
axs2[2, 0].set_title('Forces')
axs2[2, 0].set_ylabel('Force (N)')
axs2[2, 0].set_xlabel('Time (samples)')
axs2[2, 0].legend()

# Torque plots
axs2[2, 1].plot(time, df[robot3_cols['tau_x']], 'r-', label='Robot 3 X')
axs2[2, 1].plot(time, df[robot3_cols['tau_y']], 'g-', label='Robot 3 Y')
axs2[2, 1].plot(time, df[robot3_cols['tau_z']], 'b-', label='Robot 3 Z')
axs2[2, 1].plot(time, df[robot4_cols['tau_x']], 'r--', label='Robot 4 X')
axs2[2, 1].plot(time, df[robot4_cols['tau_y']], 'g--', label='Robot 4 Y')
axs2[2, 1].plot(time, df[robot4_cols['tau_z']], 'b--', label='Robot 4 Z')
axs2[2, 1].set_title('Torques')
axs2[2, 1].set_ylabel('Torque (Nm)')
axs2[2, 1].set_xlabel('Time (samples)')
axs2[2, 1].legend()

plt.tight_layout(rect=[0, 0, 1, 0.96])
plt.show()