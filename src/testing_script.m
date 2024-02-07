clear all;
close all;
clc;

% Setup plot figure
axs   = axes( 'XLim', [-0.5, 0.5], 'YLim', [-0.5, 0.5], 'ZLim', [-0.7, 0.7] ); 
view(3);
grid on;
handles(1) = axs;

% Load useful parameters - like dt for trajectory generation, D-H parameters for selected manipulator
params

% - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
% TESTING SCRIPT FOR DK, IK, IDIFFK, TRAJ GENERATION

% Initial joint configuration
q0 = [0.33, 2.476, -1.189, 2.127, 0.563, -2.138]

% Compute end-effector pose
T0 = UR5_direct_kinematics( q0 )

% Plot the configuration of the manipulator
[~, handlesR] = UR5_direct_kinematics_draw( q0, handles, true );
pause()

% Compute Inverse Kinematics for just computed pose to check that each of
% the eight resulting configurations results in the same pose and plot them
TH = UR5_inverse_kinematics( T0(1:3,4), T0(1:3,1:3) )
for i=1:8
    % Overlapped views
    [T, ~] = UR5_direct_kinematics_draw( TH(i,:), handles, true )

    % NOT overlapped views
    % [T, ~] = UR5_direct_kinematics_draw( TH(i,:), handlesR, false )

    fprintf( "Press enter to continue\n" )
    pause()
end

% Generate a desired pose for the end-effector
phi_f = [0, pi/2, pi];
R_f = eul2rotm( phi_f );
p_f = T0(1:3,4) + [0; -0.2; 0];
T_f = [R_f, p_f; 0,0,0,1];

% Time execution period for the trajectory (could be more if velocities exceed max_vel)
T = 2;

% Generate trajectory with desired position/velocity profile
[pos, vel] = generate_trajectory( q0, T_f, T, "cubic" );

% Number of generated viapoints
N = max(size(pos))

% Plot trajectory
for i=1:N
    UR5_direct_kinematics_draw( pos(i,:), handlesR, false );
    pause(0.1)
end

