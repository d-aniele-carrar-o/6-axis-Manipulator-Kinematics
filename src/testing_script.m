clc;
close all;

% Load useful parameters for trajectory generation and simulation
parameters(0)
%%
% - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
% TESTING SCRIPT FOR DK, IK, IDIFFK, TRAJ GENERATION
ax = show(robot, config);
xlim(ax, [-0.5, 0.5]);  % Custom X limits
ylim(ax, [-0.5, 0.5]);  % Custom Y limits
zlim(ax, [0, 1.0]);     % Custom Z limits

pause()
close all;

% Initial joint configuration
% q0 = [0.33, pi/6, -1.189, 2.127, 0.563, -2.138]
q0 = [-4.7124, -1.8098, -2.1206, -2.3527, -pi/2, 0.0]

% Compute end-effector pose
Te = direct_kinematics_cpp( q0, AL, A, D, TH )

direct_kinematics_draw( q0, nan, true );

% Set true to test the solutions of IK
test_IK = false;
if test_IK
    test_Inverse_Kinematics( q0 );
end

pause()

% Compute Jacobian matrix for manipulator
% J = Jacobian_cpp( Te, q0, AL, A, D, TH )

% TRAJECTORY GENERATION ---------------------------------------------------
% Trajectory initial time and time periods for each piece of trajectory
ti    = 0;

% First viapoint - set first desired configuration/pose (viapoint)
phi_f = [-pi/2, 0, 0];
Rf = eul2rotm_custom( phi_f );

if     manipulator == "UR5"
    % pf = Te(1:3,4) + [0.0; 0.35; 0.0];
    pf = [0; 0.5; 0.5];
elseif manipulator == "ABB"
    pf = Te(1:3,4) + [0.0; 1.8; 0.0];
elseif manipulator == "custom"
    q0 = [0.33, 0.9052, -1.189, 2.127, 0.563, -2.138];
    phi_f = [7/6*pi, 0, pi/2];
    Rf    = eul2rotm_custom( phi_f );
    pf    = [0.2; 0.2; 0.06];
end
Tf = [Rf, pf; 0,0,0,1]

% Second viapoint: set second desired configuration/pose (viapoint)
if     manipulator == "UR5"
    Tf2 = Tf + [zeros(3), [-0.4; 0; 0]; 0,0,0,0]
elseif manipulator == "ABB"
    Tf2 = Tf + [zeros(3), [-1.8; 0; 0]; 0,0,0,0]
elseif manipulator == "custom"
    Tf2 = Tf + [zeros(3), [-0.4; 0; 0]; 0,0,0,0]
end

% Third viapoint: set second desired configuration/pose (viapoint)
if     manipulator == "UR5"
    Tf3 = Tf2 + [zeros(3), [0; -0.4; 0]; 0,0,0,0]
elseif manipulator == "ABB"
    Tf3 = Tf2 + [zeros(3), [0; -1.8; 0]; 0,0,0,0]
elseif manipulator == "custom"
    Tf3 = Tf2 + [zeros(3), [0; -0.4; 0]; 0,0,0,0]
end

% Fourth viapoint: set second desired configuration/pose (viapoint)
if     manipulator == "UR5"
    Tf4 = Tf3 + [zeros(3), [0.4; 0; 0]; 0,0,0,0]
elseif manipulator == "ABB"
    Tf4 = Tf3 + [zeros(3), [1.8; 0; 0]; 0,0,0,0]
elseif manipulator == "custom"
    Tf4 = Tf3 + [zeros(3), [0.4; 0; 0]; 0,0,0,0]
end

viapoints = [Tf];
times     = [ti, 1];

% Compute multi-viapoint trajectory for selected times and viapoints
[t, p, v] = multipoint_trajectory( q0, viapoints, times );
% writematrix(round(p, 5), "ur5_coppeliasim/python/coppeliasim_connection/trajectory.txt")

% Simulate trajectory
[qf, handlesR] = simulate( robot, t, p, [], v, q0, NaN, false );
