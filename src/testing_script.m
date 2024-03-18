clc;
close all;

% Load useful parameters - like dt for trajectory generation, D-H parameters for selected manipulator
parameters(0)
%%
% - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
% TESTING SCRIPT FOR DK, IK, IDIFFK, TRAJ GENERATION

% Initial joint configuration
q0 = [0.33, 2.476, -1.189, 2.127, 0.563, -2.138]

% Compute end-effector pose
Te = direct_kinematics_cpp( q0, AL, A, D, TH )

% Set true to test the solutions of IK
test_IK = false;
if test_IK
    test_Inverse_Kinematics( q0 );
end

% Compute Jacobian matrix for manipulator
% J = Jacobian_cpp( Te, q0, AL, A, D, TH )

% TRAJECTORY GENERATION ---------------------------------------------------
% Trajectory initial time and time periods for each piece of trajectory
ti = 0;
T1 = 1;
T2 = 1;

% First viapoint - set first desired configuration/pose (viapoint)
phi_f = [0, pi/2, pi];
Rf    = eul2rotm_custom( phi_f );

if     manipulator == "UR5"
    pf = Te(1:3,4) + [0.0; 0.35; 0.0];
    Tf = [Rf, pf; 0,0,0,1]
elseif manipulator == "ABB"
    pf = Te(1:3,4) + [0.0; 1.8; 0.0];
    Tf = [Rf, pf; 0,0,0,1]
elseif manipulator == "custom"
    pf = Te(1:3,4) + [0.0; 0.35; 0.0];
    Tf = [Rf, pf; 0,0,0,1]
end

% Second viapoint: set second desired configuration/pose (viapoint)
if     manipulator == "UR5"
    Tf2 = Tf + [zeros(3), [-0.4; 0; 0]; 0,0,0,0]
elseif manipulator == "ABB"
    Tf2 = Tf + [zeros(3), [-1.8; 0; 0]; 0,0,0,0]
elseif manipulator == "custom"
    Tf2 = Tf + [zeros(3), [-0.4; 0; 0]; 0,0,0,0]
end

viapoints = [Tf; Tf2];
times     = [ti, T1, T2];

% Compute multi-viapoint trajectory for selected times and viapoints
[t, p, v] = multipoint_trajectory( q0, viapoints, times );

% Simulate trajectory
[qf, handlesR] = simulate( t, p, [], v, q0, NaN, false );



