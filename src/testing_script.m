% clc; close all;

% TESTING SCRIPT FOR DK, IK, IDIFFK, TRAJ GENERATION ==============================================
% Load parameters
parameters(0)

% Initial joint configuration
% q0 = [0, -pi/6, -pi/2, -pi/3, -pi/2, 0.0]
% q0 = [pi, -pi/2, 0, -pi/2, -pi/2, 0]
q0 = [-pi/2, -2*pi/3, -2*pi/3, -2*pi/3, -pi/2, 0]

% Compute end-effector pose
[T_w_e, Te] = direct_kinematics( q0 )
direct_kinematics_draw( robot, config, q0, NaN, true );

% Set true to test the solutions of IK
test_IK = false;
if test_IK
    test_Inverse_Kinematics( q0 );
end

pause()

% Compute Jacobian matrix for manipulator
% J = Jacobian_cpp( Te, q0, AL, A, D, TH )

% TRAJECTORY GENERATION ---------------------------------------------------------------------------
% Trajectory initial time and time periods for each piece of trajectory
ti    = 0;

% First viapoint - set first desired configuration/pose (viapoint)
phi_f = [0, pi/2, 0];
Rf    = Te(1:3,1:3) * eul2rotm_custom( phi_f );

if     manipulator == "UR5"
    pf = [0; 0.5; 0.5];
elseif manipulator == "UR3e"
    pf = Te(1:3,4) + [0.4; 0.2; 0];
elseif manipulator == "ABB"
    pf = Te(1:3,4) + [0.0; 1.8; 0.0];
elseif manipulator == "custom"
    q0    = [0.33, 0.9052, -1.189, 2.127, 0.563, -2.138];
    phi_f = [7/6*pi, 0, pi/2];
    Rf    = eul2rotm_custom( phi_f );
    pf    = [0.2; 0.2; 0.06];
end
Tf = [Rf, pf; 0,0,0,1]
t1 = 1

% Second viapoint: set second desired configuration/pose (viapoint)
if     manipulator == "UR5"
    Tf2 = Tf + [zeros(3), [-0.4; 0; 0]; 0,0,0,0]
elseif manipulator == "UR3e"
    Tf2 = Tf + [zeros(3), [0.2; 0.4; 0]; 0,0,0,0]
elseif manipulator == "ABB"
    Tf2 = Tf + [zeros(3), [-1.8; 0; 0]; 0,0,0,0]
elseif manipulator == "custom"
    Tf2 = Tf + [zeros(3), [-0.4; 0; 0]; 0,0,0,0]
end
t2 = 2;

% Third viapoint: set second desired configuration/pose (viapoint)
if     manipulator == "UR5"
    Tf3 = Tf2 + [zeros(3), [0; -0.4; 0]; 0,0,0,0]
elseif manipulator == "ABB"
    Tf3 = Tf2 + [zeros(3), [0; -1.8; 0]; 0,0,0,0]
elseif manipulator == "custom"
    Tf3 = Tf2 + [zeros(3), [0; -0.4; 0]; 0,0,0,0]
end
t3 = 3;

% Fourth viapoint: set second desired configuration/pose (viapoint)
if     manipulator == "UR5"
    Tf4 = Tf3 + [zeros(3), [0.4; 0; 0]; 0,0,0,0]
elseif manipulator == "ABB"
    Tf4 = Tf3 + [zeros(3), [1.8; 0; 0]; 0,0,0,0]
elseif manipulator == "custom"
    Tf4 = Tf3 + [zeros(3), [0.4; 0; 0]; 0,0,0,0]
end
t4 = 4;

viapoints = [Tf; Tf2];
times     = [ti, t1, t2];

% Compute multi-viapoint trajectory for selected times and viapoints
[t, p, v] = multipoint_trajectory( q0, viapoints, times );

% Simulate trajectory
[qf, handlesR] = simulate( robot, config, t, p, [], v, q0, NaN, false );
Tf_f = direct_kinematics_cpp( qf, AL, A, D, TH )
T_w_e_f = Trf_0 * Tf_f
