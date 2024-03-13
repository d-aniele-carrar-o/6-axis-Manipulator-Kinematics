clc;
close all;

% Load useful parameters - like dt for trajectory generation, D-H parameters for selected manipulator
params


% - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
% TESTING SCRIPT FOR DK, IK, IDIFFK, TRAJ GENERATION

% Initial joint configuration
q0 = [0.33, 2.476, -1.189, 2.127, 0.563, -2.138]

% Compute end-effector pose
Te = direct_kinematics_cpp( q0, AL, A, D, TH )

draw = false;
if draw
    % Setup plot figure
    axs = axes(); view( 3 ); grid on;

    % Plot the configuration of the manipulator
    [~, handlesR] = direct_kinematics_draw( q0, axs, true );
    
    % Compute Inverse Kinematics for just computed pose to check that each of
    % the eight resulting joint configurations results in the same pose and plot them
    H = UR5_inverse_kinematics_cpp( Te(1:3,4), Te(1:3,1:3), AL, A, D )
    for i=1:8
        % Overlapped views
        [T, ~] = direct_kinematics_draw( H(i,:), axs, true );
        
        % NOT overlapped views
        [T, ~] = direct_kinematics_draw( H(i,:), handlesR, false );

        fprintf( "Press enter to continue\n" )
        pause()
    end
end

% Compute Jacobian matrix for manipulator
% J = Jacobian_cpp( Te, q0, AL, A, D, TH )

% TRAJECTORY GENERATION ---------------------------------------------------
% Trajectory initial and final times
ti = 0;
tf = 1;

% First end-point
phi_f = [0, pi/2, pi];
Rf    = eul2rotm_custom( phi_f );
pf    = Te(1:3,4) + [0.0; 0.35; 0.0];
Tf    = [Rf, pf; 0,0,0,1]

% Set first desired configuration/pose
if space == "joint"
    Hf = UR5_inverse_kinematics_cpp( pf, Rf, AL, A, D );
    qf = get_closer( Hf, q0 )
else
    qf = Tf;
end

% Generate trajectory
[t1, p1, v1] = generate_trajectory( q0, qf, ti, tf );

% Set final configuration/pose
if space == "joint" || kinematics == "IDK"
    qf = p1(end,:)
else
    Hf = UR5_inverse_kinematics_cpp( pf, Rf, AL, A, D );
    qf = get_closer( Hf, q0 )
end

% Second end-point
Tf2 = Tf - [zeros(3), [0.4; 0; 0]; 0,0,0,0]
Rf2 = Tf2(1:3,1:3);
pf2 = Tf2(1:3,4);

% Set second desired configuration/pose
if space == "joint"
    Hf2 = UR5_inverse_kinematics_cpp( pf2, Rf2, AL, A, D );
    qf2 = get_closer( Hf2, qf )
else
    qf2 = Tf2;
end

% Generate second piece of trajectory
[t2, p2, v2] = generate_trajectory( qf, qf2, ti, tf );

% Simulate trajectory
[qf, handlesR] = simulate( [t1,t1(end)+t2], [p1; p2], [], [v1; v2], q0, NaN, false );



