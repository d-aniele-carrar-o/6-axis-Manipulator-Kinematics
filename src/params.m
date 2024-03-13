addpath("utils/")
addpath("plot_utils/")
addpath("ABB_scripts/")
addpath("UR5_scripts/")
addpath("3_link_manipulators/")
addpath("trajectory_scripts/")
addpath("robot_descriptions/")
addpath("robot_urdf/")
addpath("cpp_src/")
addpath("mex_compiled_functions/")


% Debug mode
verbose = false;

% Trajectory generation parameters ----------------------------------------

% Time interval constant used for trajectory generation
dt      = 0.01;

% Refreshing rate for simulation update
rate    = rateControl( 1/(dt*2) );

% Maximum velocity and acceleration for trajectory generation
max_vel = 10;
max_acc = 10;

space      = "task";
kinematics = "IDK";
traj_type  = "quintic";

if kinematics == "IDK"
    % Precision of final pose - determines when the loop ends (IDK)
    precision_pos    = 0.001;
    precision_orient = 0.001;
    
    % Safety parameter which allows the loop to stop if unreachable poses are given (IDK)
    count = 5;
end
% -------------------------------------------------------------------------

% Plot parameters
plot_grahps = true;


% Manipulator in use for which compute some kinematics (for now just ABB and UR5)
manipulator = "UR5";

% Use the robot's urdf to show the simulation, if false the simulations uses matlab's plot3
real_robot  = true;

% Eventual transformation between World Reference Frame and Frame 0
Trf_0       = eye(4);


% Load manipulator's D-H parameters and other useful parameters to compute kinematics
if manipulator == "ABB"  % ================================================
    % ABB IRb-7600 D-H, joint limits, and other useful stuff

    % Axis limits for IRB 7600-400/2.55, 404 mm for LeanID (not used yet)
    limits = [deg2rad(180), deg2rad(-180);  % q1
              deg2rad(85),  deg2rad(-60);   % q2
              deg2rad(60),  deg2rad(-180);  % q3
              deg2rad(300), deg2rad(-300);  % q4
              deg2rad(100), deg2rad(-100);  % q5
              deg2rad(220), deg2rad(-220);  % q6
              ];
    
    % ABB D-H parameters:
    %   T :      0->1    1->2    2->3    3->4    4->5    5->6
    %   i = |  0  |   1   |   2   |   3   |   4   |   5   |   6   |
       AL = [    0,   pi/2,      0,   pi/2,  -pi/2,   pi/2,   -0  ];
        A = [    0,   0.41,  1.075,  0.165,      0,      0,   -0  ];
        D = [  -0 ,   0.78,      0,      0,  1.056,      0,   0.25];
       TH = [  -0 ,      0,      0,      0,      0,      0,      0];
    %  th = |  -  |  th1  |  th2  |  th3  |  th4  |  th5  |  th6  |

    % Robot's urdf still not available
    real_robot = false;

elseif manipulator == "UR5"  % ============================================
    % UR5 D-H parameters
    %   T :         0->1      1->2      2->3      3->4      4->5      5->6     6->ee
    %   i = |   0    |    1    |    2    |    3    |    4    |    5    |    6   |   ee   |
       AL = [       0,     pi/2,        0,        0,     pi/2,    -pi/2,       0,   -0   ];
        A = [       0,        0,   -0.425, -0.39225,        0,        0,       0,   -0   ];
        D = [   -0   , 0.089159,        0,        0,  0.10915,  0.09465,  0.0823,  0.1475];
       TH = [   -0   ,        0,        0,        0,        0,        0,       0,       0];
    %  th = |   -    |    th1  |   th2   |   th3   |   th4   |   th5   |   th6  |   -    |
    
    if real_robot
        robot  = importrobot( "robot_urdf/ur5.urdf" );
        % show( robot, Visuals="on", Collisions="off" )
    end

elseif manipulator == "custom"  % ======================================
    % Custom manipulator D-H parameters
    %   T :         0->1      1->2      2->3      3->4      4->5      5->6
    %   i = |   0    |    1    |    2    |    3    |    4    |    5    |    6   |
       AL = [       0,     pi/2,        0,     pi/2,    -pi/2,     pi/2,    -0  ];
        A = [       0,        0,     0.15,     0.07,        0,        0,    -0  ];
        D = [   -0   ,     0.06,        0,        0,     0.13,        0,   0.031];
       TH = [   -0   ,        0,     pi/2,        0,        0,        0,       0];
    %  th = |   -    |    th1  |   th2   |   th3   |   th4   |   th5   |   th6  |
    
    % Robot's urdf still not available
    real_robot = false;

else
    fprintf( "Undefined manipulator selected. Terminating." )
    exit

end



