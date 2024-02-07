addpath("utils/")
addpath("trajectory_generation/")
addpath("ABB_scripts/")
addpath("UR5_scripts/")
addpath("3_link_manipulators/")

% time interval constant used for trajectory generation
dt = 0.1;

% manipulator in use for which compute some kinematics (for now just ABB and UR5)
manipulator = "UR5";

% load manipulator's D-H parameters and other useful parameters to compute kinematics
if     manipulator == "ABB"  % ==============================================
    % ABB IRb-7600 D-H and other useful parameters

    % axis limits for IRB 7600-400/2.55, 404 mm for LeanID (not used yet)
    limits = [deg2rad(180), deg2rad(-180);  % q1
              deg2rad(85),  deg2rad(-60);   % q2
              deg2rad(60),  deg2rad(-180);  % q3
              deg2rad(300), deg2rad(-300);  % q4
              deg2rad(100), deg2rad(-100);  % q5
              deg2rad(220), deg2rad(-220);  % q6
              ];
    
    % D-H parameters:
    %   T :      0->1    1->2    2->3    3->4    4->5    5->6
    %   i = |  0  |   1   |   2   |   3   |   4   |   5   |   6   |
       AL = [    0,   pi/2,      0,   pi/2,  -pi/2,   pi/2,   -0  ];
        A = [    0,   0.41,  1.075,  0.165,      0,      0,   -0  ];
        D = [  -0 ,   0.78,      0,      0,  1.056,      0,  0.25 ];
    %  th = |  -  |  th1  |  th2  |  th3  |  th4  |  th5  |  th6  |

elseif manipulator == "UR5"  % ==============================================
    % UR5 D-H parameters
    %   T :         0->1      1->2      2->3      3->4      4->5      5->6
    %   i = |   0    |    1    |    2    |    3    |    4    |    5    |    6   |
       AL = [       0,     pi/2,        0,        0,     pi/2,    -pi/2,    -0  ];
        A = [       0,        0,   -0.425, -0.39225,        0,        0,    -0  ];
        D = [   -0   , 0.089159,        0,        0,  0.10915,  0.09465,  0.0823];
    %  th = |   -    |    th1  |   th2   |   th3   |   th4   |   th5   |   th6  |

else
    fprintf( "Undefined manipulator selected. Terminating." )
    exit
end
