function parameters( level, robot_id )
    % Debug mode
    verbose = false;
    
    % Handle optional robot_id parameter
    if nargin < 2
        robot_id = 1; % Default to first robot if not specified
    end
    
    % Trajectory generation parameters ------------------------------------------------------------
    
    % Time interval constant used for trajectory generation
    dt      = 0.02;
    
    % Refreshing rate for simulation update
    rate    = rateControl( 1/(2*dt) );
    
    % Maximum velocity and acceleration for trajectory generation
    max_vel = 10;
    max_acc = 10;
    
    space      = "task";
    % ["joint", "task"]

    kinematics = "IK";
    % ["IK", "IDK"]

    traj_type  = "quintic";
    % ["LSPB", "cubic", "quintic"]
    
    if kinematics == "IDK"
        % Precision of final pose - determines when the loop ends (IDK)
        precision_pos    = 0.001;
        precision_orient = 0.001;
        
        % Safety parameter which allows the loop to stop if unreachable poses are given (IDK)
        count = 5;
    end
    
    % Plot parameters
    plot_grahps = false;
    % ---------------------------------------------------------------------------------------------

    robot_types = ["UR3e", "UR3e"];  % each option can be ["ABB", "UR5", "UR3e", "custom"]
    
    % Select current robot based on robot_id
    % Manipulator in use for which compute kinematics for
    manipulator = robot_types(robot_id);

    % Use robot's urdf for visualization
    real_robot = true;

    % Gripper presence depends on robot type
    gripper = (manipulator == "UR5" || manipulator == "custom");

    % Load manipulator's D-H parameters and other useful parameters to compute kinematics
    if     manipulator == "UR5"  % ================================================================
        % UR5 D-H parameters
        %   T :         0->1      1->2      2->3      3->4      4->5      5->6     6->ee
        %   i = |   0    |    1    |    2    |    3    |    4    |    5    |    6   |   ee   |
        AL = [       0,     pi/2,        0,        0,     pi/2,    -pi/2,       0,   -0   ];
        A  = [       0,        0,   -0.425, -0.39225,        0,        0,       0,   -0   ];
        D  = [   -0   , 0.089159,        0,        0,  0.10915,  0.09465,  0.0823,  0.0674]; % 0.1475
        TH = [   -0   ,        0,    -pi/2,        0,    -pi/2,        0,       0,       0];
        %  th = |   -    |    th1  |   th2   |   th3   |   th4   |   th5   |   th6  |   -    |
    
    elseif manipulator == "UR3e"  % ===============================================================
        % UR3e D-H parameters
        %   T :         0->1      1->2      2->3      3->4      4->5      5->6     6->ee
        %   i = |   0    |    1    |    2    |    3    |    4    |    5    |    6   |   ee   |
        AL = [       0,     pi/2,        0,        0,     pi/2,    -pi/2,       0,   -0   ];
        A  = [       0,        0, -0.24355,  -0.2132,        0,        0,       0,   -0   ];
        D  = [   -0   ,  0.15185,        0,        0,  0.13105,  0.08535,  0.0921,       0];
        TH = [   -0   ,        0,        0,        0,        0,        0,       0,       0];
        %  th = |   -    |    th1  |   th2   |   th3   |   th4   |   th5   |   th6  |   -    |
    
    elseif manipulator == "ABB"  % ================================================================
        % ABB IRb-7600 D-H, joint limits, and other useful stuff
    
        % Axis limits for IRB 7600-400/2.55, 404 mm for LeanID (not used yet)
        % limits = [deg2rad(180), deg2rad(-180);  % q1
        %           deg2rad(85),  deg2rad(-60);   % q2
        %           deg2rad(60),  deg2rad(-180);  % q3
        %           deg2rad(300), deg2rad(-300);  % q4
        %           deg2rad(100), deg2rad(-100);  % q5
        %           deg2rad(220), deg2rad(-220);  % q6
        %           ];
        
        % ABB D-H parameters:
        %   T :      0->1    1->2    2->3    3->4    4->5    5->6
        %   i = |  0  |   1   |   2   |   3   |   4   |   5   |   6   |
        AL = [    0,   pi/2,      0,   pi/2,  -pi/2,   pi/2,   -0  ];
        A  = [    0,   0.41,  1.075,  0.165,      0,      0,   -0  ];
        D  = [  -0 ,   0.78,      0,      0,  2.012,      0,   0.25];
        TH = [  -0 ,      0,      0,      0,      0,      0,      0];
        %  th = |  -  |  th1  |  th2  |  th3  |  th4  |  th5  |  th6  |

    elseif manipulator == "custom"  % =============================================================
        % Custom manipulator D-H parameters
        %   T :         0->1      1->2      2->3      3->4      4->5      5->6     6->ee
        %   i = |   0    |    1    |    2    |    3    |    4    |    5    |    6   |  ee  |
        AL = [       0,     pi/2,        0,     pi/2,    -pi/2,     pi/2,       0,  -0  ];
        A  = [       0,        0,     0.15,     0.07,        0,        0,       0,  -0  ];
        D  = [   -0   ,     0.06,        0,        0,     0.13,        0,   0.031, 0.064];
        TH = [   -0   ,        0,        0,     pi/2,        0,        0,       0,     0];
        %  th = |   -    |    th1  |   th2   |   th3   |   th4   |   th5   |   th6  | thee |
        
    else
        fprintf( "Undefined manipulator selected. Terminating." )
        exit
    
    end


    % Define useful parameters for evnvironment simulation ========================================
    % Set table dimensions
    tableHeight    = 0.76;
    tableWidth     = 1.5;
    tableLength    = 0.75;
    tableParams.height = tableHeight;
    tableParams.width  = tableWidth;
    tableParams.length = tableLength;

    tablePosition  = [0.0, 0.0, tableHeight];
    tableRoI       = [0.9, 0.65, 0.4];  % RoI for plane fitting in camera coords
    standHeight    = 0.215;
    robotsDistance = 1.12;  % Distance between robot's base origins

    % Transformation between World Reference Frame and Robot's Base Frame
    basePosition    = tablePosition;  % Position in [x, y, z]
    baseOrientation = [0, 0, 0];      % Orientation in [roll, pitch, yaw]
    
    % Define robot's world-to-base transformations
    if nargin > 1  % multi-robot setup
        robot_base_transforms = {
            % Robot LEFT  pose
            eul2tform(baseOrientation, "XYZ") * trvec2tform(basePosition + [0, -robotsDistance/2, standHeight]) * eul2tform(baseOrientation + [0, 0, pi], "XYZ");
            % Robot RIGHT pose
            eul2tform(baseOrientation, "XYZ") * trvec2tform(basePosition + [0,  robotsDistance/2, standHeight])
        };
    else
        robot_base_transforms = {eul2tform([0, 0, 0], "XYZ") * trvec2tform(basePosition + [0, -0.4, 0] ) * eul2tform(baseOrientation + [0, 0, pi], "XYZ")};
    end
    
    Trf_0 = robot_base_transforms{robot_id};
    if level == 0 && real_robot
        disp("Loading robot " + manipulator + " " + num2str(robot_id))
        [robot, config] = get_robot( manipulator, Trf_0 );
    end


    % Useful folders ==============================================================================
    data_folder = '/Users/danielecarraro/Documents/VSCODE/data/';
    motion_data_folder       = [data_folder, 'data/'];
    segmented_objects_folder = [data_folder, 'output/segmented_objects'];
    augmented_demos_path     = [data_folder, 'output/augmented_demos'];


    % Export variables to workspace ===============================================================
    varnames = who;
    for i = 1:length(varnames)
        assignin( 'caller', varnames{i}, eval(varnames{i}) );
    end

end
