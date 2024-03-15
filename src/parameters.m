function parameters( level )
    if level == 0
        addpath("utils/")
        addpath("plot_utils/")
        addpath("3_link_manipulators/")
        addpath("trajectory_scripts/")
        addpath("robot_descriptions/")
        addpath("robot_urdf/")
        addpath("cpp_src/")
        addpath("mex_compiled_functions/")
    end
    
    % Debug mode
    verbose = false;
    
    % Trajectory generation parameters ----------------------------------------
    
    % Time interval constant used for trajectory generation
    dt      = 0.02;
    
    % Refreshing rate for simulation update
    rate    = rateControl( 1/(2*dt) );
    
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
    plot_grahps = false;
    
    
    % Manipulator in use for which compute some kinematics (for now just ABB and UR5)
    manipulator = "UR5";
    
    % Use the robot's urdf to show the simulation, if false the simulations uses matlab's plot3
    real_robot  = true;
    
    % Presence of gripper
    gripper     = false;
    
    % Eventual transformation between World Reference Frame and Frame 0
    Trf_0       = eye(4);
    
    
    % Load manipulator's D-H parameters and other useful parameters to compute kinematics
    if manipulator == "ABB"  % ================================================
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
            A = [    0,   0.41,  1.075,  0.165,      0,      0,   -0  ];
            D = [  -0 ,   0.78,      0,      0,  2.012,      0,   0.25];
           TH = [  -0 ,      0,      0,      0,      0,      0,      0];
        %  th = |  -  |  th1  |  th2  |  th3  |  th4  |  th5  |  th6  |
        
        if level == 0 && real_robot
            robot  = importrobot( "robot_urdf/ABB_IRb-7600.urdf" );
            config = robot.homeConfiguration;
        end
    
    elseif manipulator == "UR5"  % ============================================
        % UR5 D-H parameters
        %   T :         0->1      1->2      2->3      3->4      4->5      5->6     6->ee
        %   i = |   0    |    1    |    2    |    3    |    4    |    5    |    6   |   ee   |
           AL = [       0,     pi/2,        0,        0,     pi/2,    -pi/2,       0,   -0   ];
            A = [       0,        0,   -0.425, -0.39225,        0,        0,       0,   -0   ];
            D = [   -0   , 0.089159,        0,        0,  0.10915,  0.09465,  0.0823,  0.1475];
           TH = [   -0   ,        0,        0,        0,        0,        0,       0,       0];
        %  th = |   -    |    th1  |   th2   |   th3   |   th4   |   th5   |   th6  |   -    |
        
        if level == 0 && real_robot
            robot  = importrobot( "robot_urdf/ur5.urdf" );
            config = robot.homeConfiguration;
        end
        gripper = true;
    
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

    % Export variables to workspace
    varnames = who;
    for i = 1:length(varnames)
        assignin( 'caller', varnames{i}, eval(varnames{i}) );
    end

end

