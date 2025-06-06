% Dual robot setup and simulation
clear; clc; close all;

% Create figure for visualization
figure('Name', 'Dual Robot Workspace');
hold on; grid on;

% Setup first robot (UR3e left)
parameters(0, 1);
robot_left  = robot;
config_left = config;

% Setup second robot (UR3e right)
parameters(0, 2);
robot_right  = robot;
config_right = config;

% Create environment for multi-robot setup
create_environment( tablePosition, tableLength, tableWidth, tableHeight );

% Initial joint configurations
q0_left  = [0,  pi/6,  pi/2,  pi/3,  pi/2, 0.0]
q0_right = [0, -pi/6, -pi/2, -pi/3, -pi/2, 0.0]

% Set robot configurations
config_left  = set_robot_configuration( q0_left, config_left );
config_right = set_robot_configuration( q0_right, config_right );

% Visualize robots
axs = show( robot_left, config_left, "Visuals", "on", "Frames", "on", "FastUpdate", true, "PreservePlot", false );
show( robot_right, config_right, "Visuals", "on", "Frames", "on", "FastUpdate", true, "PreservePlot", false, "Parent", axs );

% Compute end-effector poses
parameters(1, 1); % Load parameters for robot 1
[Te_w_e_left,  Te_left]  = direct_kinematics( q0_left, 1 );

parameters(1, 2); % Load parameters for robot 2
[Te_w_e_right, Te_right] = direct_kinematics( q0_right, 2 );

% Display end-effector positions
disp('Robot 1 end-effector pose (wrt world frame):');
disp(Te_w_e_left);
disp('Robot 2 end-effector pose (wrt world frame):');
disp(Te_w_e_right);

% Example of planning a simple trajectory for both robots
% This is just a placeholder - you would expand this based on your needs
disp('Planning trajectories for both robots...');
