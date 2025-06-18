%% Robot Base Calibration Script
% This script calibrates the transformation between the world reference frame
% and the robot base reference frames using known poses and joint configurations.
clear; clc; close all;

%% Define known parameters
% Height of the robot stands
stand_height = 0.215;  % meters

% Distance between robot stands along Y axis
y_distance = 1.27;  % meters

%% Define calibration poses
% Each pose is defined as [x, y, z, rx, ry, rz] in world frame
% where rx, ry, rz are Euler angles in XYZ convention (radians)

% Example poses for left robot
poses_left = [
    % Pose 1
     0.375, -0.535, 0, 0, 3.14, 0;
    % Pose 2
    -0.375, -0.535, 0, 0, 3.14, 0;
    % Pose 3
       0.0, -0.335, 0, 0, 3.14, 0
];

% Example poses for right robot
poses_right = [
    % Pose 1
     0.375, 0.535, 0, 0, 3.143, 0;
    % Pose 2
    -0.375, 0.535, 0, 0, 3.143, 0;
    % Pose 3
       0.0, 0.335, 0, 0, 3.143, 0
];

%% Define joint configurations for each pose
% Joint configurations corresponding to the poses above

% Example joint configurations for left robot
q_left = [
    % Configuration for pose 1
    deg2rad([2.53, -3.88, 92.82, -179.01, -90.4, 267.44]);
    % Configuration for pose 2
    deg2rad([144.33, -2.5, 79.66, -167.42, -90.36, 54.29]);
    % Configuration for pose 3
    deg2rad([68.63, -2.3, 107.58, -195.41, -90.27, 338.58]);
];

% Example joint configurations for right robot
q_right = [
    % Configuration for pose 1
    deg2rad([2.81, -176.07, -92.87, -1.07, 90.07, 93.21]);
    % Configuration for pose 2
    deg2rad([-143.91, -171.52, -79.66, -12.66, 90.08, 306.47]);
    % Configuration for pose 3
    deg2rad([-68.3, -177.96, -107.86, 15.86, 89.88, 22.12]);
];

%% Compute transformations
[T_world_base_left, T_world_base_right] = compute_robot_base_transforms(poses_left, poses_right, q_left, q_right, stand_height, y_distance);

%% Visualize the results
% Setup robots
% Setup first robot (UR3e left)
parameters(0, 1);
robot_left = robot;
config_left = config;

% Setup second robot (UR3e right)
parameters(0, 2);
robot_right = robot;
config_right = config;

% Create environment
figure('Name', 'Robot Base Calibration', 'Position', [100, 100, 1200, 800]);
hold on; grid on;

tableParams.height = tableHeight;
tableParams.width = tableWidth;
tableParams.length = tableLength;

axs = create_environment(tablePosition, tableParams);

% Update robot base transformations with calibrated values
Trf_0_l = T_world_base_left;
Trf_0_r = T_world_base_right;

% Set initial robot configurations
q0_left = [pi/2, -pi/3, 2*pi/3, -pi/3, pi/2, 0];
q0_right = [-pi/2, -2*pi/3, -2*pi/3, -2*pi/3, -pi/2, 0];

config_left = set_robot_configuration(q0_left, config_left);
config_right = set_robot_configuration(q0_right, config_right);

% Show robots with calibrated base transformations
show(robot_left, config_left, "Visuals", "on", "Frames", "on", "FastUpdate", true, "PreservePlot", false, "Parent", axs);
show(robot_right, config_right, "Visuals", "on", "Frames", "on", "FastUpdate", true, "PreservePlot", false, "Parent", axs);

% Add text labels for each robot
left_base_pos = Trf_0_l(1:3,4);
right_base_pos = Trf_0_r(1:3,4);
text(left_base_pos(1), left_base_pos(2), tableHeight + 0.01, 'Robot Left (Calibrated)', ...
    'Color', 'r', 'FontSize', 12, 'FontWeight', 'bold', 'HorizontalAlignment', 'center', 'Parent', axs);
text(right_base_pos(1), right_base_pos(2), tableHeight + 0.01, 'Robot Right (Calibrated)', ...
    'Color', 'b', 'FontSize', 12, 'FontWeight', 'bold', 'HorizontalAlignment', 'center', 'Parent', axs);

% Plot world coordinate frame
triad('Parent', axs, 'Matrix', eye(4), 'Scale', 0.2, 'LineWidth', 2);
text(0, 0, 0.05, 'World Frame', 'FontSize', 10, 'FontWeight', 'bold', 'Parent', axs);

% Plot robot base coordinate frames
triad('Parent', axs, 'Matrix', T_world_base_left, 'Scale', 0.15, 'LineWidth', 2);
triad('Parent', axs, 'Matrix', T_world_base_right, 'Scale', 0.15, 'LineWidth', 2);

% Set view angle
view(45, 30);
title('Robot Base Calibration Results');

% Save the calibrated transformations
save('calibrated_robot_bases.mat', 'T_world_base_left', 'T_world_base_right');
fprintf('Calibrated transformations saved to calibrated_robot_bases.mat\n');

% Display instructions for using the calibrated transformations
fprintf('\nTo use these calibrated transformations in your code:\n');
fprintf('1. Load the saved transformations: load(''calibrated_robot_bases.mat'')\n');
fprintf('2. Update the robot base transformations in parameters.m or in your code:\n');
fprintf('   robot_base_transforms = {\n');
fprintf('       T_world_base_left,  %% Left robot\n');
fprintf('       T_world_base_right  %% Right robot\n');
fprintf('   };\n');
