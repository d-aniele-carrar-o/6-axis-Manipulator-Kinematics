% Portable run script for MATLAB project
clear; clc; close all;

% Find the project root directory (containing the src folder)
current_path = pwd;
[~, current_folder] = fileparts(current_path);

% If we're already in the src directory, go up one level
if strcmp(current_folder, 'src')
    cd('..');
    project_root = pwd;
% If we're in the project root (6-axis-Manipulator-Kinematics)
elseif exist('src', 'dir')
    project_root = pwd;
% Otherwise, try to find the project root
else
    error('Please run this script from the project root or src directory');
end

% Change to src directory
cd(fullfile(project_root, 'src'));

clear; clc; close all;

% Load parameters
parameters(0)

% Run the testing script
% testing_script;
% robot_in_environment;
dual_robot_setup()
