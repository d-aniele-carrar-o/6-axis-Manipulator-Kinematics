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

% Add subfolders
addpath(fullfile(project_root, 'src', 'utils'));
addpath(fullfile(project_root, 'src', 'plot_utils'));
addpath(fullfile(project_root, 'src', '3_link_manipulators'));
addpath(fullfile(project_root, 'src', 'trajectory_scripts'));
addpath(fullfile(project_root, 'src', 'kinematics'));
addpath(fullfile(project_root, 'src', 'pointcloud_processing'));
addpath(fullfile(project_root, 'src', 'robot_descriptions'));
addpath(fullfile(project_root, 'src', 'robot_urdf'));
addpath(fullfile(project_root, 'src', 'cpp_src'));
addpath(fullfile(project_root, 'src', 'mex_compiled_functions'));

% Change to src directory
cd(fullfile(project_root, 'src'));

clear; clc; close all;

% Run the testing script
% compile_cpp_code
% testing_script;
% dual_robot_setup;
simulate_recorded_motion;
