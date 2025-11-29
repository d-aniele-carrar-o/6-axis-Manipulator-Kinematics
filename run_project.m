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
data_folder = '/Users/danielecarraro/Documents/VSCODE/data/';
addpath(fullfile(data_folder, 'data'));
addpath(fullfile(data_folder, 'output'));
addpath(fullfile(data_folder, 'output', 'pointcloud'));
addpath(fullfile(data_folder, 'output', 'segmented_objects'));
addpath(fullfile(data_folder, 'output', 'augmented_demos'));
addpath(fullfile(project_root, 'src'));
addpath(fullfile(project_root, 'src', '3_link_manipulators'));
addpath(fullfile(project_root, 'src', 'cpp_src'));
addpath(fullfile(project_root, 'src', 'files_utils'));
addpath(fullfile(project_root, 'src', 'kinematics'));
addpath(fullfile(project_root, 'src', 'mex_compiled_functions'));
addpath(fullfile(project_root, 'src', 'plot_utils'));
addpath(fullfile(project_root, 'src', 'pointcloud_processing'));
addpath(fullfile(project_root, 'src', 'robot_calibration'));
addpath(fullfile(project_root, 'src', 'robot_descriptions'));
addpath(fullfile(project_root, 'src', 'robot_urdf'));
addpath(fullfile(project_root, 'src', 'trajectory_scripts'));
addpath(fullfile(project_root, 'src', 'trajectory_transformation'));
addpath(fullfile(project_root, 'src', 'utils'));
addpath(fullfile(project_root, 'scripts'));
% Change to src directory
cd(fullfile(project_root, 'src'));

% Store the project root path in a global variable for access in other scripts
global PROJECT_ROOT;
PROJECT_ROOT = project_root;

clc; close all;

% Set the default figure properties to change the background color
set(0, 'DefaultFigureColor', 'w'); % Set default figure background to white
set(0, 'DefaultAxesColor', 'w');   % Set default axes background to white
set(0, 'DefaultAxesXColor', 'k');  % Set default x-axis color to black
set(0, 'DefaultAxesYColor', 'k');  % Set default y-axis color to black
set(0, 'DefaultAxesZColor', 'k');  % Set default z-axis color to black

% Run the desired script ------------------------------------------------------
% compile_cpp_code
testing_script;
% dual_robot_setup
% test_augment_trajectory
% simulate_recorded_motion
% simulate_inference
% testing_tfs_3d
% letter_writing;
