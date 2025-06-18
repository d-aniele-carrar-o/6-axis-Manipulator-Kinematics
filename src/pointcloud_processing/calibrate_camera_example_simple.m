%% Simple Camera Calibration Example
% This script demonstrates the easiest way to calibrate an RGB-D camera
% using a standard checkerboard pattern.

clear; clc; close all;

%% Setup parameters

% Table parameters
worldParams = struct();
worldParams.tableHeight = 0.72;  % Table height in meters

% Checkerboard parameters
checkerboardParams = struct();
checkerboardParams.squareSize = 0.025;  % 2.5cm squares
checkerboardParams.patternSize = [6, 9];  % 6x9 inner corners
checkerboardParams.worldLocation = [0, 0, worldParams.tableHeight];  % Center of checkerboard at table center
checkerboardParams.worldOrientation = eye(3);  % Checkerboard aligned with world frame

% Paths to image files
% Replace these with your actual file paths
rgbImagePath = 'path/to/rgb_image.png';
depthImagePath = 'path/to/depth_image.png';

%% Run calibration
fprintf('Running checkerboard calibration...\n');
tform_cam_to_world = calibrate_camera_checkerboard(rgbImagePath, depthImagePath, checkerboardParams, worldParams, true);

% Save the calibration result
save('camera_calibration.mat', 'tform_cam_to_world');

fprintf('Calibration complete. Results saved to camera_calibration.mat\n');

%% Using the calibration results

% Example: Transform a point from camera coordinates to world coordinates
point_cam = [0.1, 0.2, 0.5];  % Point in camera coordinates
point_world = transformPointsForward(tform_cam_to_world, point_cam);
fprintf('Point in camera coordinates: [%.3f, %.3f, %.3f]\n', point_cam);
fprintf('Point in world coordinates: [%.3f, %.3f, %.3f]\n', point_world);

%% Notes on checkerboard preparation
% 
% To prepare a checkerboard for calibration:
% 
% 1. Print a checkerboard pattern (e.g., 7x10 squares) on a rigid board
% 2. Measure the exact size of each square in meters
% 3. Place the checkerboard at a known location on the table
% 4. Make sure the checkerboard is flat and fully visible in the camera view
% 
% The calibration accuracy depends on:
% - The precision of the printed checkerboard
% - The accuracy of the square size measurement
% - The flatness of the checkerboard
% - The accuracy of the known checkerboard location
%
% For best results:
% - Use a high-quality printed checkerboard on a rigid backing
% - Measure the square size with a caliper for precision
% - Place the checkerboard on a flat, stable surface
% - Ensure good lighting conditions for corner detection