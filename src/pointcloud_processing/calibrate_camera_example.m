%% Camera Calibration Example Script
% This script demonstrates how to use the camera calibration functions
% with a physical calibration frame.

clear; clc; close all;

%% Setup parameters

% Table parameters
tableParams = struct();
tableParams.height = 0.72;  % Table height in meters
tableParams.width = 1.2;    % Table width in meters
tableParams.length = 1.8;   % Table length in meters

% Single frame calibration parameters
frameParams = struct();
frameParams.axisLength = 0.08;  % 8cm axes
frameParams.location = [0, 0, tableParams.height];  % Frame at center of table

% Dual frame calibration parameters
dualFrameParams = struct();
dualFrameParams.axisLength = 0.08;  % 8cm axes
dualFrameParams.frame1Location = [-0.3, -0.3, tableParams.height];  % First frame location
dualFrameParams.frame2Location = [0.3, 0.3, tableParams.height];    % Second frame location
dualFrameParams.frame1Color = [1, 0, 0];  % Red for first frame
dualFrameParams.frame2Color = [0, 0, 1];  % Blue for second frame

% Paths to image files
% Replace these with your actual file paths
rgbImagePath = 'path/to/rgb_image.png';
depthImagePath = 'path/to/depth_image.png';

%% Method 1: Single Frame Calibration
% Uncomment to run this method

% fprintf('Running single frame calibration...\n');
% tform_cam_to_world = calibrate_camera_frame(rgbImagePath, depthImagePath, frameParams, tableParams, true);
% 
% % Save the calibration result
% save('camera_calibration_single_frame.mat', 'tform_cam_to_world');
% 
% fprintf('Single frame calibration complete. Results saved to camera_calibration_single_frame.mat\n');

%% Method 2: Dual Frame Calibration
% Uncomment to run this method

% fprintf('Running dual frame calibration...\n');
% tform_cam_to_world = calibrate_camera_dual_frames(rgbImagePath, depthImagePath, dualFrameParams, tableParams, true);
% 
% % Save the calibration result
% save('camera_calibration_dual_frame.mat', 'tform_cam_to_world');
% 
% fprintf('Dual frame calibration complete. Results saved to camera_calibration_dual_frame.mat\n');

%% Method 3: Table Edge-Based Calibration (Using existing function)
% This method uses the existing calibration function that detects table edges

% fprintf('Running table edge-based calibration...\n');
% 
% % Add ROI parameter for table detection
% tableParams.roi = [0.6, 0.6, 0.1];  % Region of interest for table plane detection
% 
% % Path to point cloud files
% % Replace these with your actual file paths
% scenePcdPath = 'path/to/scene.ply';
% objectPcdPath = 'path/to/object.ply';
% 
% [tform_cam_to_world, ptCloudObject_world, ptCloudRemaining_world] = ...
%     calibrate_camera(scenePcdPath, objectPcdPath, tableParams, true);
% 
% % Save the calibration result
% save('camera_calibration_table_edge.mat', 'tform_cam_to_world');
% 
% fprintf('Table edge-based calibration complete. Results saved to camera_calibration_table_edge.mat\n');

%% Using the calibration results

% Load a saved calibration
% load('camera_calibration_single_frame.mat');

% Example: Transform a point from camera coordinates to world coordinates
% point_cam = [0.1, 0.2, 0.5];  % Point in camera coordinates
% point_world = transformPointsForward(tform_cam_to_world, point_cam);
% fprintf('Point in camera coordinates: [%.3f, %.3f, %.3f]\n', point_cam);
% fprintf('Point in world coordinates: [%.3f, %.3f, %.3f]\n', point_world);

% Example: Transform a point cloud from camera coordinates to world coordinates
% ptCloud_cam = pointCloud(...);  % Your point cloud in camera coordinates
% ptCloud_world = pctransform(ptCloud_cam, tform_cam_to_world);

%% Notes on physical calibration frame construction
% 
% To create a physical calibration frame:
% 
% 1. 3D print a frame with three perpendicular axes, each 8cm long
% 2. Color each axis differently (e.g., red for X, green for Y, blue for Z)
% 3. Place the frame at a known location on the table
% 4. For dual frame calibration, create two identical frames with different colors
% 
% The calibration accuracy depends on:
% - The precision of the physical frame
% - The accuracy of the depth camera
% - The visibility of the frame in the camera view
% - The accuracy of the known frame location(s)
%
% For best results:
% - Ensure the frame is clearly visible in the camera view
% - Use bright, distinct colors for the axes
% - Place the frame(s) on a flat, stable surface
% - Measure the frame location(s) as accurately as possible
% - Ensure good lighting conditions for color detection