function [tform_cam_to_world] = calibrate_camera_checkerboard(rgbImagePath, depthImagePath, checkerboardParams, worldParams, showPlots)
% CALIBRATE_CAMERA_CHECKERBOARD Camera calibration using a checkerboard pattern
%
% This function implements camera calibration using a standard checkerboard pattern
% placed at a known location. This is the simplest and most reliable method for
% RGB-D camera calibration.
%
% Inputs:
%   rgbImagePath - Path to the RGB image file
%   depthImagePath - Path to the depth image file
%   checkerboardParams - Structure with checkerboard parameters:
%       .squareSize - Size of each square in meters (default: 0.025)
%       .patternSize - [rows, cols] number of inner corners (default: [6, 9])
%       .worldLocation - [x, y, z] location of the checkerboard origin in world coordinates
%       .worldOrientation - Orientation of the checkerboard in world frame (default: eye(3))
%   worldParams - Structure with world parameters:
%       .tableHeight - Height of the table in meters
%   showPlots - Boolean to control visualization (default: false)
%
% Outputs:
%   tform_cam_to_world - Rigid transformation from camera to world frame

% Handle optional arguments
if nargin < 5
    showPlots = false;
end

% Default checkerboard parameters
if ~isfield(checkerboardParams, 'squareSize')
    checkerboardParams.squareSize = 0.025; % 2.5cm squares
end

if ~isfield(checkerboardParams, 'patternSize')
    checkerboardParams.patternSize = [6, 9]; % 6x9 inner corners
end

if ~isfield(checkerboardParams, 'worldOrientation')
    checkerboardParams.worldOrientation = eye(3); % Identity rotation
end

% Extract world parameters
tableHeight = worldParams.tableHeight;

% Load RGB image
fprintf('Loading RGB image...\n');
rgbImage = imread(rgbImagePath);

% Detect checkerboard
fprintf('Detecting checkerboard pattern...\n');
[imagePoints, boardSize] = detectCheckerboardPoints(rgbImage);

% Check if checkerboard was detected
if isempty(imagePoints)
    error('No checkerboard detected in the image.');
end

fprintf('Detected checkerboard with %d inner corners\n', size(imagePoints, 1));

% Verify detected pattern size matches expected size
if boardSize(1) ~= checkerboardParams.patternSize(1) || boardSize(2) ~= checkerboardParams.patternSize(2)
    warning('Detected pattern size [%d, %d] does not match expected size [%d, %d]', ...
        boardSize(1), boardSize(2), checkerboardParams.patternSize(1), checkerboardParams.patternSize(2));
end

% Generate world coordinates of checkerboard corners
squareSize = checkerboardParams.squareSize;
[worldPoints] = generateCheckerboardPoints(boardSize, squareSize);

% Get camera intrinsic parameters
cameraParams = getCameraIntrinsics(rgbImage);

% Estimate the pose of the checkerboard
[R_cam_to_board, t_cam_to_board] = estimateCheckerboardPose(imagePoints, worldPoints, cameraParams);

fprintf('Checkerboard pose in camera coordinates estimated\n');

% Get the checkerboard's location and orientation in world coordinates
R_board_to_world = checkerboardParams.worldOrientation;
t_board_to_world = checkerboardParams.worldLocation';

% Compute camera-to-world transformation
% T_cam_to_world = T_board_to_world * inv(T_cam_to_board)
R_cam_to_world = R_board_to_world * R_cam_to_board';
t_cam_to_world = t_board_to_world - R_board_to_world * R_cam_to_board' * t_cam_to_board;

% Create the final transformation matrix
tform_cam_to_world = rigidtform3d(R_cam_to_world, t_cam_to_world');

fprintf('Camera-to-world transformation computed:\n');
disp(tform_cam_to_world.A);

% Visualize results
if showPlots
    % Load depth image and create point cloud
    depthImage = imread(depthImagePath);
    ptCloud = depthToPointCloud(depthImage, cameraParams);
    
    % Transform point cloud to world coordinates
    ptCloud_world = pctransform(ptCloud, tform_cam_to_world);
    
    % Create figure
    figure('Name', 'Checkerboard Calibration', 'Position', [100, 100, 1200, 800]);
    
    % Plot the original RGB image with detected checkerboard
    subplot(2, 2, 1);
    imshow(rgbImage);
    hold on;
    plot(imagePoints(:,1), imagePoints(:,2), 'ro', 'MarkerSize', 10);
    title('Detected Checkerboard Corners');
    hold off;
    
    % Plot the point cloud in camera coordinates
    subplot(2, 2, 2);
    pcshow(ptCloud);
    title('Point Cloud (Camera Coordinates)');
    
    % Plot the point cloud in world coordinates
    subplot(2, 2, 3);
    pcshow(ptCloud_world);
    hold on;
    
    % Draw world coordinate axes
    axisLength = 0.2;
    plot3([0, axisLength], [0, 0], [0, 0], 'r-', 'LineWidth', 3); % X-axis
    plot3([0, 0], [0, axisLength], [0, 0], 'g-', 'LineWidth', 3); % Y-axis
    plot3([0, 0], [0, 0], [axisLength], 'b-', 'LineWidth', 3); % Z-axis
    
    % Draw checkerboard position in world coordinates
    plot3(checkerboardParams.worldLocation(1), checkerboardParams.worldLocation(2), ...
          checkerboardParams.worldLocation(3), 'ro', 'MarkerSize', 10, 'MarkerFaceColor', 'r');
    
    title('Point Cloud (World Coordinates)');
    hold off;
    
    % Plot the camera position in world coordinates
    subplot(2, 2, 4);
    
    % Create a simulated table surface
    [X, Y] = meshgrid(-0.5:0.05:0.5, -0.5:0.05:0.5);
    Z = ones(size(X)) * tableHeight;
    surf(X, Y, Z, 'FaceColor', [0.8 0.7 0.6], 'FaceAlpha', 0.3, 'EdgeColor', 'none');
    hold on;
    
    % Plot the camera position and orientation
    plotCamera('AbsolutePose', tform_cam_to_world, 'Size', 0.1, 'Color', 'b', 'Opacity', 0.7);
    
    % Plot world coordinate axes
    plot3([0, axisLength], [0, 0], [0, 0], 'r-', 'LineWidth', 3); % X-axis
    plot3([0, 0], [0, axisLength], [0, 0], 'g-', 'LineWidth', 3); % Y-axis
    plot3([0, 0], [0, 0], [axisLength], 'b-', 'LineWidth', 3); % Z-axis
    
    % Plot checkerboard position
    plot3(checkerboardParams.worldLocation(1), checkerboardParams.worldLocation(2), ...
          checkerboardParams.worldLocation(3), 'ro', 'MarkerSize', 10, 'MarkerFaceColor', 'r');
    
    title('Camera Position in World Coordinates');
    xlabel('X (m)'); ylabel('Y (m)'); zlabel('Z (m)');
    axis equal;
    grid on;
    view(30, 30);
end

end

%% Helper Functions

function [worldPoints] = generateCheckerboardPoints(boardSize, squareSize)
    % Generate 3D coordinates of checkerboard corners in board's local coordinate system
    
    % Create grid of points
    [X, Y] = meshgrid(0:boardSize(2)-1, 0:boardSize(1)-1);
    
    % Scale by square size
    X = X * squareSize;
    Y = Y * squareSize;
    
    % Combine into 3D points (Z=0 for planar board)
    worldPoints = [X(:), Y(:), zeros(numel(X), 1)];
end

function cameraParams = getCameraIntrinsics(image)
    % Get camera intrinsic parameters
    % Either load from calibration file or use default values
    
    % Get image size
    [height, width, ~] = size(image);
    
    % Default values for a typical RGB-D camera
    fx = 525.0; % focal length x
    fy = 525.0; % focal length y
    cx = width/2; % principal point x
    cy = height/2; % principal point y
    
    % Create camera parameters object
    intrinsicMatrix = [fx, 0, cx; 0, fy, cy; 0, 0, 1];
    
    % Create camera parameters structure
    cameraParams = struct();
    cameraParams.IntrinsicMatrix = intrinsicMatrix;
    cameraParams.ImageSize = [height, width];
end

function [R, t] = estimateCheckerboardPose(imagePoints, worldPoints, cameraParams)
    % Estimate pose of the checkerboard
    
    % Extract camera matrix
    cameraMatrix = cameraParams.IntrinsicMatrix;
    
    % Assume no distortion for simplicity
    distCoeffs = zeros(5, 1);
    
    % Use PnP to estimate pose
    [success, rotationVector, translationVector] = cv.solvePnP(worldPoints, imagePoints, ...
                                                              cameraMatrix, distCoeffs);
    
    if ~success
        error('Failed to estimate checkerboard pose.');
    end
    
    % Convert rotation vector to rotation matrix
    R = cv.Rodrigues(rotationVector);
    t = translationVector;
end

function ptCloud = depthToPointCloud(depthImage, cameraParams)
    % Convert depth image to point cloud
    
    % Get image dimensions
    [height, width] = size(depthImage);
    
    % Create pixel grid
    [X, Y] = meshgrid(1:width, 1:height);
    
    % Extract camera parameters
    fx = cameraParams.IntrinsicMatrix(1,1);
    fy = cameraParams.IntrinsicMatrix(2,2);
    cx = cameraParams.IntrinsicMatrix(1,3);
    cy = cameraParams.IntrinsicMatrix(2,3);
    
    % Convert depth to meters (adjust scale factor based on your depth image format)
    depthScale = 0.001; % for depth in millimeters
    Z = double(depthImage) * depthScale;
    
    % Convert image coordinates to 3D points
    X3D = (X - cx) .* Z / fx;
    Y3D = (Y - cy) .* Z / fy;
    Z3D = Z;
    
    % Create point cloud
    points = [X3D(:), Y3D(:), Z3D(:)];
    
    % Remove invalid points (where depth is 0 or NaN)
    validIdx = ~isnan(points(:,3)) & points(:,3) > 0;
    points = points(validIdx, :);
    
    % Create point cloud object
    ptCloud = pointCloud(points);
end