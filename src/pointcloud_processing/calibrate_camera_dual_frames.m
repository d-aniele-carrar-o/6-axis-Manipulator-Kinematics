function [tform_cam_to_world] = calibrate_camera_dual_frames(rgbImagePath, depthImagePath, frameParams, tableParams, showPlots)
% CALIBRATE_CAMERA_DUAL_FRAMES Camera calibration using two physical calibration frames
%
% This function implements camera calibration using two physical calibration frames
% with known dimensions and locations. Each frame has 3 axes (X, Y, Z) of 8cm length each.
% Using two frames at different known locations provides more robust calibration.
%
% Inputs:
%   rgbImagePath - Path to the RGB image file
%   depthImagePath - Path to the depth image file
%   frameParams - Structure with frame parameters:
%       .axisLength - Length of each axis in meters (default: 0.08)
%       .frame1Location - [x, y, z] location of the first frame in world coordinates
%       .frame2Location - [x, y, z] location of the second frame in world coordinates
%       .frame1Color - [r, g, b] color of the first frame (default: [1, 0, 0])
%       .frame2Color - [r, g, b] color of the second frame (default: [0, 0, 1])
%   tableParams - Structure with table parameters (height, width, length)
%   showPlots - Boolean to control visualization (default: false)
%
% Outputs:
%   tform_cam_to_world - Rigid transformation from camera to world frame

% Handle optional arguments
if nargin < 5
    showPlots = false;
end

% Default frame parameters
if ~isfield(frameParams, 'axisLength')
    frameParams.axisLength = 0.08; % 8cm axes
end

if ~isfield(frameParams, 'frame1Color')
    frameParams.frame1Color = [1, 0, 0]; % Red for first frame
end

if ~isfield(frameParams, 'frame2Color')
    frameParams.frame2Color = [0, 0, 1]; % Blue for second frame
end

% Extract table parameters
tableHeight = tableParams.height;

% Load RGB and depth images
fprintf('Loading RGB and depth images...\n');
rgbImage = imread(rgbImagePath);
depthImage = imread(depthImagePath);

% Convert depth image to point cloud
fprintf('Converting depth image to point cloud...\n');
[height, width] = size(depthImage);
[X, Y] = meshgrid(1:width, 1:height);

% Camera intrinsic parameters (these should be calibrated for your specific camera)
% Default values for a typical depth camera - replace with your camera's parameters
fx = 525.0; % focal length x
fy = 525.0; % focal length y
cx = width/2; % principal point x
cy = height/2; % principal point y

% Convert depth to meters (adjust scale factor based on your depth image format)
depthScale = 0.001; % for depth in millimeters
Z = double(depthImage) * depthScale;

% Convert image coordinates to 3D points
X3D = (X - cx) .* Z / fx;
Y3D = (Y - cy) .* Z / fy;
Z3D = Z;

% Create point cloud
points = [X3D(:), Y3D(:), Z3D(:)];
colors = reshape(rgbImage, [], 3);

% Remove invalid points (where depth is 0 or NaN)
validIdx = ~isnan(points(:,3)) & points(:,3) > 0;
points = points(validIdx, :);
colors = colors(validIdx, :);

ptCloud = pointCloud(points, 'Color', colors);

%% Detect both calibration frames
fprintf('Detecting calibration frames...\n');

% Normalize colors
normalizedColors = double(colors) / 255;

% Segment the point cloud by color to find the two frames
% Frame 1 (e.g., red)
frame1ColorThresh = 0.2; % Threshold for color similarity
frame1ColorDiff = sum(abs(normalizedColors - repmat(frameParams.frame1Color, size(normalizedColors, 1), 1)), 2);
frame1Points = points(frame1ColorDiff < frame1ColorThresh, :);

% Frame 2 (e.g., blue)
frame2ColorThresh = 0.2; % Threshold for color similarity
frame2ColorDiff = sum(abs(normalizedColors - repmat(frameParams.frame2Color, size(normalizedColors, 1), 1)), 2);
frame2Points = points(frame2ColorDiff < frame2ColorThresh, :);

fprintf('Found %d points for frame 1 and %d points for frame 2\n', size(frame1Points, 1), size(frame2Points, 1));

% Check if we have enough points for each frame
if size(frame1Points, 1) < 10 || size(frame2Points, 1) < 10
    error('Not enough points detected for one or both frames. Please check frame visibility and colors.');
end

%% Process each frame to extract axes and origins
% We'll use RANSAC to find the three orthogonal axes in each frame

% Function to extract axes from a frame's point cloud
function [frameOrigin, X_axis, Y_axis, Z_axis] = extractFrameAxes(framePoints, axisLength)
    % Cluster the points to find the three axes
    % This is a simplified approach - in practice, you might need more sophisticated clustering
    
    % Use DBSCAN clustering to separate the axes
    epsilon = 0.01; % clustering distance threshold
    minPts = 10;    % minimum points to form a cluster
    
    % Perform DBSCAN clustering
    [clusterIndices, ~] = dbscan(framePoints, epsilon, minPts);
    
    % Get unique cluster labels (excluding noise points labeled as -1)
    uniqueClusters = unique(clusterIndices);
    uniqueClusters = uniqueClusters(uniqueClusters ~= -1);
    
    % We expect to find 3 clusters (one for each axis)
    if length(uniqueClusters) < 3
        warning('Found fewer than 3 clusters in frame. Calibration may not be accurate.');
    end
    
    % Extract points for each cluster
    axisClusters = cell(1, min(3, length(uniqueClusters)));
    for i = 1:min(3, length(uniqueClusters))
        axisClusters{i} = framePoints(clusterIndices == uniqueClusters(i), :);
    end
    
    % Fit lines to each cluster to get axis directions
    axisDirections = cell(1, length(axisClusters));
    axisPoints = cell(1, length(axisClusters));
    
    for i = 1:length(axisClusters)
        % Use PCA to find the principal direction of each cluster
        [coeff, ~, ~] = pca(axisClusters{i});
        axisDirections{i} = coeff(:, 1)'; % First principal component
        
        % Find the point in the cluster farthest from the centroid (should be the tip)
        centroid = mean(axisClusters{i}, 1);
        distances = sqrt(sum((axisClusters{i} - centroid).^2, 2));
        [~, maxIdx] = max(distances);
        axisPoints{i} = axisClusters{i}(maxIdx, :);
    end
    
    % Estimate the origin as the point where the axes would intersect
    % For simplicity, we'll use the centroids of the clusters and trace back along the axis directions
    
    % For each axis, find the point that's axisLength distance from the tip along the axis direction
    originCandidates = zeros(length(axisDirections), 3);
    for i = 1:length(axisDirections)
        originCandidates(i, :) = axisPoints{i} - axisLength * axisDirections{i};
    end
    
    % The frame origin is the average of these candidates
    frameOrigin = mean(originCandidates, 1);
    
    % Now determine which axis is which (X, Y, Z)
    % This is a simplified approach - in practice, you might use color information
    % or other cues to identify the axes
    
    % For now, we'll just assign them based on the order found
    if length(axisDirections) >= 3
        X_axis = axisDirections{1};
        Y_axis = axisDirections{2};
        Z_axis = axisDirections{3};
    elseif length(axisDirections) == 2
        X_axis = axisDirections{1};
        Y_axis = axisDirections{2};
        % Compute Z as perpendicular to X and Y
        Z_axis = cross(X_axis, Y_axis);
        Z_axis = Z_axis / norm(Z_axis);
    else
        X_axis = axisDirections{1};
        % We need to make up two more axes
        % Find a vector perpendicular to X
        if abs(X_axis(1)) < abs(X_axis(2))
            Y_axis = cross(X_axis, [1, 0, 0]);
        else
            Y_axis = cross(X_axis, [0, 1, 0]);
        end
        Y_axis = Y_axis / norm(Y_axis);
        Z_axis = cross(X_axis, Y_axis);
        Z_axis = Z_axis / norm(Z_axis);
    end
    
    % Ensure the axes form a right-handed coordinate system
    if dot(cross(X_axis, Y_axis), Z_axis) < 0
        Z_axis = -Z_axis;
    end
end

% Extract axes from both frames
[frame1Origin, X1, Y1, Z1] = extractFrameAxes(frame1Points, frameParams.axisLength);
[frame2Origin, X2, Y2, Z2] = extractFrameAxes(frame2Points, frameParams.axisLength);

fprintf('Frame 1 origin in camera coordinates: [%.3f, %.3f, %.3f]\n', frame1Origin);
fprintf('Frame 2 origin in camera coordinates: [%.3f, %.3f, %.3f]\n', frame2Origin);

%% Create orthonormal bases for both frames
% Ensure the axes are orthogonal to each other using Gram-Schmidt

% Frame 1
X1 = normalize(X1, 'norm');
Y1 = normalize(Y1 - dot(Y1, X1) * X1, 'norm');
Z1 = normalize(Z1 - dot(Z1, X1) * X1 - dot(Z1, Y1) * Y1, 'norm');

% Frame 2
X2 = normalize(X2, 'norm');
Y2 = normalize(Y2 - dot(Y2, X2) * X2, 'norm');
Z2 = normalize(Z2 - dot(Z2, X2) * X2 - dot(Z2, Y2) * Y2, 'norm');

% Create rotation matrices from camera to each frame
R_cam_to_frame1 = [X1', Y1', Z1'];
R_cam_to_frame2 = [X2', Y2', Z2'];

% Verify the rotation matrices are proper (det = 1)
det_R1 = det(R_cam_to_frame1);
det_R2 = det(R_cam_to_frame2);
fprintf('Determinant of rotation matrix 1: %.6f (should be close to 1)\n', det_R1);
fprintf('Determinant of rotation matrix 2: %.6f (should be close to 1)\n', det_R2);

% Ensure they're proper rotation matrices
if abs(det_R1 - 1) > 0.01
    fprintf('Warning: Rotation matrix 1 is not proper. Applying correction.\n');
    [U, ~, V] = svd(R_cam_to_frame1);
    R_cam_to_frame1 = U * V';
end

if abs(det_R2 - 1) > 0.01
    fprintf('Warning: Rotation matrix 2 is not proper. Applying correction.\n');
    [U, ~, V] = svd(R_cam_to_frame2);
    R_cam_to_frame2 = U * V';
end

%% Compute transformation from camera to world using both frames
% Get the frames' locations in world coordinates
if isfield(frameParams, 'frame1Location') && isfield(frameParams, 'frame2Location')
    frame1_location_world = frameParams.frame1Location;
    frame2_location_world = frameParams.frame2Location;
else
    % Default: assume frames are at opposite corners of the table
    frame1_location_world = [-0.3, -0.3, tableHeight];
    frame2_location_world = [0.3, 0.3, tableHeight];
    fprintf('Using default frame locations:\n');
    fprintf('  Frame 1: [%.3f, %.3f, %.3f]\n', frame1_location_world);
    fprintf('  Frame 2: [%.3f, %.3f, %.3f]\n', frame2_location_world);
end

% Assuming both frames have the same orientation in world coordinates
R_frame_to_world = eye(3);

% Compute transformations from each frame to world
T_frame1_to_world = frame1_location_world;
T_frame2_to_world = frame2_location_world;

% Compute camera-to-world transformations using each frame
R_cam_to_world1 = R_frame_to_world * R_cam_to_frame1;
T_cam_to_world1 = frame1_location_world' - R_cam_to_world1 * frame1Origin';

R_cam_to_world2 = R_frame_to_world * R_cam_to_frame2;
T_cam_to_world2 = frame2_location_world' - R_cam_to_world2 * frame2Origin';

% Average the two transformations for a more robust result
% For rotation, we'll use SLERP (spherical linear interpolation)
% For translation, we'll use a simple average

% Convert rotation matrices to quaternions for interpolation
q1 = rotm2quat(R_cam_to_world1);
q2 = rotm2quat(R_cam_to_world2);

% Interpolate quaternions (weight = 0.5 for equal contribution)
q_avg = quatinterp(q1, q2, 0.5);
R_cam_to_world = quat2rotm(q_avg);

% Average translations
T_cam_to_world = (T_cam_to_world1 + T_cam_to_world2) / 2;

% Create the final transformation matrix
tform_cam_to_world = rigidtform3d(R_cam_to_world, T_cam_to_world');

fprintf('Camera-to-world transformation computed (average of two frames):\n');
disp(tform_cam_to_world.A);

%% Visualize results
if showPlots
    figure('Name', 'Dual Frame Calibration', 'Position', [100, 100, 1200, 800]);
    
    % Plot the original point cloud
    pcshow(ptCloud, 'MarkerSize', 20);
    hold on;
    
    % Plot the detected frames
    axisLength = frameParams.axisLength;
    
    % Plot Frame 1 axes
    plot3([frame1Origin(1), frame1Origin(1) + axisLength * X1(1)], ...
          [frame1Origin(2), frame1Origin(2) + axisLength * X1(2)], ...
          [frame1Origin(3), frame1Origin(3) + axisLength * X1(3)], ...
          'r-', 'LineWidth', 3);
    
    plot3([frame1Origin(1), frame1Origin(1) + axisLength * Y1(1)], ...
          [frame1Origin(2), frame1Origin(2) + axisLength * Y1(2)], ...
          [frame1Origin(3), frame1Origin(3) + axisLength * Y1(3)], ...
          'g-', 'LineWidth', 3);
    
    plot3([frame1Origin(1), frame1Origin(1) + axisLength * Z1(1)], ...
          [frame1Origin(2), frame1Origin(2) + axisLength * Z1(2)], ...
          [frame1Origin(3), frame1Origin(3) + axisLength * Z1(3)], ...
          'b-', 'LineWidth', 3);
    
    % Plot Frame 2 axes
    plot3([frame2Origin(1), frame2Origin(1) + axisLength * X2(1)], ...
          [frame2Origin(2), frame2Origin(2) + axisLength * X2(2)], ...
          [frame2Origin(3), frame2Origin(3) + axisLength * X2(3)], ...
          'r--', 'LineWidth', 3);
    
    plot3([frame2Origin(1), frame2Origin(1) + axisLength * Y2(1)], ...
          [frame2Origin(2), frame2Origin(2) + axisLength * Y2(2)], ...
          [frame2Origin(3), frame2Origin(3) + axisLength * Y2(3)], ...
          'g--', 'LineWidth', 3);
    
    plot3([frame2Origin(1), frame2Origin(1) + axisLength * Z2(1)], ...
          [frame2Origin(2), frame2Origin(2) + axisLength * Z2(2)], ...
          [frame2Origin(3), frame2Origin(3) + axisLength * Z2(3)], ...
          'b--', 'LineWidth', 3);
    
    % Plot the frame origins
    plot3(frame1Origin(1), frame1Origin(2), frame1Origin(3), 'ko', 'MarkerSize', 10, 'MarkerFaceColor', 'r');
    plot3(frame2Origin(1), frame2Origin(2), frame2Origin(3), 'ko', 'MarkerSize', 10, 'MarkerFaceColor', 'b');
    
    % Add labels
    title('Dual Frame Calibration');
    xlabel('X (m)'); ylabel('Y (m)'); zlabel('Z (m)');
    legend('Point Cloud', 'Frame 1 X-axis', 'Frame 1 Y-axis', 'Frame 1 Z-axis', ...
           'Frame 2 X-axis', 'Frame 2 Y-axis', 'Frame 2 Z-axis', ...
           'Frame 1 Origin', 'Frame 2 Origin');
    grid on;
    axis equal;
    
    % Create a second figure to show the world coordinate system
    figure('Name', 'World Coordinate System', 'Position', [100, 100, 1200, 800]);
    
    % Create a simulated table surface
    [X, Y] = meshgrid(-0.5:0.05:0.5, -0.5:0.05:0.5);
    Z = ones(size(X)) * tableHeight;
    surf(X, Y, Z, 'FaceColor', [0.8 0.7 0.6], 'FaceAlpha', 0.3, 'EdgeColor', 'none');
    hold on;
    
    % Plot the camera position and orientation in world coordinates
    plotCamera('AbsolutePose', tform_cam_to_world, 'Size', 0.1, 'Color', 'b', 'Opacity', 0.7);
    
    % Plot world coordinate axes
    axisLength = 0.2;
    plot3([0, axisLength], [0, 0], [0, 0], 'r-', 'LineWidth', 3); % X-axis
    plot3([0, 0], [0, axisLength], [0, 0], 'g-', 'LineWidth', 3); % Y-axis
    plot3([0, 0], [0, 0], [0, axisLength], 'b-', 'LineWidth', 3); % Z-axis
    
    % Plot the frame positions in world coordinates
    plot3(frame1_location_world(1), frame1_location_world(2), frame1_location_world(3), ...
          'ko', 'MarkerSize', 10, 'MarkerFaceColor', 'r');
    plot3(frame2_location_world(1), frame2_location_world(2), frame2_location_world(3), ...
          'ko', 'MarkerSize', 10, 'MarkerFaceColor', 'b');
    
    % Add labels
    title('Camera and Frames in World Coordinates');
    xlabel('X (m)'); ylabel('Y (m)'); zlabel('Z (m)');
    legend('Table Surface', 'Camera', 'World X-axis', 'World Y-axis', 'World Z-axis', ...
           'Frame 1 Position', 'Frame 2 Position');
    grid on;
    axis equal;
    view(30, 30);
end

end