function [tform_cam_to_world] = calibrate_camera_frame(rgbImagePath, depthImagePath, frameParams, tableParams, showPlots)
% CALIBRATE_CAMERA_FRAME Camera calibration using a physical calibration frame
%
% This function implements camera calibration using a physical calibration frame
% with known dimensions. The frame has 3 axes (X, Y, Z) of 8cm length each.
% The function detects the frame in the image and calculates the transformation
% from camera to world frame.
%
% Inputs:
%   rgbImagePath - Path to the RGB image file
%   depthImagePath - Path to the depth image file
%   frameParams - Structure with frame parameters:
%       .axisLength - Length of each axis in meters (default: 0.08)
%       .location - [x, y, z] location of the frame in world coordinates
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

%% Detect the calibration frame
fprintf('Detecting calibration frame...\n');

% Method 1: Color-based segmentation to find the frame
% Assuming the frame has distinct colors for each axis (e.g., red, green, blue)

% Extract red, green, blue components of the frame
redThresh = 0.7;
greenThresh = 0.7;
blueThresh = 0.7;

% Normalize colors
normalizedColors = double(colors) / 255;

% Find points for each axis based on color
redPoints = points(normalizedColors(:,1) > redThresh & ...
                  normalizedColors(:,2) < 0.5 & ...
                  normalizedColors(:,3) < 0.5, :);
                
greenPoints = points(normalizedColors(:,1) < 0.5 & ...
                    normalizedColors(:,2) > greenThresh & ...
                    normalizedColors(:,3) < 0.5, :);
                  
bluePoints = points(normalizedColors(:,1) < 0.5 & ...
                   normalizedColors(:,2) < 0.5 & ...
                   normalizedColors(:,3) > blueThresh, :);

% If color-based detection doesn't work well, try Method 2:
% Method 2: Geometric detection of straight lines in the point cloud
if isempty(redPoints) || isempty(greenPoints) || isempty(bluePoints)
    fprintf('Color-based detection failed. Trying geometric detection...\n');
    
    % Downsample point cloud for faster processing
    ptCloudDownsampled = pcdownsample(ptCloud, 'gridAverage', 0.005);
    
    % Find planes in the point cloud (the frame should be on the table)
    maxDistance = 0.01;
    [~, inlierIndices, outlierIndices] = pcfitplane(ptCloudDownsampled, maxDistance);
    
    % Extract non-planar points (potential frame points)
    nonPlanarPoints = select(ptCloudDownsampled, outlierIndices);
    
    % Use RANSAC to find straight lines in the non-planar points
    % This would require implementing a line detection algorithm
    % For simplicity, we'll assume the color-based method works
    fprintf('Geometric detection not implemented. Please ensure the frame has colored axes.\n');
end

%% Process detected axis points
% Function to fit a line to points and get the direction vector
fitAxisLine = @(points) pcfitline(pointCloud(points), 0.01);

% Check if we have enough points for each axis
axisDetected = [~isempty(redPoints), ~isempty(greenPoints), ~isempty(bluePoints)];

if sum(axisDetected) < 2
    error('Could not detect at least two axes of the calibration frame. Please check the frame visibility and colors.');
end

% Fit lines to each detected axis
axisVectors = cell(1, 3);
axisOrigins = cell(1, 3);

% Process X axis (red)
if axisDetected(1)
    [lineModelX, ~, ~] = fitAxisLine(redPoints);
    axisVectors{1} = lineModelX.Direction;
    axisOrigins{1} = lineModelX.Point;
    fprintf('X-axis detected with direction: [%.3f, %.3f, %.3f]\n', axisVectors{1});
end

% Process Y axis (green)
if axisDetected(2)
    [lineModelY, ~, ~] = fitAxisLine(greenPoints);
    axisVectors{2} = lineModelY.Direction;
    axisOrigins{2} = lineModelY.Point;
    fprintf('Y-axis detected with direction: [%.3f, %.3f, %.3f]\n', axisVectors{2});
end

% Process Z axis (blue)
if axisDetected(3)
    [lineModelZ, ~, ~] = fitAxisLine(bluePoints);
    axisVectors{3} = lineModelZ.Direction;
    axisOrigins{3} = lineModelZ.Point;
    fprintf('Z-axis detected with direction: [%.3f, %.3f, %.3f]\n', axisVectors{3});
end

%% Find the origin (intersection of axes)
% The origin is where all axes intersect
% For simplicity, we'll use the average of the closest points between axes

% Find frame origin
frameOrigin = zeros(1, 3);
numDetectedAxes = sum(axisDetected);

if numDetectedAxes == 3
    % If all three axes are detected, find the point that minimizes distance to all three lines
    % This is a more complex optimization problem
    % For simplicity, we'll use the average of pairwise closest points
    
    % Find closest points between X and Y axes
    [pointOnX_XY, pointOnY_XY] = closestPointsBetweenLines(axisOrigins{1}, axisVectors{1}, axisOrigins{2}, axisVectors{2});
    
    % Find closest points between X and Z axes
    [pointOnX_XZ, pointOnZ_XZ] = closestPointsBetweenLines(axisOrigins{1}, axisVectors{1}, axisOrigins{3}, axisVectors{3});
    
    % Find closest points between Y and Z axes
    [pointOnY_YZ, pointOnZ_YZ] = closestPointsBetweenLines(axisOrigins{2}, axisVectors{2}, axisOrigins{3}, axisVectors{3});
    
    % Average all closest points to get the origin
    frameOrigin = mean([pointOnX_XY; pointOnY_XY; pointOnX_XZ; pointOnZ_XZ; pointOnY_YZ; pointOnZ_YZ], 1);
    
elseif numDetectedAxes == 2
    % If only two axes are detected, find the closest points between them
    
    % Find which two axes are detected
    detectedIndices = find(axisDetected);
    
    % Find closest points between the two detected axes
    [pointOnLine1, pointOnLine2] = closestPointsBetweenLines(axisOrigins{detectedIndices(1)}, axisVectors{detectedIndices(1)}, 
                                                           axisOrigins{detectedIndices(2)}, axisVectors{detectedIndices(2)});
    
    % Average the two closest points to get the origin
    frameOrigin = mean([pointOnLine1; pointOnLine2], 1);
else
    % If only one axis is detected, use its origin point
    % This is not ideal for accurate calibration
    detectedIndex = find(axisDetected);
    frameOrigin = axisOrigins{detectedIndex};
    warning('Only one axis detected. Calibration may not be accurate.');
end

fprintf('Frame origin in camera coordinates: [%.3f, %.3f, %.3f]\n', frameOrigin);

%% Create orthonormal basis for the frame
% We need to ensure the axes are orthogonal to each other

% Start with the detected axes
X_cam = axisVectors{1};
Y_cam = axisVectors{2};
Z_cam = axisVectors{3};

% If we're missing an axis, compute it from the other two
if ~axisDetected(1) && axisDetected(2) && axisDetected(3)
    % Missing X, have Y and Z
    X_cam = cross(Y_cam, Z_cam);
    fprintf('X-axis computed from Y and Z: [%.3f, %.3f, %.3f]\n', X_cam);
elseif axisDetected(1) && ~axisDetected(2) && axisDetected(3)
    % Missing Y, have X and Z
    Y_cam = cross(Z_cam, X_cam);
    fprintf('Y-axis computed from Z and X: [%.3f, %.3f, %.3f]\n', Y_cam);
elseif axisDetected(1) && axisDetected(2) && ~axisDetected(3)
    % Missing Z, have X and Y
    Z_cam = cross(X_cam, Y_cam);
    fprintf('Z-axis computed from X and Y: [%.3f, %.3f, %.3f]\n', Z_cam);
end

% Ensure orthogonality using Gram-Schmidt process
X_cam = normalize(X_cam, 'norm');
Y_cam = normalize(Y_cam - dot(Y_cam, X_cam) * X_cam, 'norm');
Z_cam = normalize(Z_cam - dot(Z_cam, X_cam) * X_cam - dot(Z_cam, Y_cam) * Y_cam, 'norm');

% Create rotation matrix from camera to frame
R_cam_to_frame = [X_cam', Y_cam', Z_cam'];

% Verify the rotation matrix is proper (det = 1)
det_R = det(R_cam_to_frame);
fprintf('Determinant of rotation matrix: %.6f (should be close to 1)\n', det_R);

% Ensure it's a proper rotation matrix
if abs(det_R - 1) > 0.01
    fprintf('Warning: Rotation matrix is not proper. Applying correction.\n');
    % Ensure orthogonality using SVD
    [U, ~, V] = svd(R_cam_to_frame);
    R_cam_to_frame = U * V';
end

%% Compute transformation from camera to world
% The frame is placed at a known location on the table
% We need to transform from camera -> frame -> world

% Get the frame's location in world coordinates
if isfield(frameParams, 'location')
    frame_location_world = frameParams.location;
else
    % Default: assume frame is at the center of the table
    frame_location_world = [0, 0, tableHeight];
    fprintf('Using default frame location at table center: [%.3f, %.3f, %.3f]\n', frame_location_world);
end

% Create transformation from frame to world
% This depends on how the frame is placed relative to the world coordinate system
% Assuming the frame axes are aligned with world axes
R_frame_to_world = eye(3);
T_frame_to_world = frame_location_world;

% Compute the full transformation: camera -> frame -> world
R_cam_to_world = R_frame_to_world * R_cam_to_frame;
T_cam_to_world = frame_location_world' - R_cam_to_world * frameOrigin';

% Create the final transformation matrix
tform_cam_to_world = rigidtform3d(R_cam_to_world, T_cam_to_world');

fprintf('Camera-to-world transformation computed:\n');
disp(tform_cam_to_world.A);

%% Visualize results
if showPlots
    figure('Name', 'Calibration Frame Detection', 'Position', [100, 100, 1000, 800]);
    
    % Plot the original point cloud
    pcshow(ptCloud, 'MarkerSize', 20);
    hold on;
    
    % Plot the detected axes
    axisLength = frameParams.axisLength;
    
    % Plot X axis (red)
    if axisDetected(1)
        plot3([frameOrigin(1), frameOrigin(1) + axisLength * X_cam(1)], ...
              [frameOrigin(2), frameOrigin(2) + axisLength * X_cam(2)], ...
              [frameOrigin(3), frameOrigin(3) + axisLength * X_cam(3)], ...
              'r-', 'LineWidth', 3);
    end
    
    % Plot Y axis (green)
    if axisDetected(2)
        plot3([frameOrigin(1), frameOrigin(1) + axisLength * Y_cam(1)], ...
              [frameOrigin(2), frameOrigin(2) + axisLength * Y_cam(2)], ...
              [frameOrigin(3), frameOrigin(3) + axisLength * Y_cam(3)], ...
              'g-', 'LineWidth', 3);
    end
    
    % Plot Z axis (blue)
    if axisDetected(3)
        plot3([frameOrigin(1), frameOrigin(1) + axisLength * Z_cam(1)], ...
              [frameOrigin(2), frameOrigin(2) + axisLength * Z_cam(2)], ...
              [frameOrigin(3), frameOrigin(3) + axisLength * Z_cam(3)], ...
              'b-', 'LineWidth', 3);
    end
    
    % Plot the frame origin
    plot3(frameOrigin(1), frameOrigin(2), frameOrigin(3), 'ko', 'MarkerSize', 10, 'MarkerFaceColor', 'k');
    
    % Add labels
    title('Calibration Frame Detection');
    xlabel('X (m)'); ylabel('Y (m)'); zlabel('Z (m)');
    legend('Point Cloud', 'X-axis', 'Y-axis', 'Z-axis', 'Frame Origin');
    grid on;
    axis equal;
    
    % Create a second figure to show the world coordinate system
    figure('Name', 'World Coordinate System', 'Position', [100, 100, 1000, 800]);
    
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
    
    % Plot the frame position in world coordinates
    plot3(frame_location_world(1), frame_location_world(2), frame_location_world(3), ...
          'ko', 'MarkerSize', 10, 'MarkerFaceColor', 'k');
    
    % Add labels
    title('Camera and Frame in World Coordinates');
    xlabel('X (m)'); ylabel('Y (m)'); zlabel('Z (m)');
    legend('Table Surface', 'Camera', 'World X-axis', 'World Y-axis', 'World Z-axis', 'Frame Position');
    grid on;
    axis equal;
    view(30, 30);
end

end

%% Helper Functions

function [pointOnLine1, pointOnLine2] = closestPointsBetweenLines(origin1, dir1, origin2, dir2)
% Find the closest points between two 3D lines
% Lines are defined by an origin point and a direction vector
% Returns the closest point on each line

    % Normalize direction vectors
    dir1 = normalize(dir1, 'norm');
    dir2 = normalize(dir2, 'norm');
    
    % Calculate parameters for closest points
    % Based on the formula for the closest points between two skew lines
    
    % Vector connecting the origins
    w0 = origin1 - origin2;
    
    % Calculate dot products
    a = dot(dir1, dir1);
    b = dot(dir1, dir2);
    c = dot(dir2, dir2);
    d = dot(dir1, w0);
    e = dot(dir2, w0);
    
    % Calculate parameters
    denom = a*c - b*b;
    
    % If lines are parallel or nearly parallel
    if abs(denom) < 1e-10
        % Just use the origin of the first line
        pointOnLine1 = origin1;
        pointOnLine2 = origin2;
        return;
    end
    
    % Calculate parameters for closest points
    t1 = (b*e - c*d) / denom;
    t2 = (a*e - b*d) / denom;
    
    % Calculate closest points
    pointOnLine1 = origin1 + t1 * dir1;
    pointOnLine2 = origin2 + t2 * dir2;
end