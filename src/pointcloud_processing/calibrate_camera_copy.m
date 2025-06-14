function [tform_cam_to_world, ptCloudObject_world, ptCloudRemaining_world] = calibrate_camera_copy(scenePcdPath, objectPcdPath, tableParams, showPlots)
% CALIBRATE_CAMERA Camera calibration and object integration using table edge

% This function implements a scene-based extrinsic camera calibration by
% using a table's plane normal (Z-axis) and a single prominent edge (X-axis).
% It returns the camera-to-world transformation and transformed object point cloud.
%
% Inputs:
%   scenePcdPath - Path to the scene point cloud file
%   objectPcdPath - Path to the object point cloud file
%   tableParams - Structure with table parameters (height, width, length)
%   showPlots - Boolean to control visualization (default: false)
%
% Outputs:
%   tform_cam_to_world - Rigid transformation from camera to world frame
%   ptCloudObject_world - Object point cloud transformed to world frame

% Handle optional arguments
if nargin < 4
    showPlots = false;
end

% Extract table parameters
tableHeight = tableParams.height;
tableWidth = tableParams.width;
tableLength = tableParams.length;

% --- Calibration Parameters ---
planeMaxDistance = 0.03;
planeReferenceVector = [0, 0, -1]; % Adjust based on your camera's Z-axis direction
% planeReferenceVector = [0, -1/2, -sqrt(3)/2];
maxAngularDistance = 5;

% --- RANSAC Line Fitting Parameters ---
lineMaxDistance = 0.01; % Max distance from a point to the line (0.5 cm)
sampleSize = 2;          % Minimum number of points to define a line

% --- World Origin Parameters ---
% Set to 'auto' to automatically place origin at the middle of the detected table
worldOriginYOffset = 'auto';

%% =======================================================================
%  LOAD AND PREPROCESS POINT CLOUD
%  =======================================================================
fprintf('Loading and preprocessing point cloud %s\n', scenePcdPath);
ptCloudScene = pcread(scenePcdPath);
ptCloudObject_cam = pcread(objectPcdPath);
[ptCloudScene, ~] = pcdenoise(ptCloudScene);

% Crop the point cloud in camera X direction (which will be world Y)
% to remove robot bases and other noise at the edges
fprintf('Cropping point cloud in camera X direction...\n');
x_min_crop = -0.5; % -0.5m in camera X direction
x_max_crop = 0.5;  % 0.5m in camera X direction
scene_points = ptCloudScene.Location;
valid_indices = scene_points(:,1) >= x_min_crop & scene_points(:,1) <= x_max_crop;
ptCloudScene = select(ptCloudScene, valid_indices);
fprintf('   - Cropped from X=[%.2f, %.2f] m, %d points remaining\n', ...
    x_min_crop, x_max_crop, sum(valid_indices));

if showPlots
    % Visualize Scene and Object Point Clouds
    figure('Name', 'Scene and Object Visualization', 'Position', [100, 100, 1000, 800]);
    
    % Plot full scene in gray
    pcshow(ptCloudScene.Location, [0.7 0.7 0.7], 'MarkerSize', 15);
    hold on;
    
    % Plot object with larger red points and slight transparency
    object_points = ptCloudObject_cam.Location;
    scatter3(object_points(:,1), object_points(:,2), object_points(:,3), 100, 'r', ...
        'filled', 'MarkerFaceAlpha', 0.7, 'MarkerEdgeColor', 'none');
    
    % Add a subtle glow effect around object points
    scatter3(object_points(:,1), object_points(:,2), object_points(:,3), 150, ...
        'MarkerFaceColor', [1 0.6 0.6], 'MarkerFaceAlpha', 0.3, 'MarkerEdgeColor', 'none');
    
    title('Scene Point Cloud with Highlighted Object');
    xlabel('X (m)'); ylabel('Y (m)'); zlabel('Z (m)');
    axis equal; grid on;
    view(30, 25);
    legend({'Scene Points', 'Object Points', 'Object Glow'}, 'Location', 'northeast');
    hold off;
end

%% =======================================================================
%  DETECT TABLE PLANE
%  =======================================================================
fprintf('Detecting the table plane...\n');
[planeModel, inlierIndices, outlierIndices] = pcfitplane(ptCloudScene, ...
    planeMaxDistance, planeReferenceVector, maxAngularDistance);

if isempty(inlierIndices)
    error('Could not detect a suitable plane.');
end

ptCloudTable_cam = select(ptCloudScene, inlierIndices);
ptCloudRemaining_cam = select(ptCloudScene, outlierIndices);
fprintf('   - Plane found with normal vector: [%.3f, %.3f, %.3f]\n', planeModel.Normal);

% Define the Z-axis in the camera frame
% For a camera looking down at a table, the plane normal typically points up toward the camera
% We want this to be our Z_cam (which will be mapped to world -Z)
Z_cam = normalize(planeModel.Normal, 'norm');
fprintf('   - Using camera Z-axis: [%.3f, %.3f, %.3f]\n', Z_cam);

%% =======================================================================
%  DETECT TABLE EDGE VIA ROBUST LINE FITTING
%  =======================================================================
fprintf('Detecting table edge line using RANSAC...\n');

% Project table points onto the fitted plane to work in 2D
tablePoints3D_cam = ptCloudTable_cam.Location;
planeOrigin_cam = -planeModel.Normal * planeModel.Parameters(4); % A point on the plane
vecsToPoints = tablePoints3D_cam - planeOrigin_cam;
dists = vecsToPoints * Z_cam';
projectedTablePoints3D_cam = tablePoints3D_cam - dists .* Z_cam;

% Find the boundary points of the projected 2D point cloud segment
validPoints = projectedTablePoints3D_cam;
validPoints = double(validPoints);
validIdx = ~any(isnan(validPoints), 2);
validPoints = validPoints(validIdx, :);

% Now call boundary with clean data
k = boundary(validPoints(:,1), validPoints(:,2), 0.9);
allBoundaryPoints = validPoints(k,:);

% Filter boundary points to focus on those aligned with camera X-axis
fprintf('   - Filtering boundary points to find X-aligned edges...\n');

% Find the min and max X values to identify the left and right edges
minX = min(allBoundaryPoints(:,1));
maxX = max(allBoundaryPoints(:,1));
xRange = maxX - minX;

% Select points near the left and right edges (within 20% of the X range)
leftEdgePoints = allBoundaryPoints(allBoundaryPoints(:,1) < minX + 0.2*xRange, :);
rightEdgePoints = allBoundaryPoints(allBoundaryPoints(:,1) > maxX - 0.2*xRange, :);

% Combine the left and right edge points
boundaryPoints3D = [leftEdgePoints; rightEdgePoints];
fprintf('   - Selected %d boundary points from left and right edges\n', size(boundaryPoints3D, 1));

if showPlots
    figure;
    pcshow(ptCloudTable_cam);
    hold on;
    plot3(boundaryPoints3D(:,1), boundaryPoints3D(:,2), boundaryPoints3D(:,3), 'r.');
    title('Table Points with Detected Boundary Points (Red)');
    hold off;
end

% Use RANSAC to find the best straight line among the boundary points
% This is robust to curves and noise.
addpath(fullfile(fileparts(mfilename('fullpath')), '.'));  % Add current directory to path
[lineModel, inlierIdx] = ransac(boundaryPoints3D, @(pts) fitLine3d(pts(:,1),pts(:,2),pts(:,3)), ...
    @(model, pts) distPointToLine3d(pts, model), sampleSize, lineMaxDistance, 'MaxNumTrials', 1000);

edgeInlierPoints = boundaryPoints3D(inlierIdx, :);

% Extract the direction vector of the line, which is our Y-axis
Y_cam = normalize(lineModel(4:6), 'norm');

% Ensure the Y-axis is perfectly perpendicular to the Z-axis
Y_cam = normalize(Y_cam - dot(Y_cam, Z_cam) * Z_cam, 'norm');

% Check if the detected line is more aligned with camera X or Y axis
x_alignment = abs(dot(Y_cam, [1,0,0]));
y_alignment = abs(dot(Y_cam, [0,1,0]));

% If the line is more aligned with camera Y than X, we need to swap
if y_alignment > x_alignment
    fprintf('   - WARNING: Detected line is more aligned with camera Y than X\n');
    fprintf('   - Forcing Y-axis to be along camera X direction\n');
    % Project camera X-axis onto the plane perpendicular to Z_cam
    Y_cam = [1, 0, 0];
    Y_cam = normalize(Y_cam - dot(Y_cam, Z_cam) * Z_cam, 'norm');
end

% Ensure a consistent direction for the Y-axis (positive X in camera frame)
if Y_cam(1) < 0
    Y_cam = -Y_cam;
end

fprintf('   - Final Y-axis direction: [%.3f, %.3f, %.3f]\n', Y_cam);
fprintf('   - Edge line found with direction vector (Y-axis): [%.3f, %.3f, %.3f]\n', Y_cam);

if showPlots
    figure;
    pcshow(boundaryPoints3D, 'MarkerSize', 50);
    hold on;
    pcshow(edgeInlierPoints, 'r', 'MarkerSize', 80);
    % Plot the fitted line
    linePts = [mean(edgeInlierPoints) - Y_cam*0.5; mean(edgeInlierPoints) + Y_cam*0.5];
    plot3(linePts(:,1), linePts(:,2), linePts(:,3), 'g-', 'LineWidth', 3);
    title('RANSAC Line Fit to Boundary Points (Y-axis)');
    legend('Boundary Points', 'Line Inliers', 'Fitted Edge Line (Y-axis)');
    hold off;
end

%% =======================================================================
%  COMPUTE THE RIGID TRANSFORMATION FROM AXES
%  =======================================================================
fprintf('Computing the camera-to-world transformation from axes...\n');

% We have Z_cam (from plane) and Y_cam (from edge line).
% We compute X_cam to form a right-handed coordinate system.
X_cam = cross(Y_cam, Z_cam);

% The rotation matrix is what transforms the [X_cam, Y_cam, Z_cam] basis
% to the world basis [1,0,0], [0,1,0], [0,0,1].
R_cam_to_world = [X_cam; Y_cam; Z_cam]; % Note: This forms the matrix that rotates world axes TO camera axes.
R_cam_to_world = R_cam_to_world';      % The transpose rotates camera axes TO world axes.

fprintf('   - Table height from ground: %.2f m\n', tableHeight);

% Calculate the world origin to place it directly below the camera
% This ensures the camera is centered above the world frame
fprintf('   - Setting world origin to be directly below the camera\n');

% Project the camera position (origin in camera frame) onto the table plane
% Camera position in camera frame is [0,0,0]
camera_proj = planeOrigin_cam - dot(planeOrigin_cam, Z_cam) * Z_cam;

% Use this as our reference point for the world origin
P_cam_origin = camera_proj;

% No X offset needed since we're placing origin below camera
xOffset = 0;
P_world_origin = [xOffset, 0, tableHeight]; % Origin at ground level (Z=0)

% The transformation equation is: P_world = R * P_cam + T
% So, T = P_world_origin' - R * P_cam_origin'
T_cam_to_world = P_world_origin' - R_cam_to_world * P_cam_origin';

% Create the final 4x4 transformation object
tform_cam_to_world = rigidtform3d(R_cam_to_world, T_cam_to_world');

% Extract camera position in world frame
cam_pos_world = -R_cam_to_world * T_cam_to_world;
fprintf('   - Camera position in world frame: [%.3f, %.3f, %.3f]\n', cam_pos_world);

fprintf('   - Transformation successfully computed from axes:\n');
disp(tform_cam_to_world.A);

%% =======================================================================
%  TRANSFORM OBJECTS AND VISUALIZE THE FINAL SCENE
%  =======================================================================
fprintf('Transforming objects to world frame...\n');

% Transform all point clouds to the new world frame
ptCloudObject_world = pctransform(ptCloudObject_cam, tform_cam_to_world);
ptCloudRemaining_world = pctransform(ptCloudRemaining_cam, tform_cam_to_world);
ptCloudTable_world = pctransform(ptCloudTable_cam, tform_cam_to_world);

if showPlots
    % Create the final visualization
    figure('Name', 'Final Calibrated Simulation Scene (Edge-Based)', 'Position', [100, 100, 1000, 800]);
    
    % Create a simulated table surface (tableLength x tableWidth centered at world origin)
    [X, Y] = meshgrid([-tableLength/2:0.05:tableLength/2], [-tableWidth/2:0.05:tableWidth/2]);
    Z = zeros(size(X)) + tableHeight;  % Table is at Z=tableHeight in world frame
    surf(X, Y, Z, 'FaceColor', [0.8 0.7 0.6], 'FaceAlpha', 0.3, 'EdgeColor', 'none'); hold on;
    
    pcshow(ptCloudTable_world.Location, 'y', 'MarkerSize', 20); hold on;
    pcshow(ptCloudRemaining_world.Location, [0.7 0.7 0.7], 'MarkerSize', 20); hold on;
    pcshow(ptCloudObject_world.Location, 'r', 'MarkerSize', 50); hold on;
    
    % Plot the camera's position and orientation in the world frame
    plotCamera('AbsolutePose', tform_cam_to_world, 'Size', 0.1, 'Color', 'b', 'Opacity', 0.2); hold on;
    
    % Add coordinate frame axes for clarity
    plot3([0 0.2], [0 0], [0 0], 'r-', 'LineWidth', 3); text(0.2, 0.01, 0, 'World X', 'Color', 'r'); hold on;
    plot3([0 0], [0 0.2], [0 0], 'g-', 'LineWidth', 3); text(0, 0.21, 0, 'World Y', 'Color', 'g'); hold on;
    plot3([0 0], [0 0], [0 0.2], 'b-', 'LineWidth', 3); text(0, 0.01, 0.2, 'World Z', 'Color', 'b'); hold on;
    
    % Visualize the table height (from ground up to table)
    plot3([xOffset xOffset], [0 0], [0 tableHeight], 'k--', 'LineWidth', 2); hold on;
    text(xOffset, 0.02, tableHeight/2, sprintf('Table Height: %.2f m', tableHeight), 'Color', 'k'); hold on;
    
    % Final plot adjustments
    title('Final Calibrated Scene (Edge-Based)');
    xlabel('X (m)'); ylabel('Y (m)'); zlabel('Z (m)');
    axis equal; grid on; view(30, 25);
    legend('Table Surface', 'Table Points', 'Remaining Scene', 'Transformed Object', 'Location', 'northeast');
end

fprintf('--- Calibration Complete ---\n');
end

% Note: fitLine3d and distPointToLine3d are external functions
