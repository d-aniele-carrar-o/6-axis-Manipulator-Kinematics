function [tform_cam_to_world, ptCloudObject_world, ptCloudRemaining_world] = calibrate_camera(scenePcdPath, objectPcdPath, tableParams, showPlots)
% CALIBRATE_CAMERA Camera calibration and object integration using table edge
%
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
planeMaxDistance = 0.004;
planeReferenceVector = [0, 0, -1]; % Adjust based on your camera's Z-axis direction
maxAngularDistance = 5;

% --- RANSAC Line Fitting Parameters ---
lineMaxDistance = 0.003; % Max distance from a point to the line (0.5 cm)
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
% 'boundary' is better than 'convhull' for partial views.
% We use a shrink factor to get a tight boundary.
% Convert to double and remove any NaN values
validPoints = projectedTablePoints3D_cam;
validPoints = double(validPoints);
validIdx = ~any(isnan(validPoints), 2);
validPoints = validPoints(validIdx, :);

% Now call boundary with clean data
k = boundary(validPoints(:,1), validPoints(:,2), 0.9);
boundaryPoints3D = validPoints(k,:);

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

% Extract the direction vector of the line, which is our X-axis
X_cam = normalize(lineModel(4:6), 'norm');

% Ensure the X-axis is perfectly perpendicular to the Z-axis
X_cam = normalize(X_cam - dot(X_cam, Z_cam) * Z_cam, 'norm');

% Ensure a consistent direction for the X-axis (e.g., pointing generally "right" in the camera view)
% This avoids the world flipping 180 degrees between runs.
if X_cam(1) < 0
    X_cam = -X_cam;
end

fprintf('   - Edge line found with direction vector: [%.3f, %.3f, %.3f]\n', X_cam);

if showPlots
    figure;
    pcshow(boundaryPoints3D, 'MarkerSize', 50);
    hold on;
    pcshow(edgeInlierPoints, 'r', 'MarkerSize', 80);
    % Plot the fitted line
    linePts = [mean(edgeInlierPoints) - X_cam*0.5; mean(edgeInlierPoints) + X_cam*0.5];
    plot3(linePts(:,1), linePts(:,2), linePts(:,3), 'g-', 'LineWidth', 3);
    title('RANSAC Line Fit to Boundary Points');
    legend('Boundary Points', 'Line Inliers', 'Fitted Edge Line');
    hold off;
end

%% =======================================================================
%  COMPUTE THE RIGID TRANSFORMATION FROM AXES
%  =======================================================================
fprintf('Computing the camera-to-world transformation from axes...\n');

% We have Z_cam (from plane) and X_cam (from edge line).
% We can now compute Y_cam to form a right-handed coordinate system.
Y_cam = cross(Z_cam, X_cam);

% The rotation matrix is what transforms the [X_cam, Y_cam, Z_cam] basis
% to the world basis [1,0,0], [0,1,0], [0,0,1].
R_cam_to_world = [X_cam; Y_cam; Z_cam]; % Note: This forms the matrix that rotates world axes TO camera axes.
R_cam_to_world = R_cam_to_world';      % The transpose rotates camera axes TO world axes.

fprintf('   - Table height from ground: %.2f m\n', tableHeight);

% Calculate the Y-offset for the world origin
if ischar(worldOriginYOffset) && strcmpi(worldOriginYOffset, 'auto')
    % Automatically determine table width and place origin at the middle
    % Project all table points onto the XY plane of our new coordinate system
    tablePoints_projected = (R_cam_to_world * (tablePoints3D_cam - planeOrigin_cam)')';
    
    % Find the extent of the table in the Y direction
    minY = min(tablePoints_projected(:, 2));
    maxY = max(tablePoints_projected(:, 2));
    detectedTableWidth = maxY - minY;
    
    % Simply negate the Y-offset to move into the table instead of away from it
    yOffset = -detectedTableWidth / 2;
    fprintf('   - Auto-detected table width: %.3f m\n', detectedTableWidth);
    fprintf('   - Setting Y-offset to table middle: %.3f m\n', yOffset);
elseif isnumeric(worldOriginYOffset)
    % Use the user-specified Y-offset
    yOffset = -worldOriginYOffset;  % Negate to move into the table
    fprintf('   - Using user-specified Y-offset: %.3f m\n', yOffset);
else
    % Default to edge (no offset)
    yOffset = 0;
    fprintf('   - Using default Y-offset (table edge): %.3f m\n', yOffset);
end

% Apply the Y-offset to create the world origin
% The origin is at the center of the table (X=0, Y=offset) but at ground level (Z=0)
% Table is at Z=tableHeight in world frame
P_cam_origin = mean(edgeInlierPoints, 1);
P_world_origin = [0, yOffset, tableHeight]; % Origin at ground level (Z=0)

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
    plot3([0 0], [yOffset yOffset], [0 tableHeight], 'k--', 'LineWidth', 2); hold on;
    text(0.02, yOffset, tableHeight/2, sprintf('Table Height: %.2f m', tableHeight), 'Color', 'k'); hold on;
    
    % Final plot adjustments
    title('Final Calibrated Scene (Edge-Based)');
    xlabel('X (m)'); ylabel('Y (m)'); zlabel('Z (m)');
    axis equal; grid on; view(30, 25);
    legend('Table Surface', 'Table Points', 'Remaining Scene', 'Transformed Object', 'Location', 'northeast');
end

fprintf('--- Calibration Complete ---\n');
end

% Note: fitLine3d and distPointToLine3d are external functions
