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
%   tableParams - Structure with table parameters (height, width, length, roi)
%   showPlots - Boolean to control visualization (default: false)
%
% Outputs:
%   tform_cam_to_world - Rigid transformation from camera to world frame
%   ptCloudObject_world - Object point cloud transformed to world frame
%   ptCloudRemaining_world - Remaining point cloud transformed to world frame

% Handle optional arguments
if nargin < 4
    showPlots = false;
end

% Extract table parameters
tableHeight = tableParams.height;
tableWidth = tableParams.width;
tableLength = tableParams.length;
tableRoI = tableParams.roi;  % [x, y, z] - Region of Interest for table plane fitting

% --- Calibration Parameters ---
planeMaxDistance = 0.03;
planeReferenceVector = [0, 0, -1]; % Adjust based on your camera's Z-axis direction
maxAngularDistance = 10;

% --- RANSAC Line Fitting Parameters ---
lineMaxDistance = 0.03; % Max distance from a point to the line (0.5 cm)
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

if false
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

% Apply Region of Interest (RoI) filtering to exclude robot bases
fprintf('   - Applying RoI filter [%.2f, %.2f, %.2f] to exclude robot bases\n', tableRoI);
ptCloud_locations = ptCloudScene.Location;
ptCloud_center = mean(ptCloud_locations, 1);  % Use point cloud center as reference

% Filter points within the RoI (in camera coordinates)
% Note: tableRoI is expressed in camera coordinates where:
% - Camera X is aligned with world Y
% - Camera Y is aligned with world X
roi_x_half = tableRoI(1) / 2;  % Camera X dimension
roi_y_half = tableRoI(2) / 2;  % Camera Y dimension
roi_z = tableRoI(3);           % Z distance from table surface

% Find points within the RoI centered at the point cloud center
roi_indices = find(abs(ptCloud_locations(:,1) - ptCloud_center(1)) <= roi_x_half & ...
                  abs(ptCloud_locations(:,2) - ptCloud_center(2)) <= roi_y_half & ...
                  abs(ptCloud_locations(:,3) - ptCloud_center(3)) <= roi_z);

% Create a filtered point cloud for plane fitting
ptCloudRoI = select(ptCloudScene, roi_indices);

if showPlots
    figure;
    pcshow(ptCloudScene.Location, [0.7 0.7 0.7], 'MarkerSize', 15);
    hold on;
    pcshow(ptCloudRoI.Location, [0 1 0], 'MarkerSize', 20);
    title('Point Cloud with RoI Filter Applied');
    legend('Full Scene', 'RoI for Plane Fitting');
    hold off;
end

% Fit plane using only points within RoI
[planeModel, inlierIndicesRoI, outlierIndicesRoI] = pcfitplane(ptCloudRoI, ...
    planeMaxDistance, planeReferenceVector, maxAngularDistance);

% Map RoI inliers back to original point cloud indices
inlierIndices = roi_indices(inlierIndicesRoI);
outlierIndices = setdiff(1:ptCloudScene.Count, inlierIndices);

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

% Project table points onto the fitted plane to work in 2D
tablePoints3D_cam = ptCloudTable_cam.Location;
planeOrigin_cam = -planeModel.Normal * planeModel.Parameters(4);  % A point on the plane
vecsToPoints = tablePoints3D_cam - planeOrigin_cam;
dists = vecsToPoints * Z_cam';
projectedTablePoints3D_cam = tablePoints3D_cam - dists .* Z_cam;

% Visualize plane fitting results
if showPlots
    figure('Name', 'Plane Fitting Visualization');
    
    % Plot original scene points
    pcshow(ptCloudScene.Location, [0.7 0.7 0.7], 'MarkerSize', 15);
    hold on;
    
    % Create a grid of points on the fitted plane
    [gridX, gridY] = meshgrid(min(ptCloudScene.Location(:,1)):0.05:max(ptCloudScene.Location(:,1)), ...
                             min(ptCloudScene.Location(:,2)):0.05:max(ptCloudScene.Location(:,2)));
    gridZ = (-planeModel.Parameters(1)*gridX - planeModel.Parameters(2)*gridY - planeModel.Parameters(4)) / planeModel.Parameters(3);
    
    % Plot the fitted plane surface
    surf(gridX, gridY, gridZ, 'FaceColor', 'g', 'FaceAlpha', 0.3, 'EdgeColor', 'none');
    
    % Draw Z_cam direction arrow from plane center
    planeCenterPoint = planeOrigin_cam;
    arrowLength = 0.3; % Length of the arrow in meters
    quiver3(planeCenterPoint(1), planeCenterPoint(2), planeCenterPoint(3), ...
            Z_cam(1)*arrowLength, Z_cam(2)*arrowLength, Z_cam(3)*arrowLength, ...
            'r', 'LineWidth', 3, 'MaxHeadSize', 0.5);
    
    title('Plane Fitting Results with Surface Normal');
    xlabel('X (m)'); ylabel('Y (m)'); zlabel('Z (m)');
    legend('Scene Points', 'Fitted Plane', 'Surface Normal (Z_{cam})', 'Location', 'best');
    axis equal; grid on;
    view(30, 25);
    hold off;
end

%% =======================================================================
%  DETECT TABLE EDGES VIA ROBUST LINE FITTING
%  =======================================================================
fprintf('Detecting table edges using RANSAC...\n');

% Define border region width (in meters)
borderWidth = 0.05; % 5cm border region along the edges

% Find the extents of the projected table points
minX = min(projectedTablePoints3D_cam(:,1));
maxX = max(projectedTablePoints3D_cam(:,1));
minY = min(projectedTablePoints3D_cam(:,2));
maxY = max(projectedTablePoints3D_cam(:,2));

% Filter points to keep only those in the border region
borderPoints = projectedTablePoints3D_cam(  (projectedTablePoints3D_cam(:,1) < minX + borderWidth) | ...
                                            (projectedTablePoints3D_cam(:,1) > maxX - borderWidth) | ...
                                            (projectedTablePoints3D_cam(:,2) < minY + borderWidth) | ...
                                            (projectedTablePoints3D_cam(:,2) > maxY - borderWidth), :);

fprintf('   - Using %d border points (%.1f%% of total) for edge detection\n', ...
    size(borderPoints,1), 100*size(borderPoints,1)/size(projectedTablePoints3D_cam,1));

% Convert to double and remove any NaN values
validPoints = borderPoints;
validPoints = double(validPoints);
validIdx = ~any(isnan(validPoints), 2);
validPoints = validPoints(validIdx, :);

% Now call boundary with clean data
k = boundary(validPoints(:,1), validPoints(:,2), 0.8); % Using a tighter shrink factor for border points
boundaryPoints3D = validPoints(k,:);

% Additional filtering to remove potential outliers
% Calculate distances between consecutive boundary points
nPoints = size(boundaryPoints3D, 1);
distances = zeros(nPoints, 1);
for i = 1:nPoints
    next_idx = mod(i, nPoints) + 1;
    distances(i) = norm(boundaryPoints3D(i,:) - boundaryPoints3D(next_idx,:));
end

% Identify and remove outliers (points with unusually large gaps)
meanDist = mean(distances);
stdDist = std(distances);
outlierThreshold = meanDist + 2*stdDist;
outlierIndices = find(distances > outlierThreshold);

% If outliers are found, refine the boundary
if ~isempty(outlierIndices)
    fprintf('   - Removing %d outlier boundary points\n', length(outlierIndices));
    % Remove the outliers and their neighbors
    removeIndices = unique([outlierIndices; mod(outlierIndices, nPoints) + 1]);
    keepIndices = setdiff(1:nPoints, removeIndices);
    boundaryPoints3D = boundaryPoints3D(keepIndices, :);
end

if showPlots
    figure('Name', 'Edge Fitting Visualization');

    % Create point cloud objects with colors
    pcshow(ptCloudTable_cam.Location, [0.7 0.7 0.7], 'MarkerSize', 15);
    hold on;
    
    % Visualize the border region used for edge detection
    if ~isempty(borderPoints)
        scatter3(borderPoints(:,1), borderPoints(:,2), borderPoints(:,3), 20, "green")
    end
    
    % Visualize the final boundary points
    plot3(boundaryPoints3D(:,1), boundaryPoints3D(:,2), boundaryPoints3D(:,3), 'r.', 'MarkerSize', 20);
    
    % Connect boundary points to show the detected perimeter
    for i = 1:size(boundaryPoints3D, 1)
        next_idx = mod(i, size(boundaryPoints3D, 1)) + 1;
        plot3([boundaryPoints3D(i,1), boundaryPoints3D(next_idx,1)], ...
              [boundaryPoints3D(i,2), boundaryPoints3D(next_idx,2)], ...
              [boundaryPoints3D(i,3), boundaryPoints3D(next_idx,3)], 'r-', 'LineWidth', 1);
    end
    
    title('Table Points with Border Region (Green) and Boundary Points (Red)');
    legend('Table Points', 'Border Region', 'Boundary Points', 'Location', 'best');
    hold off;
end

% Add current directory to path for RANSAC functions
addpath(fullfile(fileparts(mfilename('fullpath')), '.'));

%% First edge detection (Y-axis aligned with world Y)
fprintf('   - Detecting first edge (Y-axis)...\n');

% Use RANSAC to find the first edge line (expected to align with world Y)
[lineModel1, inlierIdx1] = ransac(boundaryPoints3D, @(pts) fitLine3d(pts(:,1),pts(:,2),pts(:,3)), ...
    @(model, pts) distPointToLine3d(pts, model), sampleSize, lineMaxDistance, 'MaxNumTrials', 1000);

edgeInlierPoints1 = boundaryPoints3D(inlierIdx1, :);

% Extract the direction vector of the first line (Y-axis)
Y_cam_initial = normalize(lineModel1(4:6), 'norm');

% Ensure the Y-axis is perpendicular to the Z-axis
Y_cam = normalize(Y_cam_initial - dot(Y_cam_initial, Z_cam) * Z_cam, 'norm');

% Ensure a consistent direction for the Y-axis (camera X aligned with world Y)
if Y_cam(1) < 0  % Check X component since camera X aligns with world Y
    Y_cam = -Y_cam;
end

fprintf('   - First edge found with direction vector (Y-axis): [%.3f, %.3f, %.3f]\n', Y_cam);

%% Second edge detection (X-axis aligned with world X)
fprintf('   - Detecting second edge (X-axis)...\n');

% Remove the first edge inliers from boundary points to find the second edge
remainingBoundaryPoints = boundaryPoints3D;
remainingBoundaryPoints(inlierIdx1, :) = [];

% Use RANSAC to find the second edge line (expected to align with world X)
% We need to ensure this line is roughly perpendicular to the first one
[lineModel2, inlierIdx2] = ransac(remainingBoundaryPoints, @(pts) fitLine3d(pts(:,1),pts(:,2),pts(:,3)), ...
    @(model, pts) distPointToLine3d(pts, model), sampleSize, lineMaxDistance, 'MaxNumTrials', 1000);

edgeInlierPoints2 = remainingBoundaryPoints(inlierIdx2, :);

% Extract the direction vector of the second line
X_cam_initial = normalize(lineModel2(4:6), 'norm');

% Ensure the X-axis is perpendicular to the Z-axis
X_cam_projected = normalize(X_cam_initial - dot(X_cam_initial, Z_cam) * Z_cam, 'norm');

% Check if X_cam_projected is roughly perpendicular to Y_cam
angle_between = acosd(abs(dot(X_cam_projected, Y_cam)));
fprintf('   - Angle between detected edges: %.2f degrees\n', angle_between);

% If the angle is not close to 90 degrees, we need to force orthogonality
if abs(angle_between - 90) > 15
    fprintf('   - Detected edges are not orthogonal. Forcing orthogonality.\n');
    % Force X_cam to be perpendicular to both Y_cam and Z_cam
    X_cam = cross(Y_cam, Z_cam);
else
    % Ensure X_cam is exactly perpendicular to Y_cam and Z_cam
    X_cam = cross(Y_cam, Z_cam);
    fprintf('   - Second edge found with direction vector (X-axis): [%.3f, %.3f, %.3f]\n', X_cam);
end

% Ensure a consistent direction for the X-axis (camera Y aligned with world X)
if X_cam(2) < 0  % Check Y component since camera Y aligns with world X
    X_cam = -X_cam;
end

if showPlots
    figure('Name', 'Line Fitting Results');
    % Create point cloud objects with colors
    pcshow(boundaryPoints3D, [0.7 0.7 0.7], 'MarkerSize', 30);
    hold on;
    
    % Plot the first edge (Y-axis)
    pcshow(edgeInlierPoints1, [0 1 0], 'MarkerSize', 50);
    linePts1 = [mean(edgeInlierPoints1) - Y_cam*0.5; mean(edgeInlierPoints1) + Y_cam*0.5];
    plot3(linePts1(:,1), linePts1(:,2), linePts1(:,3), 'g-', 'LineWidth', 3);
    
    % Plot the second edge (X-axis)
    pcshow(edgeInlierPoints2, [1 0 0], 'MarkerSize', 50);
    linePts2 = [mean(edgeInlierPoints2) - X_cam*0.5; mean(edgeInlierPoints2) + X_cam*0.5];
    plot3(linePts2(:,1), linePts2(:,2), linePts2(:,3), 'r-', 'LineWidth', 3);
    
    title('RANSAC Line Fits to Table Edges');
    legend('Boundary Points', 'Y-axis Edge Points', 'Y-axis Edge Line', ...
           'X-axis Edge Points', 'X-axis Edge Line');
    hold off;
end

%% =======================================================================
%  COMPUTE THE RIGID TRANSFORMATION FROM AXES
%  =======================================================================
fprintf('Computing the camera-to-world transformation from axes...\n');

% We now have all three axes: X_cam, Y_cam, and Z_cam
% Ensure they form an orthonormal basis (already done in edge detection)

% Verify orthogonality
xy_dot = abs(dot(X_cam, Y_cam));
xz_dot = abs(dot(X_cam, Z_cam));
yz_dot = abs(dot(Y_cam, Z_cam));

fprintf('   - Orthogonality check (should be close to 0):\n');
fprintf('     X·Y = %.6f, X·Z = %.6f, Y·Z = %.6f\n', xy_dot, xz_dot, yz_dot);

% The rotation matrix transforms the camera coordinate system to the world coordinate system
% In our setup:
% - Camera X axis (roughly aligned with world Y) -> World Y axis
% - Camera Y axis (roughly aligned with world X) -> World X axis
% - Camera Z axis (pointing down to table) -> World Z axis (pointing up)

% Create the rotation matrix with the correct mapping:
% [X_cam, Y_cam, Z_cam] -> [World X, World Y, World Z]
% Note: We need to swap X and Y and negate Z to match the world frame orientation
% Form the rotation matrix with columns as the basis vectors
R_cam_to_world_corrected = [X_cam; Y_cam; Z_cam]';  % Note: X and Y are swapped to match world frame

% Verify the rotation matrix is proper (det = 1)
det_R = det(R_cam_to_world_corrected);
fprintf('   - Determinant of rotation matrix: %.6f (should be close to 1)\n', det_R);

% Ensure it's a proper rotation matrix (orthogonal with determinant 1)
if abs(det_R - 1) > 0.01
    fprintf('   - Warning: Rotation matrix is not proper. Applying correction.\n');
    % Ensure orthogonality using SVD
    [U, ~, V] = svd(R_cam_to_world_corrected);
    R_cam_to_world = U * V';
else
    R_cam_to_world = R_cam_to_world_corrected;
end

fprintf('   - Table height from ground: %.2f m\n', tableHeight);

% Calculate the X-offset for the world origin (changed from Y-offset)
if ischar(worldOriginYOffset) && strcmpi(worldOriginYOffset, 'auto')
    % Automatically determine table length and place origin at the middle
    % Project all table points onto the XY plane of our new coordinate system
    tablePoints_projected = (R_cam_to_world * (tablePoints3D_cam - planeOrigin_cam)')';
    
    % Find the extent of the table in the X direction
    minX = min(tablePoints_projected(:, 1));
    maxX = max(tablePoints_projected(:, 1));
    detectedTableLength = maxX - minX;
    
    % Simply negate the X-offset to move into the table instead of away from it
    xOffset = detectedTableLength / 2;
    fprintf('   - Auto-detected table length: %.3f m\n', detectedTableLength);
    fprintf('   - Setting X-offset to table middle: %.3f m\n', xOffset);
elseif isnumeric(worldOriginYOffset)
    % Use the user-specified offset (now for X)
    xOffset = -worldOriginYOffset;  % Negate to move into the table
    fprintf('   - Using user-specified X-offset: %.3f m\n', xOffset);
else
    % Default to edge (no offset)
    xOffset = 0;
    fprintf('   - Using default X-offset (table edge): %.3f m\n', xOffset);
end

% Apply the X-offset to create the world origin
% The origin is at the center of the table (X=offset, Y=0) but at ground level (Z=0)
% Table is at Z=tableHeight in world frame
P_cam_origin = mean(edgeInlierPoints1, 1);
P_world_origin = [xOffset, 0, tableHeight]; % Origin at ground level (Z=0)

% The transformation equation is: P_world = R * P_cam + T
% So, T = P_world_origin' - R * P_cam_origin'
T_cam_to_world = P_world_origin' - R_cam_to_world * P_cam_origin';

% Refine the translation to ensure table points are exactly at tableHeight
% This helps correct any small errors in the transformation
fprintf('   - Refining translation to ensure table points align with tableHeight...\n');
ptCloudTable_world_test = pctransform(ptCloudTable_cam, rigidtform3d(R_cam_to_world, T_cam_to_world'));
table_heights = ptCloudTable_world_test.Location(:,3);
height_error = tableHeight - median(table_heights);
fprintf('   - Height adjustment: %.4f m\n', height_error);

% Apply the height correction to the translation
T_cam_to_world(3) = T_cam_to_world(3) + height_error;

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
    figure('Name', 'Final Calibrated Simulation Scene (Two-Edge Based)', 'Position', [100, 100, 1000, 800]);
    
    % Create a simulated table surface (tableLength x tableWidth centered at world origin)
    [X, Y] = meshgrid([-tableLength/2:0.05:tableLength/2], [-tableWidth/2:0.05:tableWidth/2]);
    Z = zeros(size(X)) + tableHeight;  % Table is at Z=tableHeight in world frame
    surf(X, Y, Z, 'FaceColor', [0.8 0.7 0.6], 'FaceAlpha', 0.3, 'EdgeColor', 'none'); hold on;
    
    % Add table edges for reference
    plot3([-tableLength/2, tableLength/2], [-tableWidth/2, -tableWidth/2], [tableHeight, tableHeight], 'k-', 'LineWidth', 2); hold on;
    plot3([-tableLength/2, -tableLength/2], [-tableWidth/2, tableWidth/2], [tableHeight, tableHeight], 'k-', 'LineWidth', 2); hold on;
    plot3([tableLength/2, tableLength/2], [-tableWidth/2, tableWidth/2], [tableHeight, tableHeight], 'k-', 'LineWidth', 2); hold on;
    plot3([-tableLength/2, tableLength/2], [tableWidth/2, tableWidth/2], [tableHeight, tableHeight], 'k-', 'LineWidth', 2); hold on;
    
    % Visualize the RoI used for plane fitting
    % Exchange X and Y since RoI is expressed in camera coordinates
    % and camera X is aligned with world Y
    roi_y = tableRoI(1);  % Camera X -> World Y
    roi_x = tableRoI(2);  % Camera Y -> World X
    roi_z = tableRoI(3);
    roi_corners = [
        -roi_x/2, -roi_y/2, tableHeight;
        roi_x/2, -roi_y/2, tableHeight;
        roi_x/2, roi_y/2, tableHeight;
        -roi_x/2, roi_y/2, tableHeight;
        -roi_x/2, -roi_y/2, tableHeight
    ];
    plot3(roi_corners(:,1), roi_corners(:,2), roi_corners(:,3), 'g-', 'LineWidth', 2); hold on;
    text(0, 0, tableHeight + 0.05, 'Table RoI', 'Color', 'g', 'FontWeight', 'bold');
    
    % Visualize the transformed point clouds
    pcshow(ptCloudTable_world.Location, 'y', 'MarkerSize', 20); hold on;
    pcshow(ptCloudRemaining_world.Location, [0.7 0.7 0.7], 'MarkerSize', 20); hold on;
    pcshow(ptCloudObject_world.Location, 'r', 'MarkerSize', 50); hold on;
    
    % Add a horizontal plane at table height to check alignment
    [checkX, checkY] = meshgrid([-0.1:0.05:0.1], [-0.1:0.05:0.1]);
    checkZ = zeros(size(checkX)) + tableHeight;
    surf(checkX, checkY, checkZ, 'FaceColor', [0 0.5 1], 'FaceAlpha', 0.2, 'EdgeColor', 'none'); hold on;
    
    % Plot the camera's position and orientation in the world frame
    plotCamera('AbsolutePose', tform_cam_to_world, 'Size', 0.1, 'Color', 'b', 'Opacity', 0.2); hold on;
    
    % Add coordinate frame axes for clarity
    plot3([0 0.2], [0 0], [0 0], 'r-', 'LineWidth', 3); text(0.2, 0.01, 0, 'World X', 'Color', 'r'); hold on;
    plot3([0 0], [0 0.2], [0 0], 'g-', 'LineWidth', 3); text(0, 0.21, 0, 'World Y', 'Color', 'g'); hold on;
    plot3([0 0], [0 0], [0 0.2], 'b-', 'LineWidth', 3); text(0, 0.01, 0.2, 'World Z', 'Color', 'b'); hold on;
    
    % Visualize the table height (from ground up to table)
    plot3([xOffset xOffset], [0 0], [0 tableHeight], 'k--', 'LineWidth', 2); hold on;
    text(xOffset, 0.02, tableHeight/2, sprintf('Table Height: %.2f m', tableHeight), 'Color', 'k'); hold on;
    
    % Add coordinate axes at the detected table corners to verify alignment
    cornerSize = 0.1;
    % Bottom-left corner
    plot3([-tableLength/2, -tableLength/2+cornerSize], [-tableWidth/2, -tableWidth/2], [tableHeight, tableHeight], 'r-', 'LineWidth', 2); hold on;
    plot3([-tableLength/2, -tableLength/2], [-tableWidth/2, -tableWidth/2+cornerSize], [tableHeight, tableHeight], 'g-', 'LineWidth', 2); hold on;
    
    % Final plot adjustments
    title('Final Calibrated Scene (Two-Edge Based)');
    xlabel('X (m)'); ylabel('Y (m)'); zlabel('Z (m)');
    axis equal; grid on; view(30, 25);
    legend('Table Surface', 'Table Edge', 'Table RoI', 'Table Points', 'Remaining Scene', 'Transformed Object', 'Alignment Check', 'Location', 'northeast');
end

fprintf('--- Calibration Complete ---\n');
end

% Note: fitLine3d and distPointToLine3d are external functions
