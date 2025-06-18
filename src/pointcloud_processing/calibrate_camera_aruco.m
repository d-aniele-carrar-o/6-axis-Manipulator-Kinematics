function [tform_cam_to_world] = calibrate_camera_aruco(rgbImagePath, depthImagePath, markerParams, tableParams, showPlots)
% CALIBRATE_CAMERA_ARUCO Camera calibration using ArUco markers
%
% This function implements camera calibration using ArUco markers placed at known
% locations. ArUco markers are easy to detect and provide precise pose estimation.
%
% Inputs:
%   rgbImagePath - Path to the RGB image file
%   depthImagePath - Path to the depth image file
%   markerParams - Structure with marker parameters:
%       .dictionary - ArUco dictionary to use (default: 'DICT_6X6_250')
%       .markerSize - Size of the marker in meters (default: 0.08)
%       .markerLocations - Nx4 matrix with [markerId, x, y, z] for each marker
%   tableParams - Structure with table parameters (height)
%   showPlots - Boolean to control visualization (default: false)
%
% Outputs:
%   tform_cam_to_world - Rigid transformation from camera to world frame

% Handle optional arguments
if nargin < 5
    showPlots = false;
end

% Default marker parameters
if ~isfield(markerParams, 'dictionary')
    markerParams.dictionary = 'DICT_6X6_250';
end

if ~isfield(markerParams, 'markerSize')
    markerParams.markerSize = 0.08; % 8cm markers
end

% Extract table parameters
tableHeight = tableParams.height;

% Load RGB image
fprintf('Loading RGB image...\n');
rgbImage = imread(rgbImagePath);

% Detect ArUco markers
fprintf('Detecting ArUco markers...\n');
dictionary = arucoDictionary(markerParams.dictionary);
[ids, corners, rejected] = arucoDetectMarkers(rgbImage, dictionary);

% Check if markers were detected
if isempty(ids)
    error('No ArUco markers detected in the image.');
end

fprintf('Detected %d ArUco markers\n', length(ids));

% Get camera intrinsic parameters
% Either load from calibration file or use default values
cameraParams = getCameraParams(size(rgbImage));

% Estimate pose for each marker
worldPoints = generateMarkerObjectPoints(markerParams.markerSize);
detectedMarkers = length(ids);
markerPoses = cell(detectedMarkers, 1);

for i = 1:detectedMarkers
    % Get corners for this marker
    markerCorners = corners{i};
    
    % Estimate pose
    [rotationVector, translationVector] = estimateSingleMarkerPose(markerCorners, worldPoints, cameraParams);
    
    % Convert rotation vector to rotation matrix
    rotationMatrix = rodrigues(rotationVector);
    
    % Store pose
    markerPoses{i}.id = ids(i);
    markerPoses{i}.corners = markerCorners;
    markerPoses{i}.rotation = rotationMatrix;
    markerPoses{i}.translation = translationVector;
end

% Match detected markers with known locations
fprintf('Matching detected markers with known locations...\n');
matchedMarkers = 0;
matchedPoses = [];

for i = 1:detectedMarkers
    markerId = markerPoses{i}.id;
    
    % Find this marker in the known locations
    markerIdx = find(markerParams.markerLocations(:,1) == markerId);
    
    if ~isempty(markerIdx)
        matchedMarkers = matchedMarkers + 1;
        
        % Get marker's known position in world frame
        markerWorldPos = markerParams.markerLocations(markerIdx, 2:4);
        
        % Store matched pose
        matchedPoses(matchedMarkers).id = markerId;
        matchedPoses(matchedMarkers).rotation = markerPoses{i}.rotation;
        matchedPoses(matchedMarkers).translation = markerPoses{i}.translation;
        matchedPoses(matchedMarkers).worldPosition = markerWorldPos;
    end
end

fprintf('Matched %d markers with known locations\n', matchedMarkers);

if matchedMarkers < 1
    error('No markers matched with known locations. Check marker IDs.');
end

% Compute camera-to-world transformation
fprintf('Computing camera-to-world transformation...\n');

% Initialize transformation matrices
allTransforms = zeros(4, 4, matchedMarkers);

for i = 1:matchedMarkers
    % Get marker pose in camera frame
    R_cam_to_marker = matchedPoses(i).rotation;
    t_cam_to_marker = matchedPoses(i).translation;
    
    % Create transformation from camera to marker
    T_cam_to_marker = eye(4);
    T_cam_to_marker(1:3, 1:3) = R_cam_to_marker;
    T_cam_to_marker(1:3, 4) = t_cam_to_marker;
    
    % Get marker position in world frame
    markerWorldPos = matchedPoses(i).worldPosition;
    
    % Create transformation from marker to world
    % Assuming marker's orientation is aligned with world frame
    T_marker_to_world = eye(4);
    T_marker_to_world(1:3, 4) = markerWorldPos';
    
    % Compute camera to world transformation
    T_cam_to_world = T_marker_to_world / T_cam_to_marker;
    
    % Store this transformation
    allTransforms(:,:,i) = T_cam_to_world;
end

% Average all transformations
if matchedMarkers > 1
    % For rotation, convert to quaternions, average, then convert back
    allQuats = zeros(matchedMarkers, 4);
    allTrans = zeros(matchedMarkers, 3);
    
    for i = 1:matchedMarkers
        R = allTransforms(1:3, 1:3, i);
        t = allTransforms(1:3, 4, i);
        
        % Convert rotation to quaternion
        q = rotm2quat(R);
        
        % Store
        allQuats(i,:) = q;
        allTrans(i,:) = t';
    end
    
    % Average quaternions (ensuring they're in the same hemisphere)
    for i = 2:matchedMarkers
        if dot(allQuats(1,:), allQuats(i,:)) < 0
            allQuats(i,:) = -allQuats(i,:);
        end
    end
    
    avgQuat = mean(allQuats, 1);
    avgQuat = avgQuat / norm(avgQuat); % Normalize
    
    % Average translations
    avgTrans = mean(allTrans, 1);
    
    % Create final transformation
    R_cam_to_world = quat2rotm(avgQuat);
    t_cam_to_world = avgTrans';
else
    % Just use the single transformation
    R_cam_to_world = allTransforms(1:3, 1:3, 1);
    t_cam_to_world = allTransforms(1:3, 4, 1);
end

% Create the final transformation matrix
tform_cam_to_world = rigidtform3d(R_cam_to_world, t_cam_to_world);

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
    figure('Name', 'ArUco Marker Calibration', 'Position', [100, 100, 1200, 800]);
    
    % Plot the original RGB image with detected markers
    subplot(2, 2, 1);
    imshow(rgbImage);
    hold on;
    
    % Draw detected markers
    for i = 1:detectedMarkers
        markerCorners = corners{i};
        plot(markerCorners(:,1), markerCorners(:,2), 'g-', 'LineWidth', 2);
        text(markerCorners(1,1), markerCorners(1,2), num2str(ids(i)), ...
            'Color', 'red', 'FontSize', 12);
    end
    
    title('Detected ArUco Markers');
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
    
    % Draw marker positions in world coordinates
    for i = 1:matchedMarkers
        pos = matchedPoses(i).worldPosition;
        plot3(pos(1), pos(2), pos(3), 'ro', 'MarkerSize', 10, 'MarkerFaceColor', 'r');
        text(pos(1), pos(2), pos(3), num2str(matchedPoses(i).id), ...
            'Color', 'white', 'FontSize', 12);
    end
    
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
    
    % Plot marker positions
    for i = 1:matchedMarkers
        pos = matchedPoses(i).worldPosition;
        plot3(pos(1), pos(2), pos(3), 'ro', 'MarkerSize', 10, 'MarkerFaceColor', 'r');
    end
    
    title('Camera Position in World Coordinates');
    xlabel('X (m)'); ylabel('Y (m)'); zlabel('Z (m)');
    axis equal;
    grid on;
    view(30, 30);
end

end

%% Helper Functions

function dictionary = arucoDictionary(name)
    % Create ArUco dictionary based on name
    % This is a wrapper for OpenCV's predefined dictionaries
    
    % Map dictionary names to their OpenCV constants
    dictMap = struct();
    dictMap.DICT_4X4_50 = 0;
    dictMap.DICT_4X4_100 = 1;
    dictMap.DICT_4X4_250 = 2;
    dictMap.DICT_4X4_1000 = 3;
    dictMap.DICT_5X5_50 = 4;
    dictMap.DICT_5X5_100 = 5;
    dictMap.DICT_5X5_250 = 6;
    dictMap.DICT_5X5_1000 = 7;
    dictMap.DICT_6X6_50 = 8;
    dictMap.DICT_6X6_100 = 9;
    dictMap.DICT_6X6_250 = 10;
    dictMap.DICT_6X6_1000 = 11;
    dictMap.DICT_7X7_50 = 12;
    dictMap.DICT_7X7_100 = 13;
    dictMap.DICT_7X7_250 = 14;
    dictMap.DICT_7X7_1000 = 15;
    dictMap.DICT_ARUCO_ORIGINAL = 16;
    
    % Get dictionary constant
    if isfield(dictMap, name)
        dictConst = dictMap.(name);
    else
        dictConst = dictMap.DICT_6X6_250; % Default
        warning('Unknown dictionary name. Using DICT_6X6_250 as default.');
    end
    
    % Create dictionary
    dictionary = cv.aruco.Dictionary.get(dictConst);
end

function [ids, corners, rejected] = arucoDetectMarkers(image, dictionary)
    % Detect ArUco markers in an image
    % This is a wrapper for OpenCV's detectMarkers function
    
    % Convert image to grayscale if needed
    if size(image, 3) == 3
        grayImage = rgb2gray(image);
    else
        grayImage = image;
    end
    
    % Detect markers
    [corners, ids, rejected] = cv.aruco.detectMarkers(grayImage, dictionary);
end

function cameraParams = getCameraParams(imageSize)
    % Get camera intrinsic parameters
    % Either load from calibration file or use default values
    
    % Default values for a typical RGB-D camera
    fx = 525.0; % focal length x
    fy = 525.0; % focal length y
    cx = imageSize(2)/2; % principal point x
    cy = imageSize(1)/2; % principal point y
    
    % Create camera parameters object
    cameraMatrix = [fx, 0, cx; 0, fy, cy; 0, 0, 1];
    distCoeffs = zeros(1, 5); % Assume no distortion for simplicity
    
    cameraParams = struct('IntrinsicMatrix', cameraMatrix, 'DistortionCoefficients', distCoeffs);
end

function worldPoints = generateMarkerObjectPoints(markerSize)
    % Generate 3D coordinates of marker corners in marker's local coordinate system
    % ArUco markers are square with the origin at the center
    
    halfSize = markerSize / 2;
    worldPoints = [
        -halfSize, halfSize, 0;   % Top-left
        halfSize, halfSize, 0;    % Top-right
        halfSize, -halfSize, 0;   % Bottom-right
        -halfSize, -halfSize, 0   % Bottom-left
    ];
end

function [rotationVector, translationVector] = estimateSingleMarkerPose(imagePoints, worldPoints, cameraParams)
    % Estimate pose of a single marker
    % This is a wrapper for OpenCV's solvePnP function
    
    % Extract camera matrix and distortion coefficients
    cameraMatrix = cameraParams.IntrinsicMatrix;
    distCoeffs = cameraParams.DistortionCoefficients;
    
    % Solve PnP problem
    [success, rotationVector, translationVector] = cv.solvePnP(worldPoints, imagePoints, cameraMatrix, distCoeffs);
    
    if ~success
        error('Failed to estimate marker pose.');
    end
end

function rotationMatrix = rodrigues(rotationVector)
    % Convert rotation vector to rotation matrix using Rodrigues formula
    % This is a wrapper for OpenCV's Rodrigues function
    
    rotationMatrix = cv.Rodrigues(rotationVector);
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