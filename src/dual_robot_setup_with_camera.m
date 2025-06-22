% Dual robot setup and simulation with camera calibration
clear; clc; close all;

% Setup first robot (UR3e left)
parameters(0, 1);
robot_left  = robot;
config_left = config;
Trf_0_l     = Trf_0;

% Setup second robot (UR3e right)
parameters(0, 2);
robot_right  = robot;
config_right = config;
Trf_0_r      = Trf_0;

% Create environment for multi-robot setup
figure('Name', 'Dual Robot Workspace with Camera');
hold on; grid on;

tableParams.height = tableHeight;
tableParams.width  = tableWidth;
tableParams.length = tableLength;
tableParams.roi    = tableRoI;

axs = create_environment( tablePosition, tableParams );

% Load camera calibration from Python script
fprintf('\n===== LOADING CAMERA CALIBRATION =====\n');
try
    tform_cam_to_world = load_camera_calibration();
    
    % Add camera visualization
    plotCamera('AbsolutePose', tform_cam_to_world, 'Size', 0.1, 'Color', 'b', 'Opacity', 0.7, 'Parent', axs);
    text(tform_cam_to_world.Translation(1), tform_cam_to_world.Translation(2), tform_cam_to_world.Translation(3) + 0.1, ...
        'Camera', 'Color', 'b', 'FontSize', 12, 'FontWeight', 'bold', 'Parent', axs);
    
    fprintf('Camera calibration loaded successfully!\n');
    
catch ME
    warning(ME.identifier, '%s', ME.message);
    fprintf('Continuing without camera visualization...\n');
    tform_cam_to_world = [];
end

% File Paths
project_root = fileparts(fileparts(mfilename('fullpath')));
default_scenePcdPath = fullfile(project_root, 'output', 'segmented_objects', '25-06-07-11-21-29', 'table_surface.ply');
default_objectPcdPath = fullfile(project_root, 'output', 'segmented_objects', '25-06-07-11-21-29', 'object_01.ply');

% Processing mode selection
fprintf('\n===== PROCESSING MODE SELECTION =====\n');
useDirectory = true;
fprintf('Using directory mode processing.\n');

if useDirectory
    % Directory processing
    fprintf('\n===== DIRECTORY PATH INPUT =====\n');
    dirPath = fullfile(project_root, 'output', 'augmented_demos', '25-06-07-11-21-29', 'scene_visualizations');
    fprintf('Using directory: %s\n', dirPath);
    
    if ~exist(dirPath, 'dir')
        error('Directory not found: %s', dirPath);
    end
    
    scenePcdPath = default_scenePcdPath;
    disp(['Using calibration scene: ' scenePcdPath]);
    disp(['Processing scenes from directory: ' dirPath]);
    
    if ~exist(scenePcdPath, 'file')
        error('Calibration scene point cloud file not found: %s', scenePcdPath);
    end
    
    isMultipleObjects = true;
else
    % Single object processing
    fprintf('\n===== SCENE POINT CLOUD INPUT =====\n');
    scenePcdPath = default_scenePcdPath;
    objectPcdPath = default_objectPcdPath;
    
    if ~exist(scenePcdPath, 'file')
        error('Scene point cloud file not found: %s', scenePcdPath);
    end
    
    if ~exist(objectPcdPath, 'file')
        error('Object point cloud file not found: %s', objectPcdPath);
    end
    
    % Use existing calibration or fallback to table-based calibration
    if ~isempty(tform_cam_to_world)
        % Transform point clouds using loaded calibration
        fprintf('Using loaded camera calibration to transform point clouds...\n');
        
        % Load point clouds in camera coordinates
        ptCloudScene_cam = pcread(scenePcdPath);
        ptCloudObject_cam = pcread(objectPcdPath);
        
        % Transform to world coordinates
        ptCloudObject_world = pctransform(ptCloudObject_cam, tform_cam_to_world);
        ptCloudRemaining_world = pctransform(ptCloudScene_cam, tform_cam_to_world);
        
    else
        % Fallback to table-based calibration
        fprintf('Using table-based calibration...\n');
        [tform_cam_to_world, ptCloudObject_world, ptCloudRemaining_world] = calibrate_camera(scenePcdPath, objectPcdPath, tableParams, true);
    end
    
    % Display camera transformation
    disp('Camera-to-world transformation:');
    disp(tform_cam_to_world.A);
    
    % Add scene point cloud
    pcshow(ptCloudRemaining_world.Location, [0.7 0.7 0.7], 'MarkerSize', 10, 'Parent', axs);
    
    % Add object point cloud
    pcshow(ptCloudObject_world.Location, 'r', 'MarkerSize', 30, 'Parent', axs);
    text(mean(ptCloudObject_world.Location(:,1)), mean(ptCloudObject_world.Location(:,2)), mean(ptCloudObject_world.Location(:,3)) + 0.05, ...
        'Object', 'Color', 'r', 'Parent', axs);
    
    % Display object position
    object_centroid = mean(ptCloudObject_world.Location);
    disp('Object centroid position in world frame:');
    disp(object_centroid);
    
    isMultipleObjects = false;
end

% Initial joint configurations
q0_left  = [ pi/2,   -pi/3,  2*pi/3,   -pi/3,  pi/2, 0];
q0_right = [-pi/2, -2*pi/3, -2*pi/3, -2*pi/3, -pi/2, 0];

% Set robot configurations
config_left  = set_robot_configuration( q0_left,  config_left );
config_right = set_robot_configuration( q0_right, config_right );

% Visualize robots
show( robot_left,  config_left,  "Visuals", "on", "Frames", "on", "FastUpdate", true, "PreservePlot", false, "Parent", axs );
show( robot_right, config_right, "Visuals", "on", "Frames", "on", "FastUpdate", true, "PreservePlot", false, "Parent", axs );

% Add text labels for each robot
left_base_pos = Trf_0_l(1:3,4);
right_base_pos = Trf_0_r(1:3,4);
text(left_base_pos(1), left_base_pos(2), tableHeight + 0.01, 'Robot Left', ...
    'Color', 'b', 'FontSize', 12, 'FontWeight', 'bold', 'HorizontalAlignment', 'center', 'Parent', axs);
text(right_base_pos(1), right_base_pos(2), tableHeight + 0.01, 'Robot Right', ...
    'Color', 'r', 'FontSize', 12, 'FontWeight', 'bold', 'HorizontalAlignment', 'center', 'Parent', axs);

% Compute end-effector poses 
parameters(1, 1);
[Te_w_e_left,  Te_l] = direct_kinematics( q0_left, 1 );

parameters(1, 2);
[Te_w_e_right, Te_r] = direct_kinematics( q0_right, 2 );

% Display end-effector positions
disp('Robot left end-effector pose (wTee):');
disp(Te_w_e_left);
disp('Robot right end-effector pose (wTee):');
disp(Te_w_e_right);

% Process objects and find grasp points
if isMultipleObjects
    % Process directory of scenes
    disp("Processing directory of scenes to find objects and grasp points...");
    
    % If we have camera calibration, we can use it for processing
    if ~isempty(tform_cam_to_world)
        % Pass camera calibration to processing functions
        [allGraspPoints, allGraspOrientations] = process_scene_directory_with_camera(dirPath, Te_w_e_left, Te_w_e_right, tableParams, tform_cam_to_world);
    else
        [allGraspPoints, allGraspOrientations] = process_scene_directory(dirPath, Te_w_e_left, Te_w_e_right, tableParams);
    end
    
    if ~isempty(allGraspPoints) && ~isempty(allGraspPoints{1}) && ~isempty(allGraspPoints{1}{1})
        grasp_points = allGraspPoints{1}{1};
        grasp_orientations = allGraspOrientations{1}{1};
        disp('Using grasp points from first object in first scene:');
    else
        error('No valid grasp points found in the processed scenes.');
    end
else
    % Find grasp points on the single object
    disp("Computing grasp points for given object:");
    [grasp_points, grasp_orientations] = find_object_grasp_points(ptCloudObject_world, Te_w_e_left, Te_w_e_right);
end

% Display grasp points
disp('Grasp points:');
disp(grasp_points);

% Visualize grasp points
scatter3(grasp_points(1,1), grasp_points(2,1), grasp_points(3,1), 100, 'g*', 'Parent', axs);
scatter3(grasp_points(1,2), grasp_points(2,2), grasp_points(3,2), 100, 'g*', 'Parent', axs);
text(grasp_points(1,1), grasp_points(2,1), grasp_points(3,1) + 0.05, 'Grasp 1', 'Color', 'g', 'Parent', axs);
text(grasp_points(1,2), grasp_points(2,2), grasp_points(3,2) + 0.05, 'Grasp 2', 'Color', 'g', 'Parent', axs);

% Visualize grasp orientations
for i = 1:2
    pos = grasp_points(:,i);
    R = squeeze(grasp_orientations(:,:,i));
    
    scale = 0.05;
    quiver3(pos(1), pos(2), pos(3), R(1,1), R(2,1), R(3,1), scale, 'r', 'LineWidth', 2, 'Parent', axs);
    quiver3(pos(1), pos(2), pos(3), R(1,2), R(2,2), R(3,2), scale, 'g', 'LineWidth', 2, 'Parent', axs);
    quiver3(pos(1), pos(2), pos(3), R(1,3), R(2,3), R(3,3), scale, 'b', 'LineWidth', 2, 'Parent', axs);
end

view(90, 30);
fprintf('\n===== CAMERA CALIBRATION INTEGRATED =====\n');
if ~isempty(tform_cam_to_world)
    fprintf('Camera calibration successfully integrated into simulation!\n');
    fprintf('Camera position: [%.3f, %.3f, %.3f] m\n', tform_cam_to_world.Translation);
else
    fprintf('Using fallback table-based calibration.\n');
end

fprintf('\nPress Enter to continue to trajectory planning\n');
pause()

% Continue with trajectory planning as in original script...
% [Rest of the trajectory planning code remains the same]