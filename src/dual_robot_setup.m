% Dual robot setup and simulation
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

% Create environment for multi-robot setup --------------------------------------------------------
figure('Name', 'Dual Robot Workspace');
hold on; grid on;

tableParams.height = tableHeight;
tableParams.width  = tableWidth;
tableParams.length = tableLength;
tableParams.roi    = tableRoI;

axs = create_environment( tablePosition, tableParams );

% ---------- File Paths ----------
% Use absolute paths to ensure files are found regardless of current directory
project_root = fileparts(fileparts(mfilename('fullpath')));

% Define default paths to pointclouds
default_scenePcdPath = fullfile(project_root, 'output', 'segmented_objects', '25-06-07-11-21-29', 'table_surface.ply');
default_objectPcdPath = fullfile(project_root, 'output', 'segmented_objects', '25-06-07-11-21-29', 'object_01.ply');

% Ask user if they want to use a single object or a directory of scenes
fprintf('\n===== PROCESSING MODE SELECTION =====\n');
useDirectory = true;
fprintf('Using directory mode processing.\n');

if useDirectory
    % Get directory path from user
    fprintf('\n===== DIRECTORY PATH INPUT =====\n');
    dirPath = fullfile(project_root, 'output', 'augmented_demos', '25-06-07-11-21-29', 'scene_visualizations');
    fprintf('Using directory: %s\n', dirPath);
    
    % Check if directory exists
    if ~exist(dirPath, 'dir')
        error('Directory not found: %s', dirPath);
    end
    
    % Use default scene for calibration
    scenePcdPath = default_scenePcdPath;
    
    % Display paths
    disp(['Using calibration scene: ' scenePcdPath]);
    disp(['Processing scenes from directory: ' dirPath]);
    disp("------------------------------------------------");
    
    % Verify calibration file exists
    if ~exist(scenePcdPath, 'file')
        error('Calibration scene point cloud file not found: %s', scenePcdPath);
    end
    
    % We'll process the directory later after setting up robots
    isMultipleObjects = true;
else
    % Get paths for single object processing
    fprintf('\n===== SCENE POINT CLOUD INPUT =====\n');
    fprintf('Path to scene point cloud: %S\n', default_scenePcdPath);
    scenePcdPath = default_scenePcdPath;
    
    fprintf('\n===== OBJECT POINT CLOUD INPUT =====\n');
    fprintf('Path to object point cloud: %s\n', default_objectPcdPath);
    objectPcdPath = default_objectPcdPath;
    
    % Verify files exist
    if ~exist(scenePcdPath, 'file')
        error('Scene point cloud file not found: %s', scenePcdPath);
    end
    
    if ~exist(objectPcdPath, 'file')
        error('Object point cloud file not found: %s', objectPcdPath);
    end
    disp("------------------------------------------------");
    
    % Calibrate camera and transform object point cloud
    [tform_cam_to_world, ptCloudObject_world, ptCloudRemaining_world] = calibrate_camera(scenePcdPath, objectPcdPath, tableParams, true);
    
    % Display camera transformation
    disp('Camera-to-world transformation:');
    disp(tform_cam_to_world.A);
    
    % Add camera visualization
    plotCamera('AbsolutePose', tform_cam_to_world, 'Size', 0.1, 'Color', 'b', 'Opacity', 0.5, 'Parent', axs);
    text(tform_cam_to_world.Translation(1), tform_cam_to_world.Translation(2), tform_cam_to_world.Translation(3) + 0.1, ...
        'Camera', 'Color', 'b', 'Parent', axs);
    
    % Add scene point cloud
    pcshow(ptCloudRemaining_world.Location, [0.7 0.7 0.7], 'MarkerSize', 10);
    
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

% Initial joint configurations --------------------------------------------------------------------
q0_left  = [ pi/2,   -pi/3,  2*pi/3,   -pi/3,  pi/2, 0];
q0_right = [-pi/2, -2*pi/3, -2*pi/3, -2*pi/3, -pi/2, 0];
% q0_left  = deg2rad([76.24, -31.87, 105.41, -73.01, 82.20, 8.99]);
% q0_right = deg2rad([-64.75, -152.59, -88.65, -118.93, -57.61, 311.00]);

% Set robot configurations
config_left  = set_robot_configuration( q0_left,  config_left );
config_right = set_robot_configuration( q0_right, config_right );

% Visualize robots
show( robot_left,  config_left,  "Visuals", "on", "Frames", "on", "FastUpdate", true, "PreservePlot", false, "Parent", axs );
show( robot_right, config_right, "Visuals", "on", "Frames", "on", "FastUpdate", true, "PreservePlot", false, "Parent", axs );

% Add text labels for each robot on the table surface
left_base_pos = Trf_0_l(1:3,4);
right_base_pos = Trf_0_r(1:3,4);
text(left_base_pos(1), left_base_pos(2), tableHeight + 0.01, 'Robot Left', ...
    'Color', 'b', 'FontSize', 12, 'FontWeight', 'bold', 'HorizontalAlignment', 'center', 'Parent', axs);
text(right_base_pos(1), right_base_pos(2), tableHeight + 0.01, 'Robot Right', ...
    'Color', 'r', 'FontSize', 12, 'FontWeight', 'bold', 'HorizontalAlignment', 'center', 'Parent', axs);

% Compute end-effector poses 
parameters(1, 1); % Load parameters for robot 1
[Te_w_e_left,  Te_l] = direct_kinematics( q0_left, 1 );

parameters(1, 2); % Load parameters for robot 2
[Te_w_e_right, Te_r] = direct_kinematics( q0_right, 2 );

% Display end-effector positions
disp('Robot left end-effector pose (wTee):');
disp(Te_w_e_left);
disp('Robot left end-effector pose (bTee):');
disp(Te_l)
disp('--------------------------------------')
disp('Robot right end-effector pose (wTee):');
disp(Te_w_e_right);
disp('Robot right end-effector pose (bTee):');
disp(Te_r)
disp('--------------------------------------')

% Process objects and find grasp points
if isMultipleObjects
    % Process directory of scenes
    disp("Processing directory of scenes to find objects and grasp points...");
    [allGraspPoints, allGraspOrientations] = process_scene_directory(dirPath, Te_w_e_left, Te_w_e_right, tableParams);
    
    % For now, we'll use the first object from the first scene for visualization and simulation
    % In a real application, you might want to process each scene separately
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

%%
% Grasp points got from real robots for the box moving task:
% - left:  [0.05595, -0.38201, -0.00933][x,y,z], [76.24, -31.87, 105.41, -73.01, 82.20, 8.99]
% - right: [0.00680, -0.43632, -0.00822][x,y,z], [-64.75, -152.59, -88.65, -118.93, -57.61, 311.00]
%%

% Display grasp points
disp('Grasp points:');
disp(grasp_points);

% Visualize grasp points
scatter3(grasp_points(1,1), grasp_points(2,1), grasp_points(3,1), 100, 'g*', 'Parent', axs);
scatter3(grasp_points(1,2), grasp_points(2,2), grasp_points(3,2), 100, 'g*', 'Parent', axs);
text(grasp_points(1,1), grasp_points(2,1), grasp_points(3,1) + 0.05, 'Grasp 1', 'Color', 'g', 'Parent', axs);
text(grasp_points(1,2), grasp_points(2,2), grasp_points(3,2) + 0.05, 'Grasp 2', 'Color', 'g', 'Parent', axs);

% Visualize grasp orientations with coordinate frames
for i = 1:2
    % Extract position and orientation for current grasp point
    pos = grasp_points(:,i);
    R = squeeze(grasp_orientations(:,:,i));
    
    % Plot coordinate frame axes (x=red, y=green, z=blue)
    scale = 0.05; % Scale factor for axis visualization
    % X axis
    quiver3(pos(1), pos(2), pos(3), ...
           R(1,1), R(2,1), R(3,1), ...
           scale, 'r', 'LineWidth', 2, 'Parent', axs);
    % Y axis           
    quiver3(pos(1), pos(2), pos(3), ...
           R(1,2), R(2,2), R(3,2), ...
           scale, 'g', 'LineWidth', 2, 'Parent', axs);
    % Z axis (approach direction)
    quiver3(pos(1), pos(2), pos(3), ...
           R(1,3), R(2,3), R(3,3), ...
           scale, 'b', 'LineWidth', 2, 'Parent', axs);
    
    % Add axis labels
    text(pos(1) + scale*R(1,1), pos(2) + scale*R(2,1), pos(3) + scale*R(3,1), 'x', 'Color', 'r', 'Parent', axs);
    text(pos(1) + scale*R(1,2), pos(2) + scale*R(2,2), pos(3) + scale*R(3,2), 'y', 'Color', 'g', 'Parent', axs);
    text(pos(1) + scale*R(1,3), pos(2) + scale*R(2,3), pos(3) + scale*R(3,3), 'z', 'Color', 'b', 'Parent', axs);
end

view(90, 30); % Set frontal view of the table
fprintf('\n===== CONTINUE TO TRAJECTORY PLANNING =====\n');
fprintf('Press Enter to continue to trajectory planning\n');
pause()

% If processing multiple scenes, offer to run the full processing pipeline
if isMultipleObjects
    fprintf('\n===== FULL PIPELINE OPTION =====\n');
    fprintf('Do you want to run the full processing pipeline for all scenes?\n');
    fprintf('This will process all objects in all scenes and generate trajectories for each\n');
    runFullPipeline = input('Run full pipeline? [y/n]: ', 's');

    if lower(runFullPipeline) == 'y'
        fprintf('\nStarting full pipeline processing...\n');
        
        process_multiple_scenes(dirPath, tableParams, axs);
        
        return;  % End script here if full pipeline was run
    
    else
        fprintf('\nContinuing with single object simulation using first object from first scene...\n');
    
    end

end

% Create trajectories for both robots -------------------------------------------------------------
disp('Planning trajectories for both robots...');
%         /------------/
%        /            /
%       /      R     /
%    + /     |/     /
%   Y /      . -   / world reference frame
%  - /            /
%   /      L     /
%  /            /
% /------------/
%     - X +

% Create viapoints for both robots in world reference frame
ti   = 0;

% First viapoint: set first desired configuration/pose (viapoint) using grasp points
% Create transformation matrices for grasp points
R_grasp_l = squeeze(grasp_orientations(:,:,1));
pf_l = grasp_points(:,1);

% Add a small offset in the approach direction (z-axis of grasp orientation)
% Negative offset to approach from outside the object
approach_offset = 0.05; % 5cm offset for approach
pf_l = pf_l - approach_offset * R_grasp_l(:,3);
T_w_o_l = [R_grasp_l, pf_l; 0,0,0,1];
Tf_l = Trf_0_l \ T_w_o_l;

R_grasp_r = squeeze(grasp_orientations(:,:,2));
pf_r = grasp_points(:,2);
% Add a small offset in the approach direction
pf_r = pf_r - approach_offset * R_grasp_r(:,3);
T_w_o_r = [R_grasp_r, pf_r; 0,0,0,1];
Tf_r = Trf_0_r \ T_w_o_r;

t1   = 1;

% Second viapoint: move to actual grasp positions (no offset)
pf2_l = grasp_points(:,1);
T2_w_o_l = [R_grasp_l, pf2_l; 0,0,0,1];
Tf2_l = Trf_0_l \ T2_w_o_l;

pf2_r = grasp_points(:,2);
T2_w_o_r = [R_grasp_r, pf2_r; 0,0,0,1];
Tf2_r = Trf_0_r \ T2_w_o_r;

t2 = 2;

% Third viapoint: lift object together
pf3_l = grasp_points(:,1) + [0; 0; 0.2]; % Lift 20cm up
T3_w_o_l = [R_grasp_l, pf3_l; 0,0,0,1];
Tf3_l = Trf_0_l \ T3_w_o_l;

pf3_r = grasp_points(:,2) + [0; 0; 0.2]; % Lift 20cm up
T3_w_o_r = [R_grasp_r, pf3_r; 0,0,0,1];
Tf3_r = Trf_0_r \ T3_w_o_r;

t3 = 3;

% Compute multi-viapoint trajectory for selected times and viapoints for both robots --------------
if kinematics == "IK" || space == "joint"
    viapoints_l = [Tf_l; Tf2_l; Tf3_l];
    viapoints_r = [Tf_r; Tf2_r; Tf3_r];
    times       = [2, 2, 2];

    [t_l, p_l, v_l] = multipoint_trajectory( q0_left,  viapoints_l, times, 1 );
    [t_r, p_r, v_r] = multipoint_trajectory( q0_right, viapoints_r, times, 2 );

    [qf, axs] = simulate_dual({robot_left, robot_right}, {config_left, config_right}, ...
                              {Trf_0_l, Trf_0_r}, {t_l, t_r}, {p_l, p_r}, {v_l, v_r}, axs);
elseif kinematics == "IDK"
    % TODO: make the simulation so that even if the two manipulators take different time to execute
    % a trajectory, they will sync on every viapoint
    viapoints_l = [Tf_l];
    viapoints_r = [Tf_r];
    times       = [ti, t1];

    % First: compute the trajectory to the pre-grasp poses (they might take different amount of time 
    %        and we need to make sure that the steps after are completely synchronous)
    [t_l_appr, p_l_appr, v_l_appr] = multipoint_trajectory( q0_left,  viapoints_l, times, 1 );
    [t_r_appr, p_r_appr, v_r_appr] = multipoint_trajectory( q0_right, viapoints_r, times, 2 );

    viapoints_l = [Tf2_l];
    viapoints_r = [Tf2_r];
    t1          = max(t_l_appr(end), t_r_appr(end));
    t2          = t1 + 1;
    times       = [t1, t2];

    % Second: compute trajectory for the following pieces of the task
    [t_l2, p_l2, v_l2] = multipoint_trajectory( p_l_appr(end,:), viapoints_l, times, 1 );
    [t_r2, p_r2, v_r2] = multipoint_trajectory( p_r_appr(end,:), viapoints_r, times, 2 );

    viapoints_l = [Tf3_l];
    viapoints_r = [Tf3_r];
    t2          = max(t_l2(end), t_r2(end));
    t3          = t2 + 1;
    times       = [t2, t3];

    % Third: compute trajectory for the following pieces of the task
    [t_l3, p_l3, v_l3] = multipoint_trajectory( p_l2(end,:), viapoints_l, times, 1 );
    [t_r3, p_r3, v_r3] = multipoint_trajectory( p_r2(end,:), viapoints_r, times, 2 );

    % Run the simulation ------------------------------------------------------------------------------
    [qf, axs] = simulate_dual({robot_left, robot_right}, {config_left, config_right}, ...
                            {Trf_0_l, Trf_0_r}, {p_l_appr, p_r_appr}, {v_l_appr, v_r_appr}, axs);
    [qf, axs] = simulate_dual({robot_left, robot_right}, {config_left, config_right}, ...
                            {Trf_0_l, Trf_0_r}, {p_l2, p_r2}, {v_l2, v_r2}, axs);
    [qf, axs] = simulate_dual({robot_left, robot_right}, {config_left, config_right}, ...
                            {Trf_0_l, Trf_0_r}, {p_l3, p_r3}, {v_l3, v_r3}, axs);
end