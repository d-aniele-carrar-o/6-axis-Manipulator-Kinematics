% Script to plot 3D robot trajectories with environment, scene objects, and keyframes
clc; close all;

% Setup robots
parameters(0, 1);
robot_left = robot;
config_left = config;
Trf_0_l = Trf_0;

parameters(0, 2);
robot_right = robot;
config_right = config;
Trf_0_r = Trf_0;

% Create environment
figure('Name', '3D Robot Trajectory Visualization', 'Position', [100, 100, 1200, 800]);
hold on; grid on; axis equal;
axs = create_environment();

% File path
% Example timestamp format: '25-06-24-21-30-34'
timestamp = '25-06-24-21-38-38';
% timestamp = '25-06-24-21-30-34';

motion_file = find_closest_motion_file(timestamp);
if isempty(motion_file)
    error('No motion data found for timestamp %s', timestamp);
end

% Try to load scene pointclouds
try
    % Find pointcloud files related to this motion data
    [scenePath, objectPath] = find_pointcloud_files(timestamp);
    
    % Load and display scene pointclouds if found
    if ~isempty(scenePath)
        fprintf('Loading scene pointcloud from: %s\n', scenePath);
        scenePC = pcread(scenePath);
        
        % Transform pointclouds (scene + objects) - translate -2cm on world Y
        translation = [0, -0.025, 0];
        fprintf('Applying pointcloud transformation on world axis: [%.3f, %.3f, %.3f]\n', translation);
        scenePC = pctransform(scenePC, rigid3d(eye(3), translation));
        scenePC.Color = repmat(uint8([128 128 128]), scenePC.Count, 1); % Set scene color to grey        
        pcshow(scenePC, 'Parent', axs, 'MarkerSize', 20, 'VerticalAxisDir', 'up');
        
        % Load all segmented objects from the same directory
        if ~isempty(objectPath)
            [objectDir, ~, ~] = fileparts(objectPath);
            objectFiles = dir(fullfile(objectDir, '*object*.ply'));
            
            fprintf('Found %d object files in directory\n', length(objectFiles));
            
            % RGB color values for different objects
            light_blue = [59, 121, 162];
            brown = [92, 71, 50];
            colors = [light_blue/255; brown/255; 0 0 1; 0 1 1; 1 0 1; 1 1 0; 0.5 0.5 0.5; 1 0.5 0];         

            for i = 1:length(objectFiles)
                objPath = fullfile(objectFiles(i).folder, objectFiles(i).name);
                fprintf('Loading object %d from: %s\n', i, objPath);
                objectPC = pcread(objPath);
                
                % Apply same transformation to object pointclouds
                objectPC = pctransform(objectPC, rigid3d(eye(3), translation));
                
                % Use different color for each object
                colorIdx = mod(i-1, size(colors, 1)) + 1;
                objectPC.Color = repmat(uint8(colors(colorIdx, :) * 255), objectPC.Count, 1);
                pcshow(objectPC, 'Parent', axs, 'MarkerSize', 30, 'VerticalAxisDir', 'up');
            end
        end
    
    else
        fprintf('No scene pointcloud found for timestamp: %s\n', timestamp);
    end
catch e
    fprintf('Error loading pointclouds: %s\n', e.message);
end

% Initial configurations
q0_left  = [pi/2, -pi/3, 2*pi/3, -pi/3, pi/2, 0];
q0_right = [-pi/2, -2*pi/3, -2*pi/3, -2*pi/3, -pi/2, 0];

% Read and compute trajectories in world coordinates
[q_l_all, q_r_all, poses_l_all, poses_r_all, keyframes_data] = load_motion_data(motion_file, q0_left, q0_right);

% Plot trajectories and get handles for simulation
traj_left_handle  = plot_robot_trajectory(q_l_all, keyframes_data, 1, axs, 'r-', true, true);
traj_right_handle = plot_robot_trajectory(q_r_all, keyframes_data, 2, axs, 'b-', true, true);

% Show initial robot configurations
config_left  = set_robot_configuration(q0_left,  config_left);
config_right = set_robot_configuration(q0_right, config_right);
show(robot_left,  config_left,  "Visuals", "on", "Frames", "off", "FastUpdate", true, "PreservePlot", false, "Parent", axs);
show(robot_right, config_right, "Visuals", "on", "Frames", "off", "FastUpdate", true, "PreservePlot", false, "Parent", axs);

% Add text labels for each robot
left = 4; right = 2;  % Robot IDs
left_base_pos  = Trf_0_l(1:3,4);
right_base_pos = Trf_0_r(1:3,4);
text(left_base_pos(1), left_base_pos(2), tableHeight + 0.01, ['Robot Left (', num2str(left), ')'], ...
    'Color', 'r', 'FontSize', 12, 'FontWeight', 'bold', 'HorizontalAlignment', 'center', 'Parent', axs);
text(right_base_pos(1), right_base_pos(2), tableHeight + 0.01, ['Robot Right (', num2str(right), ')'], ...
    'Color', 'b', 'FontSize', 12, 'FontWeight', 'bold', 'HorizontalAlignment', 'center', 'Parent', axs);

% Add labels and formatting
title('3D Robot Trajectories with Keyframes', 'FontSize', 14);
xlabel('X (m)'); ylabel('Y (m)'); zlabel('Z (m)');
view(45, 30);
axis equal;

% Simulate robot trajectories
fprintf('Starting trajectory simulation, press a key to continue...\n');
pause()
simulate_robot_trajectories(robot_left, robot_right, config_left, config_right, ...
                            q_l_all, q_r_all, keyframes_data, axs, traj_left_handle, traj_right_handle);

fprintf('3D trajectory visualization completed\n');
