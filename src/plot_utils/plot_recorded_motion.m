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
hold on; grid on;

tableParams.height = tableHeight;
tableParams.width  = tableWidth;
tableParams.length = tableLength;
axs = create_environment(tablePosition, tableParams);

% File path
timestamp = '25-06-20-01-59-17';
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
        pcshow(scenePC, 'Parent', axs, 'MarkerSize', 20, 'VerticalAxisDir', 'up');
        
        % Try to segment objects if object pointcloud is available
        if ~isempty(objectPath)
            fprintf('Loading object pointcloud from: %s\n', objectPath);
            objectPC = pcread(objectPath);
            
            % Cluster objects in the scene
            objectPointClouds = cluster_scene_objects(objectPC);
            
            % Display each object with a different color
            colors = {'red', 'green', 'blue', 'cyan', 'magenta', 'yellow'};
            for i = 1:length(objectPointClouds)
                colorIdx = mod(i-1, length(colors)) + 1;
                pcshow(objectPointClouds{i}, colors{colorIdx}, 'Parent', axs, 'MarkerSize', 30, 'VerticalAxisDir', 'up');
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
step = 50; % Downsample for trajectory simulation
fprintf('Loading motion data from %s...\n', motion_file);
[q_l_all, q_r_all, poses_l_all, poses_r_all, use_keyframes, keyframe_indices] = load_motion_data(motion_file, q0_left, q0_right, step);

% Plot trajectories using plot_robot_trajectory function
plot_robot_trajectory(q_l_all, keyframe_indices, 'r-', true, step, 1, axs, 'Left Robot Trajectory',  false);
plot_robot_trajectory(q_r_all, keyframe_indices, 'b-', true, step, 2, axs, 'Right Robot Trajectory', false);

% Plot keyframes with triad visualization
if use_keyframes && ~isempty(keyframe_indices)
    fprintf('Plotting %d keyframes with coordinate frames...\n', length(keyframe_indices));
    
    % Get keyframe joint configurations
    kf_indices_downsampled = floor(keyframe_indices / step) + 1;
    kf_indices_downsampled = min(kf_indices_downsampled, size(q_l_all, 1));
    
    for i = 1:length(kf_indices_downsampled)
        idx = kf_indices_downsampled(i);
        
        % Get world poses for left robot keyframe
        [T_w_e_l, ~] = direct_kinematics(q_l_all(idx,:), 1);
        
        % Get world poses for right robot keyframe
        [T_w_e_r, ~] = direct_kinematics(q_r_all(idx,:), 2);
        
        % Plot keyframe coordinate frames using triad
        triad('Parent', axs, 'Scale', 0.05, 'LineWidth', 2, 'Matrix', T_w_e_l, 'Tag', sprintf('Keyframe %d Left', i));
        triad('Parent', axs, 'Scale', 0.05, 'LineWidth', 2, 'Matrix', T_w_e_r, 'Tag', sprintf('Keyframe %d Right', i));
        
        % Add text labels for keyframes
        text(T_w_e_l(1,4), T_w_e_l(2,4), T_w_e_l(3,4) + 0.05, sprintf('KF %d', i), ...
            'Color', 'r', 'FontSize', 10, 'FontWeight', 'bold', 'Parent', axs);
        text(T_w_e_r(1,4), T_w_e_r(2,4), T_w_e_r(3,4) + 0.05, sprintf('KF %d', i), ...
            'Color', 'b', 'FontSize', 10, 'FontWeight', 'bold', 'Parent', axs);
    end
    
    fprintf('Keyframes plotted successfully\n');
else
    fprintf('No keyframes found or keyframe visualization disabled\n');
end

% Show initial robot configurations
config_left  = set_robot_configuration(q0_left, config_left);
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
legend('show', 'Location', 'best');
title('3D Robot Trajectories with Keyframes', 'FontSize', 14);
xlabel('X (m)'); ylabel('Y (m)'); zlabel('Z (m)');
view(45, 30);
axis equal;

fprintf('3D trajectory visualization completed\n');
