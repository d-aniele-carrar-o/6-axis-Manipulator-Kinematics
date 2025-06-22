function transform_trajectory(timestamp, augmentation_id)
% TRANSFORM_TRAJECTORY Transform recorded robot trajectories based on scene augmentation
%
% Inputs:
%   timestamp - Original scene timestamp (e.g., '25-06-21-04-58-07')
%   augmentation_id - ID of the augmentation to apply (1-10)

if nargin < 2
    augmentation_id = 1;
end

% Load augmentation data
augmented_demos_path = '/Users/danielecarraro/Documents/VSCODE/data/output/augmented_demos';
json_file = fullfile(augmented_demos_path, timestamp, 'augmented_demos.json');

if ~exist(json_file, 'file')
    error('Augmentation file not found: %s', json_file);
end

fprintf('Loading augmentation data from: %s\n', json_file);
augmentation_data = jsondecode(fileread(json_file));

% Get specific augmentation
if augmentation_id >= length(augmentation_data.augmentations)
    error('Augmentation ID %d not found. Available: 0-%d', augmentation_id, length(augmentation_data.augmentations)-1);
end

aug_scene = augmentation_data.augmentations(augmentation_id + 1); % +1 for MATLAB indexing
transformations = aug_scene.transformations;

fprintf('Applying augmentation %d with %d object transformations\n', ...
    augmentation_id, length(transformations));

% Load original motion data
motion_file = find_closest_motion_file(timestamp);
if isempty(motion_file)
    error('No motion data found for timestamp %s', timestamp);
end

fprintf('Loading motion data from: %s\n', motion_file);
[q_left_orig, q_right_orig, keyframe_indices] = load_motion_data_simple(motion_file);

% Load original scene for reference
[scenePcdPath, objectPcdPath] = find_pointcloud_files(timestamp);
ptCloudObject = pcread(objectPcdPath);

% Apply camera calibration if needed
table_z_median = median(ptCloudObject.Location(:,3));
if abs(table_z_median - 0.78) > 0.1  % Assuming table height ~0.78m
    tform_cam_to_world = load_camera_calibration();
    ptCloudObject = pctransform(ptCloudObject, tform_cam_to_world);
end

% Load original objects from paths
data_folder = '/Users/danielecarraro/Documents/VSCODE/data/';
object_paths = augmentation_data.original_objects.paths;
ptCloudObject = pcread(fullfile(data_folder, object_paths{1}));
object_center = mean(ptCloudObject.Location, 1);
fprintf('Original object center: [%.3f, %.3f, %.3f]\n', object_center);

% Apply transformation to object center
T = transformations(1);  % Assuming single object for now
transformed_center = apply_transformation(object_center, T);
fprintf('Transformed object center: [%.3f, %.3f, %.3f]\n', transformed_center);

% Transform trajectory
[q_left_new, q_right_new] = transform_robot_trajectory(...
    q_left_orig, q_right_orig, object_center, transformed_center);

% Visualize results
visualize_transformed_trajectory(timestamp, augmentation_id, ...
    q_left_orig, q_right_orig, q_left_new, q_right_new, ...
    object_center, transformed_center, keyframe_indices);

end

function transformed_point = apply_transformation(point, T)
% Apply transformation to a 3D point
    % Translation
    transformed_point = point + T.translation';
    
    % Rotation (around Z-axis)
    if T.rotation_angle ~= 0
        angle_rad = deg2rad(T.rotation_angle);
        R = [cos(angle_rad), -sin(angle_rad), 0;
             sin(angle_rad),  cos(angle_rad), 0;
             0,               0,              1];
        transformed_point = (R * transformed_point')';
    end
    
    % Scale
    if T.scale_factor ~= 1
        transformed_point = transformed_point * T.scale_factor;
    end
end

function [q_left_new, q_right_new] = transform_robot_trajectory(...
    q_left_orig, q_right_orig, orig_center, new_center)
% Transform robot joint trajectories based on object displacement

    % Calculate displacement vector
    displacement = new_center - orig_center;
    fprintf('Object displacement: [%.3f, %.3f, %.3f]\n', displacement);
    
    % Initialize transformed trajectories
    q_left_new = q_left_orig;
    q_right_new = q_right_orig;
    
    % Robot parameters
    parameters(1, 1); % Left robot
    [~, Te_l] = direct_kinematics(q_left_orig(1,:), 1);
    parameters(1, 2); % Right robot  
    [~, Te_r] = direct_kinematics(q_right_orig(1,:), 2);
    
    % Transform each trajectory point
    for i = 1:size(q_left_orig, 1)
        % Get current end-effector poses
        parameters(1, 1);
        [Te_w_e_left, ~] = direct_kinematics(q_left_orig(i,:), 1);
        parameters(1, 2);
        [Te_w_e_right, ~] = direct_kinematics(q_right_orig(i,:), 2);
        
        % Apply displacement to end-effector positions
        Te_w_e_left_new = Te_w_e_left;
        Te_w_e_right_new = Te_w_e_right;
        Te_w_e_left_new(1:3, 4) = Te_w_e_left_new(1:3, 4) + displacement';
        Te_w_e_right_new(1:3, 4) = Te_w_e_right_new(1:3, 4) + displacement';
        
        % Convert to robot base frames
        Tf_l_new = Te_l \ Te_w_e_left_new;
        Tf_r_new = Te_r \ Te_w_e_right_new;
        
        % Solve inverse kinematics
        try
            parameters(1, 1);
            Hl = UR5_inverse_kinematics_cpp(Tf_l_new(1:3,4), Tf_l_new(1:3,1:3), AL, A, D);
            q_left_new(i,:) = get_closer(Hl, q_left_orig(i,:));
            
            parameters(1, 2);
            Hr = UR5_inverse_kinematics_cpp(Tf_r_new(1:3,4), Tf_r_new(1:3,1:3), AL, A, D);
            q_right_new(i,:) = get_closer(Hr, q_right_orig(i,:));
        catch
            % Keep original configuration if IK fails
            fprintf('IK failed at frame %d, keeping original\n', i);
        end
    end
end

function visualize_transformed_trajectory(timestamp, aug_id, ...
    q_left_orig, q_right_orig, q_left_new, q_right_new, ...
    orig_center, new_center, keyframe_indices)
% Visualize original and transformed trajectories

    % Setup robots
    parameters(0, 1);
    robot_left = robot; config_left = config; Trf_0_l = Trf_0;
    parameters(0, 2);
    robot_right = robot; config_right = config; Trf_0_r = Trf_0;
    
    % Load augmented scene
    augmented_demos_path = '/Users/danielecarraro/Documents/VSCODE/data/output/augmented_demos';
    aug_scene_file = fullfile(augmented_demos_path, timestamp, 'scene_visualizations', ...
        sprintf('augmented_scene_%d.ply', aug_id));
    
    figure('Name', sprintf('Trajectory Transformation - Aug %d', aug_id), 'Position', [100, 100, 1400, 600]);
    
    % Original trajectory
    subplot(1, 2, 1);
    hold on; grid on; axis equal;
    title('Original Trajectory');
    
    % Environment
    tableParams.height = 0.78; tableParams.width = 1.2; tableParams.length = 0.8;
    create_environment([0, 0, 0], tableParams);
    
    % Original object
    scatter3(orig_center(1), orig_center(2), orig_center(3), 200, 'ro', 'filled');
    
    % Robot trajectories
    plot_robot_trajectory(robot_left, config_left, q_left_orig, keyframe_indices, 'b-');
    plot_robot_trajectory(robot_right, config_right, q_right_orig, keyframe_indices, 'r-');
    
    % Transformed trajectory
    subplot(1, 2, 2);
    hold on; grid on; axis equal;
    title(sprintf('Transformed Trajectory (Aug %d)', aug_id));
    
    % Environment
    create_environment([0, 0, 0], tableParams);
    
    % Load and show augmented scene if available
    if exist(aug_scene_file, 'file')
        ptCloudAug = pcread(aug_scene_file);
        pcshow(ptCloudAug.Location, [0.8 0.4 0.4], 'MarkerSize', 20);
    else
        % Show transformed object position
        scatter3(new_center(1), new_center(2), new_center(3), 200, 'go', 'filled');
    end
    
    % Robot trajectories
    plot_robot_trajectory(robot_left, config_left, q_left_new, keyframe_indices, 'b-');
    plot_robot_trajectory(robot_right, config_right, q_right_new, keyframe_indices, 'r-');
    
    % Set consistent view
    for i = 1:2
        subplot(1, 2, i);
        xlim([-0.8, 0.8]); ylim([-0.8, 0.8]); zlim([0, 1.5]);
        view(45, 30);
    end
    
    fprintf('Visualization complete. Original center: [%.3f, %.3f, %.3f], New center: [%.3f, %.3f, %.3f]\n', ...
        orig_center, new_center);
end

function plot_robot_trajectory(robot, config, q_trajectory, keyframe_indices, line_style)
% Plot robot end-effector trajectory
    
    ee_positions = zeros(size(q_trajectory, 1), 3);
    
    for i = 1:size(q_trajectory, 1)
        config = set_robot_configuration(q_trajectory(i,:), config);
        T = getTransform(robot, config, 'wrist_3_link');
        ee_positions(i, :) = T(1:3, 4)';
    end
    
    % Plot trajectory
    plot3(ee_positions(:,1), ee_positions(:,2), ee_positions(:,3), line_style, 'LineWidth', 2);
    
    % Highlight keyframes if available
    if ~isempty(keyframe_indices)
        step = 50; % Assuming downsampling factor
        for i = 1:length(keyframe_indices)
            idx = min(floor(keyframe_indices(i) / step) + 1, size(ee_positions, 1));
            scatter3(ee_positions(idx,1), ee_positions(idx,2), ee_positions(idx,3), ...
                100, 'k*', 'LineWidth', 2);
        end
    end
end

function [q_left_all, q_right_all, keyframe_indices] = load_motion_data_simple(motion_file)
% Simplified motion data loading
    
    data = readtable(motion_file);
    
    % Robot mappings
    left = 4; right = 2;
    
    % Create column mappings
    robot_cols = struct();
    for i = 1:4
        robot_cols(i).x = sprintf('x%d', i);
        robot_cols(i).y = sprintf('y%d', i);
        robot_cols(i).z = sprintf('z%d', i);
        robot_cols(i).rx = sprintf('rx%d', i);
        robot_cols(i).ry = sprintf('ry%d', i);
        robot_cols(i).rz = sprintf('rz%d', i);
    end
    
    % Downsample data
    step = 50;
    num_frames = floor(height(data)/step);
    
    q_left_all = zeros(num_frames, 6);
    q_right_all = zeros(num_frames, 6);
    
    % Initial configurations
    q0_left  = [pi/2, -pi/3, 2*pi/3, -pi/3, pi/2, 0];
    q0_right = [-pi/2, -2*pi/3, -2*pi/3, -2*pi/3, -pi/2, 0];
    
    q_left = q0_left;
    q_right = q0_right;
    
    % Get initial end-effector poses
    parameters(1, 1); [~, Te_l] = direct_kinematics(q0_left, 1);
    parameters(1, 2); [~, Te_r] = direct_kinematics(q0_right, 2);
    
    % Compute joint configurations
    for i = 2:num_frames
        idx = (i-1)*step + 1;
        
        % Left robot
        pos_rel_l = [data.(robot_cols(left).x)(idx); data.(robot_cols(left).y)(idx); data.(robot_cols(left).z)(idx)];
        pos_left = Te_l(1:3,4) + pos_rel_l;
        left_vec = [data.(robot_cols(left).rx)(idx), -data.(robot_cols(left).rz)(idx), data.(robot_cols(left).ry)(idx)];
        left_angle = norm(left_vec);
        if left_angle > 0
            rot_left = Te_l(1:3,1:3) * axang2rotm([left_vec/left_angle, left_angle]);
        else
            rot_left = Te_l(1:3,1:3);
        end
        
        % Right robot
        pos_rel_r = [data.(robot_cols(right).x)(idx); data.(robot_cols(right).y)(idx); data.(robot_cols(right).z)(idx)];
        pos_right = Te_r(1:3,4) + pos_rel_r;
        right_vec = [data.(robot_cols(right).rx)(idx), data.(robot_cols(right).rz)(idx), -data.(robot_cols(right).ry)(idx)];
        right_angle = norm(right_vec);
        if right_angle > 0
            rot_right = Te_r(1:3,1:3) * axang2rotm([right_vec/right_angle, right_angle]);
        else
            rot_right = Te_r(1:3,1:3);
        end
        
        % Inverse kinematics
        parameters(1, 1);
        Hl = UR5_inverse_kinematics_cpp(pos_left, rot_left, AL, A, D);
        parameters(1, 2);
        Hr = UR5_inverse_kinematics_cpp(pos_right, rot_right, AL, A, D);
        q_left = get_closer(Hl, q_left);
        q_right = get_closer(Hr, q_right);
        
        q_left_all(i,:) = q_left;
        q_right_all(i,:) = q_right;
    end
    
    % Load keyframes if available
    [filepath, filename, ~] = fileparts(motion_file);
    keyframe_file = fullfile(filepath, [filename, '_keyframes.csv']);
    
    try
        keyframes = readtable(keyframe_file);
        keyframe_indices = keyframes.original_index;
        fprintf('Loaded %d keyframes\n', length(keyframe_indices));
    catch
        keyframe_indices = [];
        fprintf('No keyframes found\n');
    end
end

% Helper functions from dual_robot_setup_simple.m
function motion_file = find_closest_motion_file(target_timestamp)
    data_folder = '/Users/danielecarraro/Documents/VSCODE/data/';
    files = dir(fullfile(data_folder, '**', '*_motion*'));
    
    if isempty(files)
        motion_file = '';
        return;
    end
    
    target_time = parse_timestamp(target_timestamp);
    if isnan(target_time)
        motion_file = '';
        return;
    end
    
    best_diff = inf;
    motion_file = '';
    for i = 1:length(files)
        file_timestamp = extract_timestamp_from_filename(files(i).name);
        if ~isempty(file_timestamp)
            file_time = parse_timestamp(file_timestamp);
            if ~isnan(file_time)
                diff = abs(file_time - target_time);
                if diff < best_diff
                    best_diff = diff;
                    motion_file = fullfile(files(i).folder, files(i).name);
                end
            end
        end
    end
end

function timestamp_str = extract_timestamp_from_filename(filename)
    pattern = '\d{2}-\d{2}-\d{2}-\d{2}-\d{2}';
    match = regexp(filename, pattern, 'match');
    if ~isempty(match)
        timestamp_str = [match{1}, '-00'];
    else
        timestamp_str = '';
    end
end

function time_val = parse_timestamp(timestamp_str)
    try
        dt = datetime(timestamp_str, 'InputFormat', 'yy-MM-dd-HH-mm-ss');
        time_val = posixtime(dt);
    catch
        time_val = NaN;
    end
end

function [scenePath, objectPath] = find_pointcloud_files(timestamp)
    data_folder = '/Users/danielecarraro/Documents/VSCODE/data/';
    
    scene_files = dir(fullfile(data_folder, '**', '*table*.ply'));
    object_files = dir(fullfile(data_folder, '**', '*object*.ply'));
    
    scenePath = '';
    objectPath = '';
    
    for i = 1:length(scene_files)
        if contains(scene_files(i).name, timestamp) || contains(scene_files(i).folder, timestamp)
            scenePath = fullfile(scene_files(i).folder, scene_files(i).name);
            break;
        end
    end
    
    for i = 1:length(object_files)
        if contains(object_files(i).name, timestamp) || contains(object_files(i).folder, timestamp)
            objectPath = fullfile(object_files(i).folder, object_files(i).name);
            break;
        end
    end
end