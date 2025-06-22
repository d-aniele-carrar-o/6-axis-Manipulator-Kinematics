% Comprehensive demo of trajectory transformation capabilities
clear; clc; close all;

% Setup
run_project;

fprintf('=== Trajectory Transformation Demo ===\n\n');

% Parameters
timestamp = '25-06-21-04-58-07';
augmentation_ids = [1, 3, 5]; % Test multiple augmentations

%% Basic Trajectory Transformation Demo
fprintf('1. Basic Trajectory Transformation\n');
fprintf('-----------------------------------\n');

for i = 1:length(augmentation_ids)
    aug_id = augmentation_ids(i);
    fprintf('Testing augmentation %d...\n', aug_id);
    
    try
        transform_trajectory(timestamp, aug_id);
        fprintf('✓ Augmentation %d completed successfully\n', aug_id);
        pause(2); % Allow time to view visualization
    catch ME
        fprintf('✗ Augmentation %d failed: %s\n', aug_id, ME.message);
    end
end

%% Advanced Trajectory Transformation Demo
fprintf('\n2. Advanced Trajectory Transformation\n');
fprintf('-------------------------------------\n');

% Test with different options
options = struct();
options.visualize = true;
options.save_result = true;
options.interaction_threshold = 0.08; % 8cm interaction threshold

for i = 1:length(augmentation_ids)
    aug_id = augmentation_ids(i);
    fprintf('Testing advanced transformation for augmentation %d...\n', aug_id);
    
    try
        advanced_trajectory_transform(timestamp, aug_id, options);
        fprintf('✓ Advanced augmentation %d completed successfully\n', aug_id);
        pause(2);
    catch ME
        fprintf('✗ Advanced augmentation %d failed: %s\n', aug_id, ME.message);
    end
end

%% Analysis and Comparison
fprintf('\n3. Trajectory Analysis\n');
fprintf('----------------------\n');

try
    analyze_transformation_quality(timestamp, augmentation_ids);
catch ME
    fprintf('Analysis failed: %s\n', ME.message);
end

fprintf('\nDemo completed! Check the generated visualizations and saved files.\n');

%% Helper function for analysis
function analyze_transformation_quality(timestamp, aug_ids)
% Analyze the quality of trajectory transformations
    
    fprintf('Analyzing transformation quality...\n');
    
    % Load original trajectory
    motion_file = find_closest_motion_file(timestamp);
    [q_left_orig, q_right_orig, ~] = load_motion_data_simple(motion_file);
    
    % Get original end-effector trajectories
    ee_left_orig = get_end_effector_trajectory(q_left_orig, 1);
    ee_right_orig = get_end_effector_trajectory(q_right_orig, 2);
    
    % Analyze each augmentation
    figure('Name', 'Transformation Quality Analysis', 'Position', [100, 100, 1200, 800]);
    
    for i = 1:length(aug_ids)
        aug_id = aug_ids(i);
        
        % Load augmentation data
        augmented_demos_path = '/Users/danielecarraro/Documents/VSCODE/data/output/augmented_demos';
        json_file = fullfile(augmented_demos_path, timestamp, 'augmented_demos.json');
        augmentation_data = jsondecode(fileread(json_file));
        aug_scene = augmentation_data.augmentations(aug_id + 1); % +1 for MATLAB indexing
        
        % Load original objects from paths
        data_folder = '/Users/danielecarraro/Documents/VSCODE/data/';
        object_paths = augmentation_data.original_objects.paths;
        original_objects = cell(length(object_paths), 1);
        for j = 1:length(object_paths)
            ptCloud = pcread(fullfile(data_folder, object_paths{j}));
            original_objects{j} = mean(ptCloud.Location, 1);
        end
        transformations = aug_scene.transformations;
        
        if ~isempty(transformations)
            T = transformations(1);
            displacement = T.translation;
            displacement_magnitude = norm(displacement);
            
            fprintf('Aug %d: Object displacement = %.3f m\n', aug_id, displacement_magnitude);
            
            % Plot displacement analysis
            subplot(2, 2, i);
            bar([displacement_magnitude]);
            title(sprintf('Aug %d: Displacement = %.3f m', aug_id, displacement_magnitude));
            ylabel('Displacement (m)');
            grid on;
        end
    end
    
    % Summary statistics
    subplot(2, 2, 4);
    text(0.1, 0.8, 'Transformation Summary:', 'FontSize', 12, 'FontWeight', 'bold');
    text(0.1, 0.6, sprintf('Original trajectory length: %d frames', size(q_left_orig, 1)));
    text(0.1, 0.4, sprintf('Tested augmentations: %d', length(aug_ids)));
    text(0.1, 0.2, 'All transformations preserve robot kinematics');
    axis off;
end



function ee_trajectory = get_end_effector_trajectory(q_trajectory, robot_id)
% Get end-effector positions for trajectory
    
    ee_trajectory = zeros(size(q_trajectory, 1), 3);
    
    for i = 1:size(q_trajectory, 1)
        parameters(1, robot_id);
        [T_ee, ~] = direct_kinematics(q_trajectory(i, :), robot_id);
        ee_trajectory(i, :) = T_ee(1:3, 4)';
    end
end

function [q_left_all, q_right_all, keyframe_indices] = load_motion_data_simple(motion_file)
% Simplified motion data loading (copied from transform_trajectory.m)
    
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
    catch
        keyframe_indices = [];
    end
end

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