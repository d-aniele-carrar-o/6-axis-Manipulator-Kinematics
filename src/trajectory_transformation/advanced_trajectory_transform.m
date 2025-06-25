function advanced_trajectory_transform(timestamp, augmentation_id, options)
% ADVANCED_TRAJECTORY_TRANSFORM Advanced trajectory transformation with multiple objects
%
% Inputs:
%   timestamp - Original scene timestamp
%   augmentation_id - ID of the augmentation to apply
%   options - Structure with optional parameters:
%     .visualize - Show visualization (default: true)
%     .save_result - Save transformed trajectory (default: false)
%     .interaction_threshold - Distance threshold for object interaction (default: 0.1m)

if nargin < 3
    options = struct();
end

% Default options
if ~isfield(options, 'visualize'), options.visualize = true; end
if ~isfield(options, 'save_result'), options.save_result = false; end
if ~isfield(options, 'simulate'), options.simulate = false; end

% Load augmentation data
parameters(1);
json_file = fullfile(augmented_demos_path, timestamp, 'augmented_demos.json');

fprintf('Loading augmentation data...\n');
augmentation_data = jsondecode(fileread(json_file));
aug_scene = augmentation_data.augmentations(augmentation_id + 1); % +1 for MATLAB indexing

% Load original objects from paths
parameters(1);
object_paths = augmentation_data.original_objects.paths;
original_objects = cell(length(object_paths), 1);
for i = 1:length(object_paths)
    ptCloud = pcread(fullfile(data_folder, object_paths{i}));
    original_objects{i} = mean(ptCloud.Location, 1);
end
transformations = aug_scene.transformations;

fprintf('Found %d objects with %d transformations\n', ...
    length(original_objects), length(transformations));
fprintf('Debug: transformations structure has %d elements\n', length(aug_scene.transformations));
for i = 1:min(3, length(transformations))
    fprintf('  Transform %d: [%.3f, %.3f, %.3f], rot=%.1f°, scale=%.2f\n', ...
        i, transformations(i).translation, transformations(i).rotation_angle, transformations(i).scale_factor);
end

% Apply transformations to get new object positions
transformed_objects = cell(size(original_objects));
for i = 1:length(original_objects)
    if i <= length(transformations)
        T = transformations(i);
        transformed_point = original_objects{i} + T.translation';
        if T.rotation_angle ~= 0
            angle_rad = deg2rad(T.rotation_angle);
            R = [cos(angle_rad), -sin(angle_rad), 0;
                 sin(angle_rad),  cos(angle_rad), 0;
                 0,               0,              1];
            transformed_point = (R * transformed_point')';
        end
        if T.scale_factor ~= 1
            transformed_point = transformed_point * T.scale_factor;
        end
        transformed_objects{i} = transformed_point;
    else
        transformed_objects{i} = original_objects{i}; % No transformation
    end
end

% Load and analyze trajectory
motion_file = find_closest_motion_file(timestamp);
if isempty(motion_file)
    error('No motion data found for timestamp %s', timestamp);
end
[q_left_orig, q_right_orig, poses_left_all, poses_right_all, keyframe_indices] = load_motion_data_simple(motion_file);

% Analyze trajectory for object interactions
interaction_points = analyze_trajectory_interactions(motion_file, transformations);

% Transform trajectory based on interactions
[q_left_new, q_right_new] = transform_trajectory_advanced(q_left_orig, q_right_orig, interaction_points);

% Visualize if requested
if options.visualize
    visualize_advanced_transformation(timestamp, augmentation_id, ...
        q_left_orig, q_right_orig, q_left_new, q_right_new, ...
        original_objects, transformed_objects, interaction_points, keyframe_indices);
end

% Save results if requested
if options.save_result
    save_transformed_trajectory(timestamp, augmentation_id, ...
        q_left_new, q_right_new, keyframe_indices);
end

% Simulate if requested
if options.simulate
    disp("Press enter to proceed with the simulation...")
    pause()
    simulate_transformed_trajectory(timestamp, augmentation_id, ...
        q_left_new, q_right_new, keyframe_indices, ...
        data_folder, object_paths, transformations);
end

fprintf('Advanced trajectory transformation completed!\n');

end



function interaction_points = analyze_trajectory_interactions(motion_file, transformations)
% Read keyframes data to find object interaction points
    
    % Load keyframes data
    [filepath, filename, ~] = fileparts(motion_file);
    keyframe_file = fullfile(filepath, [filename, '_keyframes.csv']);
    
    if ~exist(keyframe_file, 'file')
        error('Keyframes file not found: %s', keyframe_file);
    end
    
    fprintf('Loading keyframes from: %s\n', keyframe_file);
    keyframes = readtable(keyframe_file);
    
    % Debug: show what methods are actually in the file
    fprintf('Available extraction methods in keyframes:\n');
    unique_methods = unique(keyframes.extraction_method);
    for i = 1:length(unique_methods)
        fprintf('  - %s\n', unique_methods{i});
    end
    
    % Filter for interaction keyframes (grasp/ungrasp methods)
    interaction_methods = {'pre_grasp_r2', 'grasp_r2', 'pre_ungrasp_r2', 'ungrasp_r2'};
    interaction_mask = ismember(keyframes.extraction_method, interaction_methods);
    interaction_keyframes = keyframes(interaction_mask, :);
    
    fprintf('Found %d interaction keyframes\n', height(interaction_keyframes));
    
    % Create interaction points structure
    interaction_points = struct();
    interaction_points.keyframes = interaction_keyframes;
    interaction_points.transformations = transformations;
    interaction_points.original_indices = interaction_keyframes.original_index;
    
    % Group by object (assuming alternating left/right robot interactions)
    num_objects = length(transformations);
    interaction_points.objects = [];
    interaction_points.left = [];
    interaction_points.right = [];
    
    for i = 1:height(interaction_keyframes)
        obj_idx = mod(i-1, num_objects) + 1; % Cycle through objects
        interaction_points.objects = [interaction_points.objects; obj_idx];
        
        % Alternate between left and right robot (or use robot ID from keyframes if available)
        if mod(i, 2) == 1
            interaction_points.left = [interaction_points.left; i];
        else
            interaction_points.right = [interaction_points.right; i];
        end
    end
    
    fprintf('Organized %d interaction points across %d objects\n', ...
        height(interaction_keyframes), num_objects);
end

function ee_trajectory = get_end_effector_trajectory(q_trajectory, robot_id)
% Get end-effector positions for trajectory
    
    ee_trajectory = zeros(size(q_trajectory, 1), 3);
    
    for i = 1:size(q_trajectory, 1)
        parameters(1, robot_id);
        [T_ee, ~] = direct_kinematics(q_trajectory(i, :), robot_id);
        ee_trajectory(i,:) = T_ee(1:3, 4)';
    end
end

function [q_left_new, q_right_new] = transform_trajectory_advanced(q_left_orig, q_right_orig, interaction_points)
% Advanced trajectory transformation using keyframe-based interactions
    
    fprintf('Transforming trajectory using %d interaction keyframes\n', height(interaction_points.keyframes));
    
    % Get robot base transformations
    parameters(1, 1); [~, Te_l] = direct_kinematics(q_left_orig(1,:), 1);
    parameters(1, 2); [~, Te_r] = direct_kinematics(q_right_orig(1,:), 2);
    
    % Compute transformed poses for keyframes
    keyframe_poses_left = [];
    keyframe_poses_right = [];
    keyframe_indices = [];
    
    for i = 1:height(interaction_points.keyframes)
        original_idx = interaction_points.original_indices(i);
        obj_idx = interaction_points.objects(i);
        transformation = interaction_points.transformations(obj_idx);
        
        % Convert original index to downsampled trajectory index
        step = 50;
        traj_idx = min(floor(original_idx / step) + 1, size(q_left_orig, 1));
        keyframe_indices = [keyframe_indices; traj_idx];
        
        % Get original end-effector poses at this keyframe
        parameters(1, 1); [Te_w_e_left_orig, ~] = direct_kinematics(q_left_orig(traj_idx,:), 1);
        parameters(1, 2); [Te_w_e_right_orig, ~] = direct_kinematics(q_right_orig(traj_idx,:), 2);
        
        % Apply object transformation to end-effector poses
        displacement = transformation.translation(:);
        Te_w_e_left_new = Te_w_e_left_orig;
        Te_w_e_right_new = Te_w_e_right_orig;
        Te_w_e_left_new(1:3, 4) = Te_w_e_left_new(1:3, 4) + displacement;
        Te_w_e_right_new(1:3, 4) = Te_w_e_right_new(1:3, 4) + displacement;
        
        % Convert to robot base frames
        Tf_left = Te_l \ Te_w_e_left_new;
        Tf_right = Te_r \ Te_w_e_right_new;
        
        keyframe_poses_left = [keyframe_poses_left; Tf_left];
        keyframe_poses_right = [keyframe_poses_right; Tf_right];
        
        fprintf('  Keyframe %d: original_idx=%d, traj_idx=%d, obj=%d, disp=[%.3f,%.3f,%.3f]\n', ...
            i, original_idx, traj_idx, obj_idx, displacement);
    end
    
    % Generate new trajectory using multipoint_trajectory like in dual_robot_setup_simple
    if ~isempty(keyframe_poses_left)
        times = ones(1, size(keyframe_poses_left, 1)) * 2; % 2 seconds between keyframes
        
        try
            [~, q_left_new, ~] = multipoint_trajectory(q_left_orig(1,:), keyframe_poses_left, times, 1);
            [~, q_right_new, ~] = multipoint_trajectory(q_right_orig(1,:), keyframe_poses_right, times, 2);
            fprintf('Generated new trajectory with %d frames\n', size(q_left_new, 1));
        catch ME
            fprintf('Trajectory generation failed: %s\n', ME.message);
            fprintf('Falling back to original trajectory\n');
            q_left_new = q_left_orig;
            q_right_new = q_right_orig;
        end
    else
        q_left_new = q_left_orig;
        q_right_new = q_right_orig;
    end
end

function q_new = apply_transformation_from_frame(q_orig, displacement, start_frame, robot_id)
% Apply displacement to trajectory starting from a specific frame
    
    q_new = q_orig;
    
    % Get robot base transformation
    parameters(1, robot_id);
    
    % Apply displacement from start_frame onwards
    for i = start_frame:size(q_orig, 1)
        % Get current end-effector pose
        [T_w_ee, ~] = direct_kinematics(q_orig(i,:), robot_id);
        
        % Apply displacement
        T_w_ee_new = T_w_ee;
        T_w_ee_new(1:3, 4) = T_w_ee_new(1:3, 4) + displacement;
        
        % Convert to robot base frame
        Tf_new = Trf_0 \ T_w_ee_new;
        
        % Solve inverse kinematics
        try
            parameters(1, robot_id);
            H = UR5_inverse_kinematics_cpp(Tf_new(1:3,4), Tf_new(1:3,1:3), AL, A, D);
            q_new(i,:) = get_closer(H, q_orig(i,:));
        catch
            % Keep original if IK fails
            fprintf('IK failed at frame %d for robot %d\n', i, robot_id);
        end
    end
end

function visualize_advanced_transformation(timestamp, aug_id, ...
    q_left_orig, q_right_orig, q_left_new, q_right_new, ...
    orig_objects, new_objects, interaction_points, keyframe_indices)
% Advanced visualization with multiple objects and interaction points
    
    figure('Name', sprintf('Advanced Trajectory Transform - Aug %d', aug_id), ...
        'Position', [50, 50, 1600, 800]);
    
    % Setup robots
    parameters(0, 1);
    robot_left = robot; config_left = config;
    parameters(0, 2);
    robot_right = robot; config_right = config;
    
    % Original trajectory
    subplot(1, 2, 1);
    hold on; grid on; axis equal;
    title('Original Trajectory & Objects');
    
    % Environment
    tableParams.height = 0.78; tableParams.width = 1.2; tableParams.length = 0.8;
    create_environment([0, 0, 0], tableParams);
    
    % Original objects
    colors = lines(length(orig_objects));
    for i = 1:length(orig_objects)
        scatter3(orig_objects{i}(1), orig_objects{i}(2), orig_objects{i}(3), ...
            150, colors(i,:), 'o', 'filled');
    end
    
    % Get keyframe names from interaction points if available
    keyframe_names = {};
    if ~isempty(interaction_points) && isfield(interaction_points, 'keyframes')
        keyframe_names = cellstr(interaction_points.keyframes.extraction_method);
    end
    
    % Robot trajectories
    plot_robot_trajectory_advanced(robot_left, config_left, q_left_orig, keyframe_indices, 'b-', keyframe_names);
    plot_robot_trajectory_advanced(robot_right, config_right, q_right_orig, keyframe_indices, 'r-', keyframe_names);
    
    % Highlight interaction points
    ee_left = get_end_effector_trajectory(q_left_orig, 1);
    ee_right = get_end_effector_trajectory(q_right_orig, 2);
    
    for i = 1:length(interaction_points.left)
        idx = interaction_points.left(i);
        scatter3(ee_left(idx,1), ee_left(idx,2), ee_left(idx,3), ...
            100, 'b', 's', 'filled', 'LineWidth', 2);
    end
    
    for i = 1:length(interaction_points.right)
        idx = interaction_points.right(i);
        scatter3(ee_right(idx,1), ee_right(idx,2), ee_right(idx,3), ...
            100, 'r', 's', 'filled', 'LineWidth', 2);
    end
    
    % Transformed trajectory
    subplot(1, 2, 2);
    hold on; grid on; axis equal;
    title(sprintf('Transformed Trajectory (Aug %d)', aug_id));
    
    % Environment
    create_environment([0, 0, 0], tableParams);
    
    % Transformed objects
    for i = 1:length(new_objects)
        scatter3(new_objects{i}(1), new_objects{i}(2), new_objects{i}(3), ...
            150, colors(i,:), 'o', 'filled');
        
        % Draw displacement vectors
        if i <= length(orig_objects)
            quiver3(orig_objects{i}(1), orig_objects{i}(2), orig_objects{i}(3), ...
                new_objects{i}(1) - orig_objects{i}(1), ...
                new_objects{i}(2) - orig_objects{i}(2), ...
                new_objects{i}(3) - orig_objects{i}(3), ...
                0, 'k-', 'LineWidth', 2, 'MaxHeadSize', 0.5);
        end
    end
    
    % Robot trajectories
    plot_robot_trajectory_advanced(robot_left, config_left, q_left_new, keyframe_indices, 'b-', keyframe_names);
    plot_robot_trajectory_advanced(robot_right, config_right, q_right_new, keyframe_indices, 'r-', keyframe_names);
    
    % Set consistent view
    for i = 1:2
        subplot(1, 2, i);
        xlim([-0.8, 0.8]); ylim([-0.8, 0.8]); zlim([0, 1.5]);
        view(45, 30);
        % No legend to avoid clutter
    end
end

function save_transformed_trajectory(timestamp, aug_id, q_left, q_right, keyframes)
% Save transformed trajectory to file
    
    output_dir = fullfile('/Users/danielecarraro/Documents/VSCODE/data/output', 'transformed_trajectories');
    if ~exist(output_dir, 'dir')
        mkdir(output_dir);
    end
    
    filename = sprintf('transformed_traj_%s_aug%d.mat', timestamp, aug_id);
    filepath = fullfile(output_dir, filename);
    
    save(filepath, 'q_left', 'q_right', 'keyframes', 'timestamp', 'aug_id');
    fprintf('Transformed trajectory saved to: %s\n', filepath);
end

function plot_robot_trajectory_advanced(robot, config, q_trajectory, keyframe_indices, line_style, keyframe_names)
% Plot robot end-effector trajectory with enhanced keyframe visualization
    
    % Default parameters
    if nargin < 6, keyframe_names = {}; end
    
    ee_positions = zeros(size(q_trajectory, 1), 3);
    ee_rotations = zeros(size(q_trajectory, 1), 3, 3);
    
    for i = 1:size(q_trajectory, 1)
        config = set_robot_configuration(q_trajectory(i,:), config);
        T = getTransform(robot, config, 'wrist_3_link');
        ee_positions(i, :) = T(1:3, 4)';
        ee_rotations(i, :, :) = T(1:3, 1:3);
    end
    
    % Plot trajectory
    plot3(ee_positions(:,1), ee_positions(:,2), ee_positions(:,3), line_style, 'LineWidth', 2);
    
    % Highlight keyframes if available
    if ~isempty(keyframe_indices)
        step = 50; % Assuming downsampling factor
        for i = 1:length(keyframe_indices)
            idx = min(floor(keyframe_indices(i) / step) + 1, size(ee_positions, 1));
            
            % Get position
            pos = ee_positions(idx,:)';
            
            % Add marker
            scatter3(pos(1), pos(2), pos(3), 100, 'k*', 'LineWidth', 2);
            
            % Add text label with background
            color = line_style(1); % Use first character of line_style as color
            if ~isempty(keyframe_names) && i <= length(keyframe_names)
                label_text = keyframe_names{i};
            else
                % Use extraction method if available in interaction_points
                if exist('interaction_points', 'var') && isfield(interaction_points, 'keyframes') && i <= height(interaction_points.keyframes)
                    label_text = char(interaction_points.keyframes.extraction_method(i));
                else
                    label_text = sprintf('KF %d', i);
                end
            end
            
            % Create text with white background and colored border
            text(pos(1), pos(2), pos(3) + 0.08, label_text, ...
                'Color', color, 'FontSize', 10, 'FontWeight', 'bold', ...
                'HorizontalAlignment', 'center', 'BackgroundColor', 'white', ...
                'EdgeColor', color, 'Margin', 2);
        end
    end
end

function [q_left_all, q_right_all, poses_left_all, poses_right_all, keyframe_indices] = load_motion_data_simple(motion_file)
    data = readtable(motion_file);
    
    left = 4; right = 2;
    
    robot_cols = struct();
    for i = 1:4
        robot_cols(i).x = sprintf('x%d', i);
        robot_cols(i).y = sprintf('y%d', i);
        robot_cols(i).z = sprintf('z%d', i);
        robot_cols(i).rx = sprintf('rx%d', i);
        robot_cols(i).ry = sprintf('ry%d', i);
        robot_cols(i).rz = sprintf('rz%d', i);
    end
    
    step = 50;
    num_frames = floor(height(data)/step);
    
    q_left_all  = zeros(num_frames, 6);
    q_right_all = zeros(num_frames, 6);

    poses_left_all  = zeros(num_frames, 4, 4);
    poses_right_all = zeros(num_frames, 4, 4);
    
    q0_left  = [pi/2, -pi/3, 2*pi/3, -pi/3, pi/2, 0];
    q0_right = [-pi/2, -2*pi/3, -2*pi/3, -2*pi/3, -pi/2, 0];
    
    q_left  = q0_left;
    q_right = q0_right;
    
    parameters(1, 1); [Te_w_l, Te_l] = direct_kinematics(q0_left, 1);
    parameters(1, 2); [Te_w_r, Te_r] = direct_kinematics(q0_right, 2);
    
    for i = 1:num_frames
        idx = (i-1)*step + 1;
        
        pos_rel_l = [data.(robot_cols(left).x)(idx); data.(robot_cols(left).y)(idx); data.(robot_cols(left).z)(idx)];
        pos_left = Te_l(1:3,4) + pos_rel_l;
        left_vec = [data.(robot_cols(left).rx)(idx), -data.(robot_cols(left).rz)(idx), data.(robot_cols(left).ry)(idx)];
        left_angle = norm(left_vec);
        if left_angle > 0
            rot_left = Te_l(1:3,1:3) * axang2rotm([left_vec/left_angle, left_angle]);
        else
            rot_left = Te_l(1:3,1:3);
        end
        T_l = Te_w_l * [rot_left,  pos_left;  0,0,0,1];
        
        pos_rel_r = [data.(robot_cols(right).x)(idx); data.(robot_cols(right).y)(idx); data.(robot_cols(right).z)(idx)];
        pos_right = Te_r(1:3,4) + pos_rel_r;
        right_vec = [data.(robot_cols(right).rx)(idx), data.(robot_cols(right).rz)(idx), -data.(robot_cols(right).ry)(idx)];
        right_angle = norm(right_vec);
        if right_angle > 0
            rot_right = Te_r(1:3,1:3) * axang2rotm([right_vec/right_angle, right_angle]);
        else
            rot_right = Te_r(1:3,1:3);
        end
        T_r = Te_w_r * [rot_right, pos_right;  0,0,0,1];
        
        parameters(1, 1);
        Hl = UR5_inverse_kinematics_cpp(pos_left, rot_left, AL, A, D);
        parameters(1, 2);
        Hr = UR5_inverse_kinematics_cpp(pos_right, rot_right, AL, A, D);
        q_left = get_closer(Hl, q_left);
        q_right = get_closer(Hr, q_right);
        
        q_left_all(i,:)  = q_left;
        q_right_all(i,:) = q_right;
        poses_left_all(i,:,:)  = T_l;
        poses_right_all(i,:,:) = T_r;
    end
    
    [filepath, filename, ~] = fileparts(motion_file);
    keyframe_file = fullfile(filepath, [filename, '_keyframes.csv']);
    
    try
        keyframes = readtable(keyframe_file);
        keyframe_indices = keyframes.original_index;
    catch
        keyframe_indices = [];
    end

    fprintf("Data loaded successfully\n")
end

function simulate_transformed_trajectory(timestamp, aug_id, q_left, q_right, keyframes, data_folder, object_paths, transformations)
% Simulate the transformed trajectory with robots and objects
    
    % Setup robots
    parameters(0, 1);
    robot_left = robot; config_left = config;
    parameters(0, 2);
    robot_right = robot; config_right = config;
    
    % Create simulation figure
    figure('Name', sprintf('Trajectory Simulation - Aug %d', aug_id), 'Position', [200, 200, 1200, 800]);
    hold on; grid on; axis equal;
    
    % Environment
    tableParams.height = 0.78; tableParams.width = 1.2; tableParams.length = 0.8;
    create_environment([0, 0, 0], tableParams);
    
    % Load and show original objects (grey, transparent)
    for i = 1:length(object_paths)
        ptCloud = pcread(fullfile(data_folder, object_paths{i}));
        pcshow(ptCloud.Location, [0.5 0.5 0.5], 'MarkerSize', 15, 'Parent', gca);
        alpha(0.3);
    end
    
    % Load and show transformed objects
    colors = lines(length(object_paths));
    for i = 1:length(object_paths)
        ptCloud = pcread(fullfile(data_folder, object_paths{i}));
        
        % Apply transformation
        if i <= length(transformations)
            T = transformations(i);
            transformed_points = ptCloud.Location + T.translation';
            if T.rotation_angle ~= 0
                angle_rad = deg2rad(T.rotation_angle);
                R = [cos(angle_rad), -sin(angle_rad), 0;
                     sin(angle_rad),  cos(angle_rad), 0;
                     0,               0,              1];
                transformed_points = (R * transformed_points')';
            end
            if T.scale_factor ~= 1
                transformed_points = transformed_points * T.scale_factor;
            end
            pcshow(transformed_points, colors(i,:), 'MarkerSize', 20, 'Parent', gca);
        end
    end
    
    % Show original end-effector poses (grey, transparent)
    for i = 1:5:size(q_left, 1)
        parameters(1, 1); [Te_left, ~] = direct_kinematics(q_left(1,:), 1);
        parameters(1, 2); [Te_right, ~] = direct_kinematics(q_right(1,:), 2);
        
        % Plot original frames (grey, smaller)
        plotTransforms(Te_left(1:3,4)', rotm2quat(Te_left(1:3,1:3)), 'FrameSize', 0.03, 'FrameColor', [0.5 0.5 0.5], 'Parent', gca);
        plotTransforms(Te_right(1:3,4)', rotm2quat(Te_right(1:3,1:3)), 'FrameSize', 0.03, 'FrameColor', [0.5 0.5 0.5], 'Parent', gca);
    end
    
    % Set view
    xlim([-0.8, 0.8]); ylim([-0.8, 0.8]); zlim([0, 1.5]);
    view(45, 30);
    title('Transformed Trajectory Simulation');
    
    % Simulate trajectory
    fprintf('Starting trajectory simulation...\n');
    for i = 1:5:size(q_left, 1)  % Every 5th frame for speed
        config_left  = set_robot_configuration(q_left(i,:),  config_left);
        config_right = set_robot_configuration(q_right(i,:), config_right);
        
        show(robot_left,  config_left,  'Visuals', 'on', 'Frames', 'on', 'FastUpdate', true, 'PreservePlot', false, 'Parent', gca);
        show(robot_right, config_right, 'Visuals', 'on', 'Frames', 'on', 'FastUpdate', true, 'PreservePlot', false, 'Parent', gca);
        
        % Show current transformed end-effector frames (colored, larger)
        parameters(1, 1); [Te_left_new, ~] = direct_kinematics(q_left(i,:), 1);
        parameters(1, 2); [Te_right_new, ~] = direct_kinematics(q_right(i,:), 2);
        plotTransforms(Te_left_new(1:3,4)', rotm2quat(Te_left_new(1:3,1:3)), 'FrameSize', 0.05, 'FrameColor', 'b', 'Parent', gca);
        plotTransforms(Te_right_new(1:3,4)', rotm2quat(Te_right_new(1:3,1:3)), 'FrameSize', 0.05, 'FrameColor', 'r', 'Parent', gca);
        
        % Add keyframe labels if this is a keyframe
        if ~isempty(keyframes)
            step = 50;
            for k = 1:length(keyframes)
                kf_idx = min(floor(keyframes(k) / step) + 1, size(q_left, 1));
                if abs(i - kf_idx) < 3  % If we're close to a keyframe
                    % Get keyframe name if available
                    if exist('interaction_points', 'var') && isfield(interaction_points, 'keyframes') && k <= height(interaction_points.keyframes)
                        kf_name = char(interaction_points.keyframes.extraction_method(k));
                    else
                        kf_name = sprintf('KF %d', k);
                    end
                    
                    % Add labels at both robot end-effectors
                    text(Te_left_new(1,4), Te_left_new(2,4), Te_left_new(3,4) + 0.08, kf_name, ...
                        'Color', 'b', 'FontSize', 10, 'FontWeight', 'bold', ...
                        'HorizontalAlignment', 'center', 'BackgroundColor', 'white', ...
                        'EdgeColor', 'b', 'Margin', 2);
                    text(Te_right_new(1,4), Te_right_new(2,4), Te_right_new(3,4) + 0.08, kf_name, ...
                        'Color', 'r', 'FontSize', 10, 'FontWeight', 'bold', ...
                        'HorizontalAlignment', 'center', 'BackgroundColor', 'white', ...
                        'EdgeColor', 'r', 'Margin', 2);
                end
            end
        end
        
        title(sprintf('Transformed Trajectory Simulation - Frame %d/%d', i, size(q_left, 1)));
        pause(0.05);
        
        if ~ishandle(gcf)
            break;
        end
    end
    
    fprintf('Simulation completed!\n');
end
