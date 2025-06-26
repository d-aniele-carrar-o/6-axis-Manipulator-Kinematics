function [ql_new, qr_new, keyframes_data] = augment_trajectory(timestamp, aug_id, options, motion_data, aug_data)
% TRAJECTORY_TRANSFORM Transform robot trajectories based on object transformations
%
% Inputs:
%   timestamp - Original scene timestamp
%   augmentation_id - ID of the augmentation to apply
%   options - Structure with optional parameters:
%     .visualize - Show visualization (default: true)
%     .simulate - Simulate transformed trajectory (default: true)
%     .save_result - Save transformed trajectory (default: false)
%   motion_data - Pre-loaded motion data (optional, for efficiency)
%   aug_data - Pre-loaded augmentation data (optional, for efficiency)

% Default options
if nargin < 3
    options = struct();
end
if ~isfield(options, 'visualize'), options.visualize = false; end
if ~isfield(options, 'simulate'), options.simulate = true; end
if ~isfield(options, 'save_result'), options.save_result = false; end

% Load augmentation data
if nargin < 5 || isempty(aug_data)
    parameters(1);
    json_file = fullfile(augmented_demos_path, timestamp, 'augmented_demos.json');
    fprintf('Loading augmentation data...\n');
    augmentation_data = jsondecode(fileread(json_file));
else
    augmentation_data = aug_data;
end

aug_scene = augmentation_data.augmentations(aug_id+1); % +1 for MATLAB indexing
transformations = aug_scene.transformations;

% Load objects and apply transformations
object_data = load_and_transform_objects(augmentation_data.original_objects.paths, transformations);

if nargout == 0
    fprintf('Found %d objects with %d transformations\n', length(object_data.original_pcs), length(transformations));
    for i = 1:min(3, length(transformations))
        fprintf('  Transform %d: [%.3f, %.3f, %.3f], rot=%.1f°, scale=%.2f\n', ...
            i, transformations(i).translation, transformations(i).rotation_angle, transformations(i).scale_factor);
    end
end

% Load and analyze trajectory
if nargin < 4 || isempty(motion_data)
    motion_file = find_closest_motion_file(timestamp);
    if isempty(motion_file)
        error('No motion data found for timestamp %s', timestamp);
    end
    [ql_orig, qr_orig, ~, ~, keyframes_data] = load_motion_data(motion_file);
else
    ql_orig = motion_data.ql_orig;
    qr_orig = motion_data.qr_orig;
    keyframes_data = motion_data.keyframes_data;
end

% Add trajectory data to keyframes_data for analyze_trajectory_interactions
keyframes_data.q_left_all  = ql_orig;
keyframes_data.q_right_all = qr_orig;

% Analyze trajectory for object interactions
interaction_points = analyze_trajectory_interactions(keyframes_data, transformations, ...
                                object_data.transformed_centroids);

% Transform trajectory based on interactions
[ql_new, qr_new, keyframes_data, interaction_points] = transform_trajectory(keyframes_data, interaction_points);

% Visualize if requested
if options.visualize
    visualize_transformation(aug_id, ql_orig, qr_orig, ql_new, qr_new, ...
        object_data, interaction_points, keyframes_data);
    disp("press enter to continue...")
    pause()
end

% Save results if requested
if options.save_result
    save_transformed_trajectory(timestamp, aug_id, ql_new, qr_new, keyframes_data.indices);
end

% Simulate if requested
if options.simulate
    simulate_transformed_trajectory(aug_id, ql_orig, qr_orig, ql_new, qr_new, keyframes_data, object_data);
end

if nargout == 0
    fprintf('Advanced trajectory transformation completed!\n');
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

function simulate_transformed_trajectory(aug_id, ql_orig, qr_orig, ql_new, qr_new, keyframes_data, object_data)
% Simulate the transformed trajectory with robots and objects
    
    % Setup robots
    parameters(0, 1);
    robot_left = robot; config_left = config;
    parameters(0, 2);
    robot_right = robot; config_right = config;
    
    % Create simulation figure
    figure('Name', sprintf('Trajectory Simulation - Aug %d', aug_id), 'Position', [200, 200, 1200, 800]);
    hold on; grid on;
    
    % Environment
    create_environment();
    
    % Show original objects (grey, transparent)
    for i = 1:length(object_data.original_pcs)
        pcshow(object_data.original_pcs{i}.Location, [0.5 0.5 0.5], 'MarkerSize', 15, 'Parent', gca);
        alpha(0.3);
    end
    
    % Show transformed objects
    colors = lines(length(object_data.transformed_pcs));
    for i = 1:length(object_data.transformed_pcs)
        pcshow(object_data.transformed_pcs{i}.Location, colors(i,:), 'MarkerSize', 20, 'Parent', gca);
    end
    
    % Show original trajectories (grey)
    [ee_posl_orig, ~] = get_end_effector_trajectory(ql_orig, 1, true);
    [ee_posr_orig, ~] = get_end_effector_trajectory(qr_orig, 2, true);
    plot3(ee_posl_orig(:,1), ee_posl_orig(:,2), ee_posl_orig(:,3), 'Color', [0.5 0.5 0.5], 'LineWidth', 1);
    plot3(ee_posr_orig(:,1), ee_posr_orig(:,2), ee_posr_orig(:,3), 'Color', [0.5 0.5 0.5], 'LineWidth', 1);
    
    % Show new trajectories (colored)
    [ee_posl_new, ~] = get_end_effector_trajectory(ql_new, 1, true);
    [ee_posr_new, ~] = get_end_effector_trajectory(qr_new, 2, true);
    plot3(ee_posl_new(:,1), ee_posl_new(:,2), ee_posl_new(:,3), 'b-', 'LineWidth', 2);
    plot3(ee_posr_new(:,1), ee_posr_new(:,2), ee_posr_new(:,3), 'r-', 'LineWidth', 2);
    
    % Show original keyframes (grey frames)
    if isfield(keyframes_data, 'q_l') && isfield(keyframes_data, 'q_r')
        for i = 1:size(keyframes_data.q_l, 1)
            parameters(1, 1); [Twel_orig, ~] = direct_kinematics(keyframes_data.q_l(i,:), 1);
            parameters(1, 2); [Twer_orig, ~] = direct_kinematics(keyframes_data.q_r(i,:), 2);
            
            plotTransforms(Twel_orig(1:3,4)', rotm2quat(Twel_orig(1:3,1:3)), 'FrameSize', 0.05, 'FrameColor', [0.5 0.5 0.5]);
            plotTransforms(Twer_orig(1:3,4)', rotm2quat(Twer_orig(1:3,1:3)), 'FrameSize', 0.05, 'FrameColor', [0.5 0.5 0.5]);
        end
    end
    
    % Show transformed keyframes (colored frames)
    if isfield(keyframes_data, 'transformed_keyframes_l') && isfield(keyframes_data, 'transformed_keyframes_r')
        for i = 1:size(keyframes_data.transformed_keyframes_l, 1)
            if any(any(keyframes_data.transformed_keyframes_l(i,:,:)))
                T_l = squeeze(keyframes_data.transformed_keyframes_l(i,:,:));
                plotTransforms(T_l(1:3,4)', rotm2quat(T_l(1:3,1:3)), 'FrameSize', 0.08, 'FrameColor', [0 0 1]);
            end
        end
        for i = 1:size(keyframes_data.transformed_keyframes_r, 1)
            if any(any(keyframes_data.transformed_keyframes_r(i,:,:)))
                T_r = squeeze(keyframes_data.transformed_keyframes_r(i,:,:));
                plotTransforms(T_r(1:3,4)', rotm2quat(T_r(1:3,1:3)), 'FrameSize', 0.08, 'FrameColor', [1 0 0]);
            end
        end
    end
    
    % Set view
    xlim([-0.8, 0.8]); ylim([-0.8, 0.8]); zlim([0, 1.5]);
    view(45, 30);
    title('Transformed Trajectory Simulation');
    
    % Simulate trajectory
    fprintf('Starting trajectory simulation, press any key to continue...\n');
    pause()
    kf_idx = 1
    for i = 1:size(ql_new, 1)
        config_left  = set_robot_configuration(ql_new(i,:),  config_left);
        config_right = set_robot_configuration(qr_new(i,:), config_right);
        
        show(robot_left,  config_left,  'Visuals', 'on', 'Frames', 'on', 'FastUpdate', true, 'PreservePlot', false, 'Parent', gca);
        show(robot_right, config_right, 'Visuals', 'on', 'Frames', 'on', 'FastUpdate', true, 'PreservePlot', false, 'Parent', gca);
        
        % Show current transformed end-effector frames (colored, larger)
        parameters(1, 1); [Twel_new, ~] = direct_kinematics(ql_new(i,:), 1);
        parameters(1, 2); [Twer_new, ~] = direct_kinematics(qr_new(i,:), 2);
        
        % Check if current frame is a keyframe
        if keyframes_data.transformed_indices(kf_idx) == i
            keyframes_data.transformed_indices(kf_idx)
            kf_name = keyframes_data.names{kf_idx}

            % Add labels at both robot end-effectors
            text(Twel_new(1,4), Twel_new(2,4), Twel_new(3,4) + 0.08, kf_name, ...
                'Color', 'b', 'FontSize', 10, 'FontWeight', 'bold', ...
                'HorizontalAlignment', 'center', 'BackgroundColor', 'white', ...
                'EdgeColor', 'b', 'Margin', 2);
            text(Twer_new(1,4), Twer_new(2,4), Twer_new(3,4) + 0.08, kf_name, ...
                'Color', 'r', 'FontSize', 10, 'FontWeight', 'bold', ...
                'HorizontalAlignment', 'center', 'BackgroundColor', 'white', ...
                'EdgeColor', 'r', 'Margin', 2);
            
            kf_idx = kf_idx + 1
        end
        
        title(sprintf('Transformed Trajectory Simulation - Frame %d/%d', i, size(ql_new, 1)));
        pause(0.05);
        
        if ~ishandle(gcf)
            break;
        end
    end
    
    fprintf('Simulation completed!\n');
end
