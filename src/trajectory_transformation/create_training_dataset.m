function create_training_dataset(timestamp, output_filename, task_name)
% CREATE_TRAINING_DATASET Generate training dataset from augmented trajectories
%
% Inputs:
%   timestamp - Original scene timestamp
%   output_filename - Name of output JSON file (optional)
%   task_name - Name of the task (optional, default: 'manipulation_task')

    if nargin < 2
        output_filename = sprintf('training_dataset_%s.json', timestamp);
    end
    if nargin < 3
        task_name = 'lift_box';
    end

    % Load augmentation data
    parameters(1);
    json_file = fullfile(augmented_demos_path, timestamp, 'augmented_demos.json');
    augmentation_data = jsondecode(fileread(json_file));

    fprintf('Processing %d augmentations for timestamp %s\n', length(augmentation_data.augmentations), timestamp);

    % Load motion data once
    motion_file = find_closest_motion_file(timestamp);
    if isempty(motion_file)
        error('No motion data found for timestamp %s', timestamp);
    end
    [ql_orig, qr_orig, ~, ~, keyframes_data] = load_motion_data(motion_file);
    motion_data = struct('ql_orig', ql_orig, 'qr_orig', qr_orig, 'keyframes_data', keyframes_data);

    % Initialize dataset structure
    dataset = struct();
    dataset.summary = create_summary(timestamp, task_name, augmentation_data, keyframes_data);
    dataset.demonstrations = [];

    % Add original demonstration
    original_demo = create_original_demonstration(motion_data, augmentation_data, task_name, 0);
    dataset.demonstrations = original_demo;

    % Process each augmentation
    for aug_id = 0:(length(augmentation_data.augmentations)-1)
        fprintf('Processing augmentation %d/%d...\n', aug_id+1, length(augmentation_data.augmentations));
        
        % Create augmented demonstration
        aug_demo = create_augmented_demonstration(timestamp, aug_id, motion_data, augmentation_data, task_name, aug_id+1);
        dataset.demonstrations(end+1) = aug_demo;
        
        disp('---------------------------------------------------------------------------------------')
    end

    % Save dataset
    output_path = fullfile(augmented_demos_path, timestamp, output_filename);
    json_str = jsonencode(dataset, 'PrettyPrint', true);
    fid = fopen(output_path, 'w');
    fprintf(fid, '%s', json_str);
    fclose(fid);

    fprintf('Training dataset saved to: %s\n', output_path);
    fprintf('Total demonstrations: %d\n', length(dataset.demonstrations));
end

function summary = create_summary(timestamp, task_name, augmentation_data, keyframes_data)
% Create summary section for dataset

    summary = struct();
    summary.task_name = task_name;
    summary.total_demonstrations = length(augmentation_data.augmentations) + 1;
    summary.original_demonstrations = 1;
    summary.augmented_demonstrations = length(augmentation_data.augmentations);
    summary.num_objects_per_demo = length(augmentation_data.original_objects.paths);
    summary.num_action_steps_per_demo = length(keyframes_data.indices);
    summary.timestamp = timestamp;
    summary.has_real_actions = true;
end

function demo = create_original_demonstration(motion_data, augmentation_data, task_name, demo_idx)
% Create original demonstration structure

    demo = struct();
    demo.obs = load_original_point_cloud(augmentation_data.original_objects.paths);
    demo.actions = create_actions_from_trajectory(motion_data.ql_orig, motion_data.qr_orig, motion_data.keyframes_data);
    demo.demo_idx = demo_idx;
    demo.is_augmented = false;
    demo.task_name = task_name;
    demo.num_objects = length(augmentation_data.original_objects.paths);
    demo.num_action_steps = length(demo.actions);
    demo.transformations = []; % Add empty transformations field
end

function demo = create_augmented_demonstration(timestamp, aug_id, motion_data, augmentation_data, task_name, demo_idx)
% Create augmented demonstration structure

    % Get transformed trajectory
    options = struct('visualize', false, 'simulate', false, 'save_result', false);
    [ql_new, qr_new, keyframes_data] = augment_trajectory(timestamp, aug_id, options, motion_data, augmentation_data);

    demo = struct();
    demo.obs = load_transformed_point_cloud(augmentation_data, aug_id);
    demo.actions = create_actions_from_trajectory(ql_new, qr_new, keyframes_data);
    demo.demo_idx = demo_idx;
    demo.is_augmented = true;
    demo.task_name = task_name;
    demo.transformations = format_transformations(augmentation_data.augmentations(aug_id+1).transformations);
    demo.num_objects = length(augmentation_data.original_objects.paths);
    demo.num_action_steps = length(demo.actions);
end


function obs = load_original_point_cloud(object_paths)
% Load original point cloud data

    parameters(1);
    obs = [];
    for i = 1:length(object_paths)
        pc = pcread([data_folder, object_paths{i}]);
        obs = [obs; pc.Location];
    end
end

function obs = load_transformed_point_cloud(augmentation_data, aug_id)
% Load transformed point cloud data

    transformations = augmentation_data.augmentations(aug_id+1).transformations;
    object_data = load_and_transform_objects(augmentation_data.original_objects.paths, transformations);

    obs = [];
    for i = 1:length(object_data.transformed_pcs)
        obs = [obs; object_data.transformed_pcs{i}.Location];
    end
end

function actions = create_actions_from_trajectory(ql_traj, qr_traj, keyframes_data)
% Create actions array from joint trajectories

    actions = [];
    task_phases = generate_task_phases(keyframes_data);

    for i = 1:size(ql_traj, 1)
        % Convert joint angles to end-effector poses
        parameters(1, 1); [T_left, ~] = direct_kinematics(ql_traj(i,:), 1);
        parameters(1, 2); [T_right, ~] = direct_kinematics(qr_traj(i,:), 2);
        
        % Extract poses
        left_pose = [T_left(1:3,4)', rotm2eul(T_left(1:3,1:3), 'XYZ')];
        right_pose = [T_right(1:3,4)', rotm2eul(T_right(1:3,1:3), 'XYZ')];
        
        % Determine task phase
        phase_idx = find(keyframes_data.indices <= i, 1, 'last');
        if isempty(phase_idx), phase_idx = 1; end
        
        action = struct();
        action.left_arm = left_pose;
        action.right_arm = right_pose;
        action.step = i-1;
        action.task_phase = task_phases{min(phase_idx, length(task_phases))};
        
        actions = [actions; action];
    end
end

function methods = generate_task_phases(keyframes_data)
% Generate extraction methods based on keyframe names

    methods = {};
    if isfield(keyframes_data, 'names')
        for i = 1:length(keyframes_data.names)
            name = keyframes_data.names{i};
            if contains(name, 'boundary', 'IgnoreCase', true)
                methods{end+1} = 'boundary';
            elseif contains(name, 'grasp', 'IgnoreCase', true)
                % Extract robot number from keyframe name (look for r1, r2, r3, r4)
                robot_match = regexp(name, 'r([1-4])', 'tokens', 'once');
                if ~isempty(robot_match)
                    robot_num = robot_match{1};
                else
                    robot_num = '2'; % default to r2
                end
                
                if contains(name, 'pre', 'IgnoreCase', true)
                    methods{end+1} = sprintf('pre_grasp_r%s', robot_num);
                elseif contains(name, 'post', 'IgnoreCase', true) || contains(name, 'ungrasp', 'IgnoreCase', true)
                    methods{end+1} = sprintf('post-ungrasp_r%s', robot_num);
                else
                    methods{end+1} = sprintf('grasp_r%s', robot_num);
                end
            else
                methods{end+1} = 'boundary';
            end
        end
    else
        % Default methods if no names available
        methods = {'boundary', 'pre_grasp_r2', 'grasp_r2', 'ungrasp_r2', 'post-ungrasp_r2', 'boundary'};
    end
end

function transformations = format_transformations(raw_transformations)
% Format transformations for JSON output

    transformations = [];
    for i = 1:length(raw_transformations)
        t = raw_transformations(i);
        trans = struct();
        trans.translation = t.translation;
        trans.rotation_angle = t.rotation_angle;
        trans.rotation_axis = 'z';
        trans.scale_factor = t.scale_factor;
        transformations = [transformations; trans];
    end
end
