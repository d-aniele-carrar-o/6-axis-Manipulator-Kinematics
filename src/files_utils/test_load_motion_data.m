% Test script for load_motion_data function
clc; close all;

% Find a motion file to test with
parameters(1);
motion_files = dir(fullfile(data_folder, '**/*_motion.csv'));
if isempty(motion_files)
    error('No motion files found in data folder');
end

% Use the first motion file found
motion_file = fullfile(motion_files(1).folder, motion_files(1).name);
fprintf('Testing with motion file: %s\n', motion_file);

% Load motion data with default parameters
step = 50; % Default step size
fprintf('\n--- Loading with step size %d ---\n', step);
[q_l_all, q_r_all, poses_l_all, poses_r_all, keyframes_data] = load_motion_data(motion_file, [], [], step);

% Also test with a different step size
smaller_step = 25;
fprintf('\n--- Loading with step size %d ---\n', smaller_step);
[q_l_small, q_r_small, poses_l_small, poses_r_small, keyframes_small] = load_motion_data(motion_file, [], [], smaller_step);

% Function to display and analyze loaded data
function analyze_loaded_data(q_l, q_r, poses_l, poses_r, kf_data, step_size)
    % Display information about the loaded data
    fprintf('\nLoaded data summary (step=%d):\n', step_size);
    fprintf('  Left robot joint configurations: %d x %d\n', size(q_l, 1), size(q_l, 2));
    fprintf('  Right robot joint configurations: %d x %d\n', size(q_r, 1), size(q_r, 2));
    fprintf('  Left robot poses: %d x %d x %d\n', size(poses_l, 1), size(poses_l, 2), size(poses_l, 3));
    fprintf('  Right robot poses: %d x %d x %d\n', size(poses_r, 1), size(poses_r, 2), size(poses_r, 3));
    
    % Display keyframe information
    if kf_data.available
        fprintf('\nKeyframes information:\n');
        fprintf('  Number of keyframes: %d\n', length(kf_data.indices));
        fprintf('  Keyframe indices in trajectory: %s\n', mat2str(kf_data.indices'));
        
        % Verify that keyframe indices are valid
        valid_indices = all(kf_data.indices >= 1 & kf_data.indices <= size(q_l, 1));
        fprintf('  All keyframe indices are valid: %s\n', mat2str(valid_indices));
        
        % Plot trajectory with keyframes highlighted
        figure('Name', sprintf('Trajectory with Keyframes (step=%d)', step_size), 'Position', [100, 100, 1200, 600]);
        
        % Left robot trajectory
        subplot(1, 2, 1);
        hold on; grid on;
        title('Left Robot Trajectory');
        
        % Extract end-effector positions
        ee_left = zeros(size(q_l, 1), 3);
        for i = 1:size(q_l, 1)
            ee_left(i,:) = squeeze(poses_l(i,1:3,4))';
        end
        
        % Plot trajectory
        plot3(ee_left(:,1), ee_left(:,2), ee_left(:,3), 'b-', 'LineWidth', 1);
        
        % Highlight keyframes
        for i = 1:length(kf_data.indices)
            kf_idx = kf_data.indices(i);
            scatter3(ee_left(kf_idx,1), ee_left(kf_idx,2), ee_left(kf_idx,3), 100, 'r*');
            text(ee_left(kf_idx,1), ee_left(kf_idx,2), ee_left(kf_idx,3) + 0.05, ...
                kf_data.names{i}, 'FontSize', 8);
        end
        
        % Right robot trajectory
        subplot(1, 2, 2);
        hold on; grid on;
        title('Right Robot Trajectory');
        
        % Extract end-effector positions
        ee_right = zeros(size(q_r, 1), 3);
        for i = 1:size(q_r, 1)
            ee_right(i,:) = squeeze(poses_r(i,1:3,4))';
        end
        
        % Plot trajectory
        plot3(ee_right(:,1), ee_right(:,2), ee_right(:,3), 'r-', 'LineWidth', 1);
        
        % Highlight keyframes
        for i = 1:length(kf_data.indices)
            kf_idx = kf_data.indices(i);
            scatter3(ee_right(kf_idx,1), ee_right(kf_idx,2), ee_right(kf_idx,3), 100, 'b*');
            text(ee_right(kf_idx,1), ee_right(kf_idx,2), ee_right(kf_idx,3) + 0.05, ...
                kf_data.names{i}, 'FontSize', 8);
        end
        
        % Set consistent view for both subplots
        for i = 1:2
            subplot(1, 2, i);
            view(45, 30);
            xlabel('X'); ylabel('Y'); zlabel('Z');
        end
    else
        fprintf('\nNo keyframes available in this motion file.\n');
    end
end

% Analyze data from both step sizes
analyze_loaded_data(q_l_all, q_r_all, poses_l_all, poses_r_all, keyframes_data, step);
analyze_loaded_data(q_l_small, q_r_small, poses_l_small, poses_r_small, keyframes_small, smaller_step);

% Compare keyframe indices between different step sizes
if keyframes_data.available && keyframes_small.available
    fprintf('\nComparing keyframes between different step sizes:\n');
    fprintf('  Step %d: %d keyframes, indices: %s\n', step, length(keyframes_data.indices), mat2str(keyframes_data.indices'));
    fprintf('  Step %d: %d keyframes, indices: %s\n', smaller_step, length(keyframes_small.indices), mat2str(keyframes_small.indices'));
    
    % Check if the same keyframes are found in both cases
    same_num_keyframes = length(keyframes_data.indices) == length(keyframes_small.indices);
    same_names = all(strcmp(keyframes_data.names, keyframes_small.names));
    fprintf('  Same number of keyframes: %s\n', mat2str(same_num_keyframes));
    fprintf('  Same keyframe names: %s\n', mat2str(same_names));
    
    % Check if keyframe indices are proportional to step size
    if same_num_keyframes && same_names
        % Calculate expected indices for smaller step size
        expected_indices = round(keyframes_data.indices * step / smaller_step);
        fprintf('  Expected indices for step %d: %s\n', smaller_step, mat2str(expected_indices'));
        fprintf('  Actual indices for step %d: %s\n', smaller_step, mat2str(keyframes_small.indices'));
    end
end