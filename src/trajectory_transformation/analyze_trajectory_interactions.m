function interaction_points = analyze_trajectory_interactions(keyframes_data, transformations, transformed_centroids)
% Read keyframes data to find object interaction points and compute closest objects
%
% Parameters:
%   keyframes_data - Structure containing keyframe information
%   transformations - Array of transformation structures for each object
%   transformed_centroids - Matrix of transformed object centroids [num_objects x 3]
    
    parameters(1);
    
    fprintf('Available extraction methods in keyframes:\n');
    unique_methods = unique(keyframes_data.names);
    if verbose
        for i = 1:length(unique_methods)
            fprintf('  - %s\n', unique_methods{i});
        end
    end
    
    % Filter for interaction keyframes (grasp/ungrasp methods) for all robots
    % Create pattern for all follower robot numbers (2,4)
    interaction_methods = {};
    for robot_num = [2,4]
        interaction_methods = [interaction_methods, ...
            {sprintf('pre_grasp_r%d', robot_num), ...
             sprintf('grasp_r%d', robot_num), ...
             sprintf('ungrasp_r%d', robot_num), ...
             sprintf('post_ungrasp_r%d', robot_num)}];
    end
    interaction_mask = ismember(keyframes_data.names, interaction_methods);
    
    % Create a table-like structure from keyframes_data for filtered keyframes
    interaction_keyframes = struct();
    interaction_keyframes.extraction_method = keyframes_data.names(interaction_mask);
    interaction_keyframes.indices = keyframes_data.indices(interaction_mask);
    
    % Get joint configurations for the filtered keyframes
    % Since keyframes are now integrated into the main trajectory, we need to extract them
    if keyframes_data.available
        interaction_keyframes.q_l = keyframes_data.q_l(interaction_mask,:);
        interaction_keyframes.q_r = keyframes_data.q_r(interaction_mask,:);
    end
    
    fprintf('Found %d interaction keyframes\n', length(interaction_keyframes.indices));
    
    % Create interaction points structure
    interaction_points = struct();
    interaction_points.keyframes = interaction_keyframes;
    interaction_points.transformations = transformations;
    interaction_points.indices = interaction_keyframes.indices;
    interaction_points.extraction_method = interaction_keyframes.extraction_method;
    interaction_points.q_l = interaction_keyframes.q_l;
    interaction_points.q_r = interaction_keyframes.q_r;
    
    % Store object centroids for transformation calculations
    interaction_points.object_centroids = transformed_centroids;
    
    % Initialize arrays for robot and object assignments
    num_objects = size(transformed_centroids, 1);
    interaction_points.objects = zeros(length(interaction_keyframes.indices), 1);
    interaction_points.left  = [];
    interaction_points.right = [];
    interaction_points.robot_nums = zeros(length(interaction_keyframes.indices), 1);
    interaction_points.ee_poses = cell(length(interaction_keyframes.indices), 1);
    
    % Group keyframes by action type and compute end-effector poses in world frame
    for i = 1:length(interaction_keyframes.indices)
        % Extract robot number from keyframe name
        method_name = interaction_keyframes.extraction_method{i};
        robot_num = str2double(regexp(method_name, 'r(\d+)', 'tokens', 'once'));
        
        % Assign to left or right based on robot number (4=left, 2=right)
        if robot_num == 4
            interaction_points.left = [interaction_points.left; i];
            % Compute end-effector pose in world frame for left robot
            parameters(1, 1); % Set parameters for left robot
            [T_w_e, ~] = direct_kinematics(interaction_keyframes.q_l(i,:), 1);
        elseif robot_num == 2
            interaction_points.right = [interaction_points.right; i];
            % Compute end-effector pose in world frame for right robot
            parameters(1, 2); % Set parameters for right robot
            [T_w_e, ~] = direct_kinematics(interaction_keyframes.q_r(i,:), 2);
        end
        
        % Store robot number and end-effector pose for reference
        interaction_points.robot_nums(i) = robot_num;
        interaction_points.ee_poses{i} = T_w_e;
        
        % Find closest object by computing distances to all transformed objects
        ee_position = T_w_e(1:3,4);  % Extract end-effector position
        min_dist = inf;
        closest_obj = 1; % Default to first object
        
        % Compute distances to transformed object centroids
        for obj_idx = 1:num_objects
            obj_position = transformed_centroids(obj_idx,:);
            dist = norm(ee_position - obj_position');
            if dist < min_dist
                min_dist = dist;
                closest_obj = obj_idx;
            end
        end
        
        % Assign closest object
        interaction_points.objects(i) = closest_obj;
        if verbose
            fprintf('  Keyframe %d [%15s]: closest_obj=%d, distance=%.3f\n', ...
                i, method_name, closest_obj, min_dist);
        end
    end
    
    fprintf('Organized %d interaction points across %d objects\n', ...
        length(interaction_keyframes.indices), num_objects);
end
