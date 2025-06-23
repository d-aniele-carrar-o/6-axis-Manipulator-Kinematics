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
parameters(1);
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
parameters(1);
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
    create_environment([0, 0, 0], tableParams);
    
    % Original object
    scatter3(orig_center(1), orig_center(2), orig_center(3), 200, 'ro', 'filled');
    
    % Robot trajectories
    plot_robot_trajectory(q_left_orig,  keyframe_indices, 'b-', true, step, 1);
    plot_robot_trajectory(q_right_orig, keyframe_indices, 'r-', true, step, 2);
    
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
