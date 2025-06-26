function [ql_new, qr_new, keyframes_data, interaction_points] = transform_trajectory(keyframes_data, interaction_points)
% Advanced trajectory transformation using keyframe-based interactions
    N = length(interaction_points.keyframes.indices);
    fprintf('Transforming trajectory using %d interaction keyframes\n', N);
    
    % Compute transformed poses for keyframes (+1 for final boundary keyframe)
    keyframe_poses_left  = zeros(N/2+1, 4, 4);
    keyframe_poses_right = zeros(N/2+1, 4, 4);
    
    % Initialize storage for world-frame transformations
    if ~isfield(interaction_points, 'world_frame_tfs')
        interaction_points.world_frame_tfs = struct('left', cell(1, N/2), 'right', cell(1, N/2));
        keyframes_data.transformed_keyframes_l = zeros(N/2+2, 4, 4);
        keyframes_data.transformed_keyframes_r = zeros(N/2+2, 4, 4);
    end

    % Get world-base tf for both robots
    parameters(1, 1); Trf_0_l = Trf_0;
    parameters(1, 2); Trf_0_r = Trf_0;
    
    idx_l = 1; idx_r = 1;
    for i = 1:N
        kf_idx = interaction_points.indices(i);
        
        % Get object index from interaction_points (computed based on closest object)
        obj_idx = interaction_points.objects(i);
        transformation = interaction_points.transformations(obj_idx);
        
        % Get end-effector poses at this keyframe - use pre-computed end-effector pose
        T_w_e_orig = interaction_points.ee_poses{i};
        robot_num = interaction_points.robot_nums(i);

        % Apply full object transformation to keyframes end-effector poses
        % Get transformation parameters
        translation    = transformation.translation;
        rotation_angle = transformation.rotation_angle;
        scale_factor   = transformation.scale_factor;
        
        % Create copy of original pose for transformation
        T_w_e_new = T_w_e_orig;
        
        % Extract end-effector positions
        pos = T_w_e_new(1:3,4);
        
        % Get object centroid from interaction_points
        if isfield(interaction_points, 'object_centroids') && obj_idx <= size(interaction_points.object_centroids, 1)
            centroid = interaction_points.object_centroids(obj_idx,:)';
        else
            % Fallback: Use a default centroid based on the translation
            % This assumes the translation is applied to the object's centroid
            centroid = translation;
        end
        
        % Apply transformations in the correct order: scale, rotate, translate
        
        % 1. Apply scaling relative to object centroid
        if scale_factor ~= 1
            pos = centroid + scale_factor * (pos - centroid);
        end
        
        % 2. Apply rotation around Z axis centered at object centroid
        if rotation_angle ~= 0
            angle_rad = deg2rad(rotation_angle);
            R = [cos(angle_rad), -sin(angle_rad), 0;
                 sin(angle_rad),  cos(angle_rad), 0;
                 0,               0,              1];
            
            % Rotate position around centroid
            pos_centered = pos - centroid;
            pos = R * pos_centered + centroid;
            
            % Also rotate the orientation
            T_w_e_new(1:3,1:3) = R * T_w_e_new(1:3,1:3);
        end
        
        % 3. Apply translation
        pos = pos + translation;
        
        % Update the transformation matrix with the new position
        T_w_e_new(1:3,4) = pos;
        
        % Save the transformed keyframe to the appropiate robot (left/right) for traj generation
        if robot_num == 4 % Left robot
            T_e = Trf_0_l \ T_w_e_new;
            keyframe_poses_left(idx_l,:,:) = T_e;
            % Store the world-frame transformations for visualization
            idx_l = idx_l + 1;
            interaction_points.world_frame_tfs(idx_l).left = T_w_e_new;
            keyframes_data.transformed_keyframes_l(idx_l,:,:) = T_w_e_new;
        else % Right robot
            T_e = Trf_0_r \ T_w_e_new;
            keyframe_poses_right(idx_r,:,:) = T_e;
            % Store the world-frame transformations for visualization
            idx_r = idx_r + 1;
            interaction_points.world_frame_tfs(idx_r).right = T_w_e_new;
            keyframes_data.transformed_keyframes_r(idx_r,:,:) = T_w_e_new;
        end
        
        if verbose
            fprintf('  Keyframe %d: traj_idx=%d, obj=%d, trans=[%.3f,%.3f,%.3f], rot=%.1f°, scale=%.2f\n', ...
                i, kf_idx, obj_idx, translation, rotation_angle, scale_factor);
        end
    end

    % Add boundary keyframes (unchanged) in robots base frame
    % Find indices where keyframes_data.names is "boundary"
    boundary_indices = find(strcmp(keyframes_data.names, 'boundary'));

    % Extract the boundary keyframes
    boundary_kfs_l = keyframes_data.q_l(boundary_indices,:,:);
    boundary_kfs_r = keyframes_data.q_r(boundary_indices,:,:);
    
    % Compute end-effector poses for left/right robots
    [Twel_i, ~] = direct_kinematics(boundary_kfs_l(1,:), 1);
    [Twel_f, Tel_f] = direct_kinematics(boundary_kfs_l(2,:), 1);
    [Twer_i, ~] = direct_kinematics(boundary_kfs_r(1,:), 1);
    [Twer_f, Ter_f] = direct_kinematics(boundary_kfs_r(2,:), 1);

    keyframe_poses_left(end,:,:)  = Tel_f;
    keyframe_poses_right(end,:,:) = Ter_f;
    keyframes_data.transformed_keyframes_l(1,:,:)   = Twel_i;
    keyframes_data.transformed_keyframes_l(end,:,:) = Twel_f;
    keyframes_data.transformed_keyframes_r(1,:,:)   = Twer_i;
    keyframes_data.transformed_keyframes_r(end,:,:) = Twer_f;
    
    % Generate new trajectory using multipoint_trajectory based on the transformed keyframes
    period = 2;
    times = ones(1, N/2+1) * period; % 2 seconds between keyframes, +1 for final boundary keyframe
    
    % TODO: !!!!! use the original times/periods between the keyframes to create the new trajectories!!!!!!!!!!!
    
    keyframes_data.transformed_indices = [1, (1:(N/2+1)) * period/dt];

    q0l = keyframes_data.q_left_all(1,:);
    q0r = keyframes_data.q_right_all(1,:);
    [~, posl_new, vl_new] = multipoint_trajectory(q0l, keyframe_poses_left,  times, 1);
    [~, posr_new, vr_new] = multipoint_trajectory(q0r, keyframe_poses_right, times, 2);
    
    [ql_new, ~] = task2joint_space(q0l, posl_new, vl_new);
    [qr_new, ~] = task2joint_space(q0r, posr_new, vr_new);
    
    fprintf('Generated new trajectory with %d frames\n', size(ql_new, 1));
    
    % Add the transformed keyframe indices to the interaction_points structure
    interaction_points.transformed_keyframe_indices = interaction_points.indices;
end
