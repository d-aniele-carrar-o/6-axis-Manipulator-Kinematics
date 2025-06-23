function [q_l_all, q_r_all, poses_l_all, poses_r_all, use_keyframes, keyframe_indices] = load_motion_data(motion_file, q0_left, q0_right, step)
    % LOAD_MOTION_DATA Load and process motion data from CSV file
    %
    % Parameters:
    %   motion_file - Path to the CSV file containing motion data
    %   q0_left     - Initial joint configuration for left robot [1x6]
    %   q0_right    - Initial joint configuration for right robot [1x6]
    %   step        - Downsampling step (default: 50)
    %
    % Returns:
    %   q_l_all          - Joint configurations for left robot [Nx6]
    %   q_r_all          - Joint configurations for right robot [Nx6]
    %   poses_l_all      - End-effector poses for left robot  (in robot's Base Reference Frame) [Nx4x4]
    %   poses_r_all      - End-effector poses for right robot (in robot's Base Reference Frame) [Nx4x4]
    %   use_keyframes    - Boolean indicating if keyframes are available
    %   keyframe_indices - Indices of keyframes in original data
    
    if nargin < 4
        step = 50;
    end
    
    fprintf('Loading motion data from %s\n', motion_file);
    data = readtable(motion_file);
    
    % Robot mappings
    left = 4; right = 2;
    
    % Create column mappings
    robot_cols = struct();
    for i = [right, left]
        robot_cols(i).x = sprintf('x%d', i);
        robot_cols(i).y = sprintf('y%d', i);
        robot_cols(i).z = sprintf('z%d', i);
        robot_cols(i).rx = sprintf('rx%d', i);
        robot_cols(i).ry = sprintf('ry%d', i);
        robot_cols(i).rz = sprintf('rz%d', i);
    end
    
    % Downsample data
    num_frames = floor(height(data)/step);
    fprintf('Processing %d frames (downsampled from %d)\n', num_frames, height(data));
    
    % Preallocate output matrices
    q_l_all = zeros(num_frames, 6);
    q_r_all = zeros(num_frames, 6);
    poses_l_all = zeros(num_frames, 4, 4);
    poses_r_all = zeros(num_frames, 4, 4);
    
    q_l = q0_left;
    q_r = q0_right;
    [~, Te_l] = direct_kinematics(q0_left,  1);
    [~, Te_r] = direct_kinematics(q0_right, 2);
    
    % Compute joint configurations
    for i = 1:num_frames
        idx = (i-1)*step + 1;
        
        % Left robot
        pos_rel_l = [data.(robot_cols(left).x)(idx); data.(robot_cols(left).y)(idx); data.(robot_cols(left).z)(idx)];
        pos_left = Te_l(1:3,4) + pos_rel_l;
        left_vec = [data.(robot_cols(left).rx)(idx), -data.(robot_cols(left).rz)(idx), data.(robot_cols(left).ry)(idx)];
        left_angle = norm(left_vec);
        if left_angle > 0
            left_axis = left_vec / left_angle;
            rot_left = Te_l(1:3,1:3) * axang2rotm([left_axis, left_angle]);
        else
            rot_left = Te_l(1:3,1:3);
        end
        
        % Right robot
        pos_rel_r = [data.(robot_cols(right).x)(idx); data.(robot_cols(right).y)(idx); data.(robot_cols(right).z)(idx)];
        pos_right = Te_r(1:3,4) + pos_rel_r;
        right_vec = [data.(robot_cols(right).rx)(idx), data.(robot_cols(right).rz)(idx), -data.(robot_cols(right).ry)(idx)];
        right_angle = norm(right_vec);
        if right_angle > 0
            right_axis = right_vec / right_angle;
            rot_right = Te_r(1:3,1:3) * axang2rotm([right_axis, right_angle]);
        else
            rot_right = Te_r(1:3,1:3);
        end
        
        % Inverse kinematics
        parameters(1, 1);
        Hl = UR5_inverse_kinematics_cpp(pos_left, rot_left, AL, A, D);
        parameters(1, 2);
        Hr = UR5_inverse_kinematics_cpp(pos_right, rot_right, AL, A, D);
        q_l = get_closer(Hl, q_l);
        q_r = get_closer(Hr, q_r);
        
        q_l_all(i,:) = q_l;
        q_r_all(i,:) = q_r;
        
        % Store poses
        poses_l_all(i,:,:) = [rot_left,  pos_left;  0,0,0,1];
        poses_r_all(i,:,:) = [rot_right, pos_right; 0,0,0,1];

        if mod(i, 100) == 0
            fprintf("Processing %d of %d\n", i, num_frames);
        end
    end
    
    % Check for keyframes
    [filepath, filename, ~] = fileparts(motion_file);
    keyframe_file = fullfile(filepath, [filename, '_keyframes.csv']);
    
    try
        keyframes = readtable(keyframe_file);
        keyframe_indices = keyframes.original_index;
        use_keyframes = true;
        fprintf('Loaded %d keyframes\n', length(keyframe_indices));
    catch e
        use_keyframes = false;
        keyframe_indices = [];
        fprintf('No keyframes found: %s\nUsing all frames\n', e.message);
    end
end
