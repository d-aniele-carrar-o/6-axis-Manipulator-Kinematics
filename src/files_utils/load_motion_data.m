function [q_l_all, q_r_all, poses_l_all, poses_r_all, use_keyframes, keyframe_indices, keyframe_names] = load_motion_data(motion_file, q0_left, q0_right, step)
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
    %   keyframe_names   - Names of keyframes
    
    if nargin < 4
        step = 50;
    end
    
    fprintf('Loading motion data from %s\n', motion_file);
    
    % Read raw data without headers (matching Python implementation)
    opts = detectImportOptions(motion_file, 'NumHeaderLines', 0);
    data_raw = readmatrix(motion_file, opts);
    
    % Process timestamps to seconds (matching Python implementation)
    if size(data_raw, 2) >= 2
        data_raw(:,2) = (data_raw(:,2) - data_raw(1,2)) / 1000000.0; % Convert to seconds
    end
    
    % Define column offsets based on selected robots
    right = 2;
    left  = 4;
    right_offset = 18*(right-1);  % Right pair
    left_offset  = 18*(left-1);   % Left pair
    
    % Downsample data
    num_frames = floor(size(data_raw, 1)/step);
    fprintf('Processing %d frames (downsampled from %d)\n', num_frames, size(data_raw, 1));
    
    % Preallocate output matrices
    q_l_all = zeros(num_frames, 6);
    q_r_all = zeros(num_frames, 6);
    poses_l_all = zeros(num_frames, 4, 4);
    poses_r_all = zeros(num_frames, 4, 4);
    
    q_l = q0_left;
    q_r = q0_right;
    
    % Compute joint configurations
    for i = 1:num_frames
        idx = (i-1)*step + 1;
        
        % Left robot (using Python column indices)
        pos_left = [data_raw(idx, 3+left_offset); data_raw(idx, 6+left_offset); data_raw(idx, 9+left_offset)];
        
        % Get rotation vector and convert to rotation matrix
        left_vec = [data_raw(idx, 12+left_offset), data_raw(idx, 15+left_offset), data_raw(idx, 18+left_offset)];
        left_angle = norm(left_vec);
        if left_angle > 0
            left_axis = left_vec / left_angle;
            rot_left = axang2rotm([left_axis, left_angle]);
        else
            rot_left = eye(4);
        end
        
        % Right robot (using Python column indices)
        pos_right = [data_raw(idx, 3+right_offset); data_raw(idx, 6+right_offset); data_raw(idx, 9+right_offset)];
        
        % Get rotation vector and convert to rotation matrix
        right_vec = [data_raw(idx, 12+right_offset), data_raw(idx, 15+right_offset), data_raw(idx, 18+right_offset)];
        right_angle = norm(right_vec);
        if right_angle > 0
            right_axis = right_vec / right_angle;
            rot_right = axang2rotm([right_axis, right_angle]);
        else
            rot_right = eye(4);
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
            fprintf('Processing %d of %d\n', i, num_frames);
        end
    end
    
    % Check for keyframes
    [filepath, filename, ~] = fileparts(motion_file);
    keyframe_file = fullfile(filepath, [filename, '_keyframes.csv']);
    
    try
        keyframes = readtable(keyframe_file);
        keyframe_indices = keyframes.original_index;
        keyframe_names = keyframes.extraction_method;
        use_keyframes = true;
        fprintf('Loaded %d keyframes\n', length(keyframe_indices));
    catch e
        use_keyframes = false;
        keyframe_indices = [];
        keyframe_names = {};
        fprintf('No keyframes found: %s\nUsing all frames\n', e.message);
    end
end
