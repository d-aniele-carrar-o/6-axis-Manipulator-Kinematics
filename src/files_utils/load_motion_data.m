function [q_l_all, q_r_all, poses_l_all, poses_r_all, keyframes_data] = ...
            load_motion_data(motion_file, q0_left, q0_right, step)
    % LOAD_MOTION_DATA Load and process motion data from CSV file
    %
    % Parameters:
    %   motion_file - Path to the CSV file containing motion data
    %   q0_left     - Initial joint configuration for left robot [1x6] (default: [pi/2, -pi/3, 2*pi/3, -pi/3, pi/2, 0])
    %   q0_right    - Initial joint configuration for right robot [1x6] (default: [-pi/2, -2*pi/3, -2*pi/3, -2*pi/3, -pi/2, 0])
    %   step        - Downsampling step (default: 50)
    %
    % Returns:
    %   q_l_all             - Joint configurations for left robot [Nx6] (includes keyframes)
    %   q_r_all             - Joint configurations for right robot [Nx6] (includes keyframes)
    %   poses_l_all         - End-effector poses for left robot [Nx4x4] (includes keyframes)
    %   poses_r_all         - End-effector poses for right robot [Nx4x4] (includes keyframes)
    %   keyframes_data      - Structure containing keyframe information with fields:
    %     .available        - Boolean indicating if keyframes are available
    %     .indices          - Indices of keyframes in the trajectory arrays (not original data)
    %     .names            - Names of keyframes
    %     .original_indices - Original indices in the raw data
    parameters(1);

    if nargin < 2
        q0_left  = [pi/2, -pi/3, 2*pi/3, -pi/3, pi/2, 0];
        q0_right = [-pi/2, -2*pi/3, -2*pi/3, -2*pi/3, -pi/2, 0];
    end
    if nargin < 4, step = 50; end
    
    fprintf('Loading motion data from %s\n', motion_file);
    
    % Read raw data without headers
    opts = detectImportOptions(motion_file, 'NumHeaderLines', 0);
    data_raw = readmatrix(motion_file, opts);
    
    % Process timestamps to seconds
    if size(data_raw, 2) >= 2
        data_raw(:,2) = (data_raw(:,2) - data_raw(1,2)) / 1000000.0; % Convert to seconds
    end
    
    % Define column offsets based on selected robots
    right = 2;
    left  = 4;
    right_offset = 18*(right-1);  % Right pair
    left_offset  = 18*(left-1);   % Left pair
    
    % Check for keyframes
    [filepath, filename, ~] = fileparts(motion_file);
    keyframe_file = fullfile(filepath, [filename, '_keyframes.csv']);
    
    % Initialize keyframes data structure
    keyframes_data = struct();
    keyframes_data.available = false;
    
    try
        keyframes = readtable(keyframe_file);
        keyframes_data.original_indices = keyframes.original_index;
        keyframes_data.names = cellstr(keyframes.extraction_method);
        keyframes_data.q_l = zeros(length(keyframes_data.original_indices), 6);
        keyframes_data.q_r = zeros(length(keyframes_data.original_indices), 6);
        keyframes_data.available = true;
        if verbose
            fprintf('Found %d keyframes in %s\n', length(keyframes_data.original_indices), keyframe_file);
        end
    catch e
        keyframes_data.original_indices = [];
        keyframes_data.names = {};
        keyframes_data.indices = [];
        fprintf('No keyframes found: %s\nUsing only regular frames\n', e.message);
    end
    
    % Prepare for processing
    q_l = q0_left;
    q_r = q0_right;
    
    % First pass: count how many frames we'll have (downsampled + keyframes)
    num_regular_frames = floor(size(data_raw, 1)/step);
    
    if keyframes_data.available
        % Get the raw indices of regular frames
        regular_indices = (0:num_regular_frames-1)*step + 1;
        
        % Count unique frames (avoid duplicates if a keyframe falls on a regular frame)
        all_frame_indices = unique([regular_indices, keyframes_data.original_indices']);
        num_total_frames = length(all_frame_indices);
        
        % Sort keyframes by original index for processing
        [keyframes_data.original_indices, sort_idx] = sort(keyframes_data.original_indices);
        keyframes_data.names = keyframes_data.names(sort_idx);
        
        fprintf('Processing %d frames (%d regular + %d keyframes) from %d total\n', ...
            num_total_frames, num_regular_frames, length(keyframes_data.original_indices), size(data_raw, 1));
    else
        num_total_frames = num_regular_frames;
        fprintf('Processing %d frames from %d total\n', ...
            num_total_frames, size(data_raw, 1));
    end
    
    % Preallocate output matrices
    q_l_all = zeros(num_total_frames, 6);
    q_r_all = zeros(num_total_frames, 6);
    poses_l_all = zeros(num_total_frames, 4, 4);
    poses_r_all = zeros(num_total_frames, 4, 4);
    
    % Second pass: process all frames (regular + keyframes)
    frame_counter = 1;
    keyframe_counter = 1;
    keyframes_data.indices = zeros(length(keyframes_data.original_indices), 1);
    
    % Process all frames in order
    for raw_idx = 1:size(data_raw, 1)
        is_regular_frame = mod(raw_idx-1, step) == 0;
        is_keyframe = keyframes_data.available && keyframe_counter <= length(keyframes_data.original_indices) ...
                                               && keyframes_data.original_indices(keyframe_counter)+1 == raw_idx;

        % Process this frame if it's a regular frame or a keyframe
        if is_regular_frame || is_keyframe
            % Left robot
            pos_left = [data_raw(raw_idx, 3+left_offset); data_raw(raw_idx, 6+left_offset); data_raw(raw_idx, 9+left_offset)];
            
            % Get rotation vector and convert to rotation matrix
            left_vec = [data_raw(raw_idx, 12+left_offset), data_raw(raw_idx, 15+left_offset), data_raw(raw_idx, 18+left_offset)];
            left_angle = norm(left_vec);
            if left_angle > 0
                left_axis = left_vec / left_angle;
                rot_left = axang2rotm([left_axis, left_angle]);
            else
                rot_left = eye(3);
            end
            
            % Right robot
            pos_right = [data_raw(raw_idx, 3+right_offset); data_raw(raw_idx, 6+right_offset); data_raw(raw_idx, 9+right_offset)];
            
            % Get rotation vector and convert to rotation matrix
            right_vec = [data_raw(raw_idx, 12+right_offset), data_raw(raw_idx, 15+right_offset), data_raw(raw_idx, 18+right_offset)];
            right_angle = norm(right_vec);
            if right_angle > 0
                right_axis = right_vec / right_angle;
                rot_right = axang2rotm([right_axis, right_angle]);
            else
                rot_right = eye(3);
            end
            
            % Inverse kinematics
            parameters(1, 1);
            Hl  = UR5_inverse_kinematics_cpp(pos_left, rot_left, AL, A, D);
            q_l = get_closer(Hl, q_l);
            parameters(1, 2);
            Hr  = UR5_inverse_kinematics_cpp(pos_right, rot_right, AL, A, D);
            q_r = get_closer(Hr, q_r);
            
            % Store joint configurations and poses
            q_l_all(frame_counter,:) = q_l;
            q_r_all(frame_counter,:) = q_r;
            poses_l_all(frame_counter,:,:) = [rot_left, pos_left; 0,0,0,1];
            poses_r_all(frame_counter,:,:) = [rot_right, pos_right; 0,0,0,1];
            
            % If this is a keyframe, store its trajectory index
            if is_keyframe
                keyframes_data.indices(keyframe_counter) = frame_counter;
                keyframes_data.q_l(keyframe_counter,:) = q_l;
                keyframes_data.q_r(keyframe_counter,:) = q_r;
                fprintf('  Processed keyframe %d/%d (original idx: %d, traj idx: %d)\n', ...
                        keyframe_counter, length(keyframes_data.original_indices), raw_idx, frame_counter);
                keyframe_counter = keyframe_counter + 1;
            end
            
            % Print progress for regular frames
            if is_regular_frame && mod(raw_idx, step*100) == 1 && frame_counter-1 > 0
                fprintf('Processing frame %d of %d (%.1f%%)\n', ...
                    frame_counter-1, num_total_frames, 100*(frame_counter-1)/num_total_frames);
            end

            % Increment frame counter
            frame_counter = frame_counter + 1;
        end
    end
    
    % Ensure we have the right number of frames
    if frame_counter-1 < num_total_frames
        fprintf('Warning: Expected %d frames but processed only %d\n', num_total_frames, frame_counter-1);
        q_l_all = q_l_all(1:frame_counter-1,:);
        q_r_all = q_r_all(1:frame_counter-1,:);
        poses_l_all = poses_l_all(1:frame_counter-1,:,:);
        poses_r_all = poses_r_all(1:frame_counter-1,:,:);
    end
    
    % Finalize keyframes data
    if keyframes_data.available
        % Verify all keyframes were processed
        if keyframe_counter-1 < length(keyframes_data.original_indices)
            fprintf('Warning: Expected %d keyframes but processed only %d\n', ...
                length(keyframes_data.original_indices), keyframe_counter-1);
            keyframes_data.indices = keyframes_data.indices(1:keyframe_counter-1);
            keyframes_data.original_indices = keyframes_data.original_indices(1:keyframe_counter-1);
            keyframes_data.names = keyframes_data.names(1:keyframe_counter-1);
        end
        
        fprintf('Integrated %d keyframes into trajectory with %d total frames\n', ...
            keyframe_counter-1, size(q_l_all, 1));
        
    end
    
end
