% Simple dual robot setup with camera calibration
clear; clc; close all;

% Configuration flags
use_recorded_motion = true;  % Set to false to use automatic grasp generation

% Setup robots
parameters(0, 1);
robot_left = robot; config_left = config; Trf_0_l = Trf_0;
parameters(0, 2);
robot_right = robot; config_right = config; Trf_0_r = Trf_0;

% Load segmented scene
timestamp = '25-06-21-04-58-07';
[scenePcdPath, objectPcdPath] = find_pointcloud_files(timestamp);

fprintf('Scene path: %s\n', scenePcdPath);
fprintf('Object path: %s\n', objectPcdPath);

if ~exist(scenePcdPath, 'file') || ~exist(objectPcdPath, 'file')
    error('Point cloud files not found. Check timestamp and file locations.');
end

fprintf('Loading point clouds...\n');
ptCloudScene  = pcread(scenePcdPath);
ptCloudObject = pcread(objectPcdPath);

% Check if scene is already transformed (table at correct height)
table_z_median = median(ptCloudScene.Location(:,3));
fprintf('Table median Z: %.3f m\n', table_z_median);

if abs(table_z_median - tableHeight) > 0.1
    % Scene needs transformation
    fprintf('Scene in camera coordinates, applying calibration...\n');
    tform_cam_to_world = load_camera_calibration();
    ptCloudScene  = pctransform(ptCloudScene, tform_cam_to_world);
    ptCloudObject = pctransform(ptCloudObject, tform_cam_to_world);
    fprintf('Scene transformed to world coordinates\n');
else
    fprintf('Scene already in world coordinates\n');
    tform_cam_to_world = [];
end

% Create environment
figure('Name', 'Dual Robot Simulation');
hold on; grid on;
tableParams.height = tableHeight;
tableParams.width  = tableWidth;
tableParams.length = tableLength;
axs = create_environment(tablePosition, tableParams);

% Add camera if available
if ~isempty(tform_cam_to_world)
    plotCamera('AbsolutePose', tform_cam_to_world, 'Size', 0.1, 'Color', 'b', 'Opacity', 0.7, 'Parent', axs);
end

% Visualize point clouds
pcshow(ptCloudScene.Location, [0.7 0.7 0.7], 'MarkerSize', 10, 'Parent', axs);
pcshow(ptCloudObject.Location, 'r', 'MarkerSize', 30, 'Parent', axs);

% Setup robots
q0_left  = [pi/2, -pi/3, 2*pi/3, -pi/3, pi/2, 0];
q0_right = [-pi/2, -2*pi/3, -2*pi/3, -2*pi/3, -pi/2, 0];

config_left  = set_robot_configuration(q0_left, config_left);
config_right = set_robot_configuration(q0_right, config_right);

show(robot_left, config_left, "Visuals", "on", "Frames", "on", "FastUpdate", true, "PreservePlot", false, "Parent", axs);
show(robot_right, config_right, "Visuals", "on", "Frames", "on", "FastUpdate", true, "PreservePlot", false, "Parent", axs);

% Compute end-effector poses
parameters(1, 1); [Te_w_e_left, Te_l]  = direct_kinematics(q0_left, 1);
parameters(1, 2); [Te_w_e_right, Te_r] = direct_kinematics(q0_right, 2);

if use_recorded_motion
    % Load recorded motion data
    fprintf('Loading recorded motion data...\n');
    motion_file = find_closest_motion_file(timestamp);
    if isempty(motion_file)
        fprintf('No motion data found, falling back to automatic grasp generation\n');
        use_recorded_motion = false;
    else
        fprintf('Using motion file: %s\n', motion_file);
        [q_left_all, q_right_all, use_keyframes, keyframe_indices] = load_motion_data(motion_file, q0_left, q0_right, Te_l, Te_r);
    end
end

if ~use_recorded_motion
    % Find grasp points
    fprintf('Computing grasp points...\n');
    [grasp_points, grasp_orientations] = find_object_grasp_points(ptCloudObject, Te_w_e_left, Te_w_e_right);
    
    % Visualize grasp points
    scatter3(grasp_points(1,1), grasp_points(2,1), grasp_points(3,1), 100, 'g*', 'Parent', axs);
    scatter3(grasp_points(1,2), grasp_points(2,2), grasp_points(3,2), 100, 'g*', 'Parent', axs);
    
    % Create trajectories
    fprintf('Planning trajectories...\n');
    
    % Grasp poses with approach offset
    approach_offset = 0.05;
    R_grasp_l = squeeze(grasp_orientations(:,:,1));
    R_grasp_r = squeeze(grasp_orientations(:,:,2));
    
    % Approach positions
    pf_l = grasp_points(:,1) - approach_offset * R_grasp_l(:,3);
    pf_r = grasp_points(:,2) - approach_offset * R_grasp_r(:,3);
    T_w_o_l = [R_grasp_l, pf_l; 0,0,0,1];
    T_w_o_r = [R_grasp_r, pf_r; 0,0,0,1];
    Tf_l = Trf_0_l \ T_w_o_l;
    Tf_r = Trf_0_r \ T_w_o_r;
    
    % Grasp positions
    pf2_l = grasp_points(:,1);
    pf2_r = grasp_points(:,2);
    T2_w_o_l = [R_grasp_l, pf2_l; 0,0,0,1];
    T2_w_o_r = [R_grasp_r, pf2_r; 0,0,0,1];
    Tf2_l = Trf_0_l \ T2_w_o_l;
    Tf2_r = Trf_0_r \ T2_w_o_r;
    
    % Lift positions
    pf3_l = grasp_points(:,1) + [0; 0; 0.2];
    pf3_r = grasp_points(:,2) + [0; 0; 0.2];
    T3_w_o_l = [R_grasp_l, pf3_l; 0,0,0,1];
    T3_w_o_r = [R_grasp_r, pf3_r; 0,0,0,1];
    Tf3_l = Trf_0_l \ T3_w_o_l;
    Tf3_r = Trf_0_r \ T3_w_o_r;
    
    % Generate trajectories
    viapoints_l = [Tf_l; Tf2_l; Tf3_l];
    viapoints_r = [Tf_r; Tf2_r; Tf3_r];
    times = [2, 2, 2];
    
    [t_l, p_l, v_l] = multipoint_trajectory(q0_left, viapoints_l, times, 1);
    [t_r, p_r, v_r] = multipoint_trajectory(q0_right, viapoints_r, times, 2);
    
    % Run simulation
    fprintf('Running simulation...\n');
    view(90, 30);
    pause(1);
    
    simulate_dual({robot_left, robot_right}, {config_left, config_right}, ...
                  {Trf_0_l, Trf_0_r}, {t_l, t_r}, {p_l, p_r}, {v_l, v_r}, axs);
else
    % Run recorded motion simulation
    fprintf('Running recorded motion simulation...\n');
    view(90, 30);
    pause(1);
    
    simulate_recorded_motion_dual(robot_left, robot_right, config_left, config_right, ...
                                  q_left_all, q_right_all, use_keyframes, keyframe_indices, axs);
end

fprintf('Simulation complete!\n');

% Helper functions
function motion_file = find_closest_motion_file(target_timestamp)
    data_folder = '/Users/danielecarraro/Documents/VSCODE/data/';
    files = dir(fullfile(data_folder, '**', '*_motion*'));
    
    if isempty(files)
        motion_file = '';
        return;
    end
    
    target_time = parse_timestamp(target_timestamp);
    if isnan(target_time)
        motion_file = '';
        return;
    end
    
    best_diff = inf;
    motion_file = '';
    for i = 1:length(files)
        file_timestamp = extract_timestamp_from_filename(files(i).name);
        if ~isempty(file_timestamp)
            file_time = parse_timestamp(file_timestamp);
            if ~isnan(file_time)
                diff = abs(file_time - target_time);
                if diff < best_diff
                    best_diff = diff;
                    motion_file = fullfile(files(i).folder, files(i).name);
                end
            end
        end
    end
end

function timestamp_str = extract_timestamp_from_filename(filename)
    % Extract YY-MM-DD-hh-mm pattern from filename
    pattern = '\d{2}-\d{2}-\d{2}-\d{2}-\d{2}';
    match = regexp(filename, pattern, 'match');
    if ~isempty(match)
        timestamp_str = [match{1}, '-00']; % Add seconds
    else
        timestamp_str = '';
    end
end

function time_val = parse_timestamp(timestamp_str)
    try
        dt = datetime(timestamp_str, 'InputFormat', 'yy-MM-dd-HH-mm-ss');
        time_val = posixtime(dt);
    catch
        time_val = NaN;
    end
end

function [scenePath, objectPath] = find_pointcloud_files(timestamp)
    data_folder = '/Users/danielecarraro/Documents/VSCODE/data/';
    
    % Search in data folder
    scene_files = dir(fullfile(data_folder, '**', '*table*.ply'));
    object_files = dir(fullfile(data_folder, '**', '*object*.ply'));
    
    scenePath = '';
    objectPath = '';
    
    fprintf('Found %d table files, %d object files\n', length(scene_files), length(object_files));
    
    % Find files with matching timestamp
    for i = 1:length(scene_files)
        if contains(scene_files(i).name, timestamp) || contains(scene_files(i).folder, timestamp)
            scenePath = fullfile(scene_files(i).folder, scene_files(i).name);
            break;
        end
    end
    
    for i = 1:length(object_files)
        if contains(object_files(i).name, timestamp) || contains(object_files(i).folder, timestamp)
            objectPath = fullfile(object_files(i).folder, object_files(i).name);
            break;
        end
    end
end

function [q_left_all, q_right_all, use_keyframes, keyframe_indices] = load_motion_data(motion_file, q0_left, q0_right, Te_l, Te_r)
    data = readtable(motion_file);
    
    % Robot mappings
    left = 4; right = 2;
    
    % Create column mappings
    robot_cols = struct();
    for i = 1:4
        robot_cols(i).x = sprintf('x%d', i);
        robot_cols(i).y = sprintf('y%d', i);
        robot_cols(i).z = sprintf('z%d', i);
        robot_cols(i).rx = sprintf('rx%d', i);
        robot_cols(i).ry = sprintf('ry%d', i);
        robot_cols(i).rz = sprintf('rz%d', i);
    end
    
    % Downsample data
    step = 50;
    num_frames = floor(height(data)/step);
    
    q_left_all = zeros(num_frames, 6);
    q_right_all = zeros(num_frames, 6);
    
    q_left = q0_left;
    q_right = q0_right;
    
    % Compute joint configurations
    for i = 2:num_frames
        idx = (i-1)*step + 1;
        
        % Left robot
        pos_rel_l = [data.(robot_cols(left).x)(idx); data.(robot_cols(left).y)(idx); data.(robot_cols(left).z)(idx)];
        pos_left = Te_l(1:3,4) + pos_rel_l;
        left_vec = [data.(robot_cols(left).rx)(idx), data.(robot_cols(left).ry)(idx), data.(robot_cols(left).rz)(idx)];
        left_angle = norm(left_vec);
        rot_left = Te_l(1:3,1:3) * axang2rotm([left_vec/left_angle, left_angle]);
        
        % Right robot
        pos_rel_r = [data.(robot_cols(right).x)(idx); data.(robot_cols(right).y)(idx); data.(robot_cols(right).z)(idx)];
        pos_right = Te_r(1:3,4) + pos_rel_r;
        right_vec = [data.(robot_cols(right).rx)(idx), data.(robot_cols(right).rz)(idx), -data.(robot_cols(right).ry)(idx)];
        right_angle = norm(right_vec);
        if right_angle > 0
            rot_right = Te_r(1:3,1:3) * axang2rotm([right_vec/right_angle, right_angle]);
        else
            rot_right = Te_r(1:3,1:3);
        end
        
        % Inverse kinematics
        parameters( 1, 1 )
        Hl = UR5_inverse_kinematics_cpp(pos_left, rot_left, AL, A, D);
        parameters( 1, 2 )
        Hr = UR5_inverse_kinematics_cpp(pos_right, rot_right, AL, A, D);
        q_left = get_closer(Hl, q_left);
        q_right = get_closer(Hr, q_right);
        
        q_left_all(i,:) = q_left;
        q_right_all(i,:) = q_right;
    end
    
    % Check for keyframes
    [filepath, filename, ~] = fileparts(motion_file);
    keyframe_file = fullfile(filepath, [filename, '_keyframes.csv']);
    
    try
        keyframes = readtable(keyframe_file);
        keyframe_indices = keyframes.original_index;
        use_keyframes = true;
        fprintf('Loaded %d keyframes\n', length(keyframe_indices));
    catch
        use_keyframes = false;
        keyframe_indices = [];
        fprintf('No keyframes found, using all frames\n');
    end
end

function simulate_recorded_motion_dual(robot_left, robot_right, config_left, config_right, q_left_all, q_right_all, use_keyframes, keyframe_indices, axs)
    if use_keyframes
        step = 50;
        for i = 1:length(keyframe_indices)
            orig_idx = keyframe_indices(i);
            downsampled_idx = min(floor(orig_idx / step) + 1, size(q_left_all, 1));
            
            config_left = set_robot_configuration(q_left_all(downsampled_idx,:), config_left);
            config_right = set_robot_configuration(q_right_all(downsampled_idx,:), config_right);
            
            show(robot_left, config_left, "Visuals", "on", "Frames", "on", "FastUpdate", true, "PreservePlot", false, "Parent", axs);
            show(robot_right, config_right, "Visuals", "on", "Frames", "on", "FastUpdate", true, "PreservePlot", false, "Parent", axs);
            
            title(sprintf('Recorded Motion - Keyframe %d/%d', i, length(keyframe_indices)), 'FontSize', 14);
            pause(1);
            
            if ~ishandle(axs.Parent)
                break;
            end
        end
    else
        for i = 1:size(q_left_all, 1)
            config_left = set_robot_configuration(q_left_all(i,:), config_left);
            config_right = set_robot_configuration(q_right_all(i,:), config_right);
            
            show(robot_left, config_left, "Visuals", "on", "Frames", "on", "FastUpdate", true, "PreservePlot", false, "Parent", axs);
            show(robot_right, config_right, "Visuals", "on", "Frames", "on", "FastUpdate", true, "PreservePlot", false, "Parent", axs);
            
            title(sprintf('Recorded Motion - Frame %d/%d', i, size(q_left_all, 1)), 'FontSize', 14);
            pause(0.001);
            
            if ~ishandle(axs.Parent)
                break;
            end
        end
    end
end
