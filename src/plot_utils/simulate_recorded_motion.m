% Script to simulate recorded robot motion from CSV data
clear; clc; close all;

% Setup robots
% Setup first robot (UR3e left)
parameters(0, 1);
robot_left  = robot;
config_left = config;
Trf_0_l     = Trf_0;

% Setup second robot (UR3e right)
parameters(0, 2);
robot_right  = robot;
config_right = config;
Trf_0_r      = Trf_0;

% Create environment
figure('Name', 'Robot Motion Simulation', 'Position', [100, 100, 1200, 800]);
hold on; grid on;

tableParams.height = tableHeight;
tableParams.width = tableWidth;
tableParams.length = tableLength;

axs = create_environment(tablePosition, tableParams);

%{
    Master
        x_raw  dx_res  f_res
    x   2       3       4
    y   5       6       7
    z   8       9       10
    rx  11      12      13
    ry  14      15      16
    rz  17      18      19

    Slave
        x_raw  dx_res  f_res
    x   20       21     22
    y   23       24     25
    z   26       27     28
    rx  29       30     31
    ry  32       33     34
    rz  35       36     37

Read and prepare the CSV file with motion data
Old header format: control_mode, timestamp, 
Robot 1: x1, xdot1, fx1, y1, ydot1, fy1, z1, zdot1, fz1, rx1, rx_dot1, tau_x1, ry1, ry_dot1, tau_y1, rz1, rz_dot1, tau_z1, 
Robot 2: x2, xdot2, fx2, y2, ydot2, fy2, z2, zdot2, fz2, rx2, rx_dot2, tau_x2, ry2, ry_dot2, tau_y2, rz2, rz_dot2, tau_z2, 
Robot 3: x3, xdot3, fx3, y3, ydot3, fy3, z3, zdot3, fz3, rx3, rx_dot3, tau_x3, ry3, ry_dot3, tau_y3, rz3, rz_dot3, tau_z3, 
Robot 4: x4, xdot4, fx4, y4, ydot4, fy4, z4, zdot4, fz4, rx4, rx_dot4, tau_x4, ry4, ry_dot4, tau_y4, rz4, rz_dot4, tau_z4
control_type, timestamp, x1, xdot1, fx1, y1, ydot1, fy1, z1, zdot1, fz1, rx1, rx_dot1, tau_x1, ry1, ry_dot1, tau_y1, rz1, rz_dot1, tau_z1, x2, xdot2, fx2, y2, ydot2, fy2, z2, zdot2, fz2, rx2, rx_dot2, tau_x2, ry2, ry_dot2, tau_y2, rz2, rz_dot2, tau_z2, x3, xdot3, fx3, y3, ydot3, fy3, z3, zdot3, fz3, rx3, rx_dot3, tau_x3, ry3, ry_dot3, tau_y3, rz3, rz_dot3, tau_z3, x4, xdot4, fx4, y4, ydot4, fy4, z4, zdot4, fz4, rx4, rx_dot4, tau_x4, ry4, ry_dot4, tau_y4, rz4, rz_dot4, tau_z4
New header format: control_mode, timestamp, 
Robot 1: x1, xdot1, fx1, y1, ydot1, fy1, z1, zdot1, fz1, rx1, rx_dot1, tau_x1, ry1, ry_dot1, tau_y1, rz1, rz_dot1, tau_z1, q1_1, qdot1_1, tau1_1, q2_1, qdot2_1, tau2_1, q3_1, qdot3_1, tau3_1, q4_1, qdot4_1, tau4_1, q5_1, qdot5_1, tau5_1, q6_1, qdot6_1, tau6_1, 
Robot 2: x2, xdot2, fx2, y2, ydot2, fy2, z2, zdot2, fz2, rx2, rx_dot2, tau_x2, ry2, ry_dot2, tau_y2, rz2, rz_dot2, tau_z2, q1_2, qdot1_2, tau1_2, q2_2, qdot2_2, tau2_2, q3_2, qdot3_2, tau3_2, q4_2, qdot4_2, tau4_2, q5_2, qdot5_2, tau5_2, q6_2, qdot6_2, tau6_2,
Robot 3: x3, xdot3, fx3, y3, ydot3, fy3, z3, zdot3, fz3, rx3, rx_dot3, tau_x3, ry3, ry_dot3, tau_y3, rz3, rz_dot3, tau_z3, q1_3, qdot1_3, tau1_3, q2_3, qdot2_3, tau2_3, q3_3, qdot3_3, tau3_3, q4_3, qdot4_3, tau4_3, q5_3, qdot5_3, tau5_3, q6_3, qdot6_3, tau6_3,
Robot 4: x4, xdot4, fx4, y4, ydot4, fy4, z4, zdot4, fz4, rx4, rx_dot4, tau_x4, ry4, ry_dot4, tau_y4, rz4, rz_dot4, tau_z4, q1_4, qdot1_4, tau1_4, q2_4, qdot2_4, tau2_4, q3_4, qdot3_4, tau3_4, q4_4, qdot4_4, tau4_4, q5_4, qdot5_4, tau5_4, q6_4, qdot6_4, tau6_4
%}

% File path
root = '/Users/danielecarraro/Documents/VSCODE/data/';
csv_file = 'data/test_motion.csv';
% csv_file = 'data/1750352390_motion_unbalanced_good';

% Load data with column headers
data = readtable([root, csv_file]);
    
% Create time index for new format
time = (0:height(data)-1)'/1000000.0;

% Always have 4 robots (2 leader-follower pairs)
num_robots = 4;
left = 4;
right = 2;

% Define column mappings based on format
robot_cols = struct();

% Create column mappings for each robot with named columns
for i = 1:num_robots
    robot_cols(i).x = sprintf('x%d', i);
    robot_cols(i).y = sprintf('y%d', i);
    robot_cols(i).z = sprintf('z%d', i);
    robot_cols(i).rx = sprintf('rx%d', i);
    robot_cols(i).ry = sprintf('ry%d', i);
    robot_cols(i).rz = sprintf('rz%d', i);
    robot_cols(i).xdot = sprintf('xdot%d', i);
    robot_cols(i).ydot = sprintf('ydot%d', i);
    robot_cols(i).zdot = sprintf('zdot%d', i);
    robot_cols(i).rx_dot = sprintf('rx_dot%d', i);
    robot_cols(i).ry_dot = sprintf('ry_dot%d', i);
    robot_cols(i).rz_dot = sprintf('rz_dot%d', i);
    robot_cols(i).fx = sprintf('fx%d', i);
    robot_cols(i).fy = sprintf('fy%d', i);
    robot_cols(i).fz = sprintf('fz%d', i);
    robot_cols(i).tau_x = sprintf('tau_x%d', i);
    robot_cols(i).tau_y = sprintf('tau_y%d', i);
    robot_cols(i).tau_z = sprintf('tau_z%d', i);
end

% Initial joint configurations
q0_left  = [pi/2, -pi/3, 2*pi/3, -pi/3, pi/2, 0];
q0_right = [-pi/2, -2*pi/3, -2*pi/3, -2*pi/3, -pi/2, 0];

% Set initial robot configurations
config_left  = set_robot_configuration(q0_left,  config_left);
config_right = set_robot_configuration(q0_right, config_right);

% Show initial robot configurations
show(robot_left,  config_left,  "Visuals", "on", "Frames", "on", "FastUpdate", true, "PreservePlot", false, "Parent", axs);
show(robot_right, config_right, "Visuals", "on", "Frames", "on", "FastUpdate", true, "PreservePlot", false, "Parent", axs);

% Add text labels for each robot
left_base_pos  = Trf_0_l(1:3,4);
right_base_pos = Trf_0_r(1:3,4);
text(left_base_pos(1), left_base_pos(2), tableHeight + 0.01, ['Robot Left (', num2str(left), ')'], ...
    'Color', 'r', 'FontSize', 12, 'FontWeight', 'bold', 'HorizontalAlignment', 'center', 'Parent', axs);
text(right_base_pos(1), right_base_pos(2), tableHeight + 0.01, ['Robot Right (', num2str(right), ')'], ...
    'Color', 'b', 'FontSize', 12, 'FontWeight', 'bold', 'HorizontalAlignment', 'center', 'Parent', axs);

% Set view angle
view(90, 30);

% Prepare for animation
disp('Starting motion simulation...');

% Downsample data for smoother visualization
step = 50;  % Controls simulation speed
num_frames = floor(height(data)/step);

% Preallocate arrays for joint configurations
q_left_all  = zeros(num_frames, 6);
q_right_all = zeros(num_frames, 6);

q_left  = q0_left;
q_right = q0_right;

[~, Te_l] = direct_kinematics( q_left,  1 );
[~, Te_r] = direct_kinematics( q_right, 2 );

% Compute joint configurations for each frame
for i = 2:num_frames
    idx = (i-1)*step + 1;
    
    % Extract pose data for robot 1 (left)
    xl_val = data.(robot_cols(left).x)(idx);
    yl_val = data.(robot_cols(left).y)(idx);
    zl_val = data.(robot_cols(left).z)(idx);
    rxl_val = data.(robot_cols(left).rx)(idx);
    ryl_val = data.(robot_cols(left).ry)(idx);
    rzl_val = data.(robot_cols(left).rz)(idx);
    
    xr_val = data.(robot_cols(right).x)(idx);
    yr_val = data.(robot_cols(right).y)(idx);
    zr_val = data.(robot_cols(right).z)(idx);
    rxr_val = data.(robot_cols(right).rx)(idx);
    ryr_val = data.(robot_cols(right).ry)(idx);
    rzr_val = data.(robot_cols(right).rz)(idx);
    
    pos_rel_l = [xl_val; yl_val; zl_val];
    pos_left = Te_l(1:3,4) + pos_rel_l;
    left_vec = [rxl_val, -rzl_val, ryl_val];
    left_angle = norm(left_vec);
    if left_angle > 0
        left_axis = left_vec / left_angle;
        rot_left = Te_l(1:3,1:3) * axang2rotm([left_axis, left_angle]);
    else
        rot_left = Te_l(1:3,1:3) * eye(3);
    end
    
    % Extract pose data for robot 3 (right)
    pos_rel_r = [xr_val; yr_val; zr_val];
    pos_right = Te_r(1:3,4) + pos_rel_r;
    right_vec = [rxr_val, rzr_val, -ryr_val];
    right_angle = norm(right_vec);
    if right_angle > 0
        right_axis = right_vec / right_angle;
        rot_right = Te_r(1:3,1:3) * axang2rotm([right_axis, right_angle]);
    else
        rot_right = Te_r(1:3,1:3) * eye(3);
    end
    
    % Compute inverse kinematics
    Hl  = UR5_inverse_kinematics_cpp( pos_left,  rot_left,  AL, A, D );
    Hr  = UR5_inverse_kinematics_cpp( pos_right, rot_right, AL, A, D );
    q_left  = get_closer( Hl, q_left );
    q_right = get_closer( Hr, q_right );

    % Store joint configurations
    q_left_all(i,:)  = q_left;
    q_right_all(i,:) = q_right;
    
    % Display progress
    if mod(i, step) == 0
        fprintf('Computing IK: %d/%d frames\n', i, num_frames);
    end

end

pause();

% Load keyframe indices from the same directory as the input file with _keyframes.csv suffix
[filepath, filename, ~] = fileparts([root, csv_file]);
keyframe_file = fullfile(filepath, [filename, '_keyframes.csv']);

try
    keyframes = readtable(keyframe_file);
    keyframe_indices = keyframes.original_index;
    use_keyframes = true;
    fprintf('Loaded %d keyframes from %s\n', length(keyframe_indices), keyframe_file);
    
    % Display extraction methods if available
    if ismember('extraction_method', keyframes.Properties.VariableNames)
        methods = unique(keyframes.extraction_method);
        fprintf('Keyframe extraction methods: %s\n', strjoin(methods, ', '));
    end
catch
    use_keyframes = false;
    fprintf('Keyframe file not found at %s, using all frames\n', keyframe_file);
end

% Animate the motion
if use_keyframes
    % Only animate keyframes
    for i = 1:length(keyframe_indices)
        % Convert original CSV index to downsampled index
        orig_idx = keyframe_indices(i);
        downsampled_idx = floor(orig_idx / step) + 1;
        
        % Handle the last frame specially to avoid rounding issues
        if orig_idx >= height(data) - 1
            downsampled_idx = num_frames;  % Use the last available frame
        end
        
        if downsampled_idx > size(q_left_all, 1)
            fprintf('Warning: Keyframe index %d (original: %d) exceeds available frames, using last available frame\n', downsampled_idx, orig_idx);
            downsampled_idx = size(q_left_all, 1);  % Use the last available frame instead of skipping
        end
        
        % Update robot configurations
        config_left  = set_robot_configuration(q_left_all(downsampled_idx,:),  config_left);
        config_right = set_robot_configuration(q_right_all(downsampled_idx,:), config_right);
        
        % Show robots
        show(robot_left,  config_left,  "Visuals", "on", "Frames", "on", "FastUpdate", true, "PreservePlot", false, "Parent", axs);
        show(robot_right, config_right, "Visuals", "on", "Frames", "on", "FastUpdate", true, "PreservePlot", false, "Parent", axs);
        
        % Update title with frame info
        title_text = sprintf('Robot Motion Simulation - Keyframe %d/%d (Original Frame %d)', i, length(keyframe_indices), orig_idx);
        
        % Add extraction method if available
        if ismember('extraction_method', keyframes.Properties.VariableNames)
            method = keyframes.extraction_method{i};
            title_text = [title_text, sprintf(' - Method: %s', method)];
        end
        
        title(title_text, 'FontSize', 14);
        
        % Pause to control animation speed
        pause(1); % Longer pause for keyframes to better see each pose
        
        % Break if figure is closed
        if ~ishandle(axs.Parent)
            break;
        end
    end
else
    % Animate all frames
    for i = 1:num_frames
        % Update robot configurations
        config_left  = set_robot_configuration(q_left_all(i,:),  config_left);
        config_right = set_robot_configuration(q_right_all(i,:), config_right);
        
        % Show robots
        show(robot_left,  config_left,  "Visuals", "on", "Frames", "on", "FastUpdate", true, "PreservePlot", false, "Parent", axs);
        show(robot_right, config_right, "Visuals", "on", "Frames", "on", "FastUpdate", true, "PreservePlot", false, "Parent", axs);
        
        % Update title with frame info
        title_text = sprintf('Robot Motion Simulation - Frame %d/%d', i, num_frames);
        title(title_text, 'FontSize', 14);
        
        % Pause to control animation speed
        pause(0.001);
        
        % Break if figure is closed
        if ~ishandle(axs.Parent)
            break;
        end
    end
end

disp('Motion simulation completed using inverse kinematics.');

% Display summary of data
fprintf('Data summary:\n');
fprintf('  - Total frames: %d\n', height(data));
if use_keyframes
    fprintf('  - Simulated frames: %d\n', length(keyframe_indices));
else
    fprintf('  - Simulated frames: %d\n', num_frames);
end
