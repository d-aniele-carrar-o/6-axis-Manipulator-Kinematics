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

% Read the CSV file with motion data
% data = readtable('/Users/danielecarraro/Documents/VSCODE/master-thesis/YOTO/data/1749729939_motion.csv');
data = readtable('/Users/danielecarraro/Documents/VSCODE/master-thesis/YOTO/data/1749734485_motion.csv');

% Initial joint configurations
q0_left  = [pi/2, -pi/3, 2*pi/3, -pi/3, pi/2, 0];
q0_right = [-pi/2, -2*pi/3, -2*pi/3, -2*pi/3, -pi/2, 0];

% Set initial robot configurations
config_left  = set_robot_configuration(q0_left, config_left);
config_right = set_robot_configuration(q0_right, config_right);

% Show initial robot configurations
show(robot_left, config_left, "Visuals", "on", "Frames", "on", "FastUpdate", true, "PreservePlot", false, "Parent", axs);
show(robot_right, config_right, "Visuals", "on", "Frames", "on", "FastUpdate", true, "PreservePlot", false, "Parent", axs);

% Add text labels for each robot
left_base_pos = Trf_0_l(1:3,4);
right_base_pos = Trf_0_r(1:3,4);
text(left_base_pos(1), left_base_pos(2), tableHeight + 0.01, 'Robot Left (1)', ...
    'Color', 'r', 'FontSize', 12, 'FontWeight', 'bold', 'HorizontalAlignment', 'center', 'Parent', axs);
text(right_base_pos(1), right_base_pos(2), tableHeight + 0.01, 'Robot Right (3)', ...
    'Color', 'b', 'FontSize', 12, 'FontWeight', 'bold', 'HorizontalAlignment', 'center', 'Parent', axs);

% Set view angle
view(90, 30);

% Prepare for animation
disp('Starting motion simulation...');

% Downsample data for smoother visualization
step = 50; % Adjust this value to control simulation speed
num_frames = floor(height(data)/step);

% Preallocate arrays for joint configurations
q_left_all = zeros(num_frames, 6);
q_right_all = zeros(num_frames, 6);

q_left = q0_left;
q_right = q0_right;

[Tew_l, Te_l] = direct_kinematics( q_left, 1 );
[Tew_r, Te_r] = direct_kinematics( q_right, 2 );

% Compute inverse kinematics for each pose
for i = 1:num_frames
% for i = 1:500
    idx = (i-1)*step + 1;
    
    % Extract pose data for robot 1 (left)
    pos_rel_l = [-data.x1(idx); data.y1(idx); data.z1(idx)];
    pos_left = Te_l(1:3,4) + pos_rel_l;
    rot_left = Te_l(1:3,1:3) * eul2rotm_custom([data.rx1(idx), data.ry1(idx), data.rz1(idx)]);
    T_left = [rot_left, pos_left; 0 0 0 1];
    
    % Extract pose data for robot 3 (right)
    pos_rel_r = [-data.x3(idx); data.y3(idx); data.z3(idx)];
    pos_right = Te_r(1:3,4) + pos_rel_r;
    rot_right = Te_r(1:3,1:3) * eul2rotm_custom([data.rx3(idx), data.ry3(idx), data.rz3(idx)]);
    T_right = [rot_right, pos_right; 0 0 0 1];
   
    % Compute inverse kinematics
    Hl = UR5_inverse_kinematics_cpp( T_left(1:3,4), T_left(1:3,1:3), AL, A, D );
    Hr = UR5_inverse_kinematics_cpp( T_right(1:3,4), T_right(1:3,1:3), AL, A, D );
    q_left  = get_closer( Hl, q_left );
    q_right = get_closer( Hr, q_right );

    % Check for NaN values in q_right
    if any(isnan(q_left))
        if i > 1
            q_left = q_left_all(i-1,:);
        else
            q_left = q0_left;
        end
    end
    if any(isnan(q_right))
        if i > 1
            q_right = q_right_all(i-1,:);
        else
            q_right = q0_right;
        end
    end

    % Store joint configurations
    q_left_all(i,:)  = q_left;
    q_right_all(i,:) = q_right;
    
    % Display progress
    if mod(i, step) == 0
        fprintf('Computing IK: %d/%d frames\n', i, num_frames);
        % pause()
    end
end

pause();

% Load keyframe indices
try
    keyframes = readtable('keyframe_indices.csv');
    keyframe_indices = keyframes.keyframe_idx;
    use_keyframes = true;
    fprintf('Loaded %d keyframes for animation\n', length(keyframe_indices));
catch
    use_keyframes = false;
    fprintf('Keyframe file not found, using all frames\n');
end

% Animate the motion
if use_keyframes
    % Only animate keyframes
    for i = 1:length(keyframe_indices)
        % Convert original CSV index to downsampled index
        orig_idx = keyframe_indices(i);
        downsampled_idx = floor(orig_idx / step) + 1;
        
        if downsampled_idx > size(q_left_all, 1)
            fprintf('Warning: Keyframe index %d (original: %d) exceeds available frames, skipping\n', downsampled_idx, orig_idx);
            continue;
        end
        
        % Update robot configurations
        config_left = set_robot_configuration(q_left_all(downsampled_idx,:), config_left);
        config_right = set_robot_configuration(q_right_all(downsampled_idx,:), config_right);
        
        % Show robots
        show(robot_left, config_left, "Visuals", "on", "Frames", "on", "FastUpdate", true, "PreservePlot", false, "Parent", axs);
        show(robot_right, config_right, "Visuals", "on", "Frames", "on", "FastUpdate", true, "PreservePlot", false, "Parent", axs);
        
        % Update title with frame info
        title(sprintf('Robot Motion Simulation - Keyframe %d/%d (Original Frame %d)', i, length(keyframe_indices), orig_idx), 'FontSize', 14);
        
        % Pause to control animation speed
        pause(0.5); % Longer pause for keyframes to better see each pose
        
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
        show(robot_left, config_left, "Visuals", "on", "Frames", "on", "FastUpdate", true, "PreservePlot", false, "Parent", axs);
        show(robot_right, config_right, "Visuals", "on", "Frames", "on", "FastUpdate", true, "PreservePlot", false, "Parent", axs);
        
        % Update title with frame info
        title(sprintf('Robot Motion Simulation - Frame %d/%d', i, num_frames), 'FontSize', 14);
        
        % Pause to control animation speed
        pause(0.05);
        
        % Break if figure is closed
        if ~ishandle(axs.Parent)
            break;
        end
    end
end

disp('Motion simulation completed.');
