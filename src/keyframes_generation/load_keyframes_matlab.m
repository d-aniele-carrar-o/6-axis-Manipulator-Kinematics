% Script to load and use keyframe indices in MATLAB

% Method 1: Load from simple keyframe_indices.csv
keyframes = readtable('keyframe_indices.csv');
keyframe_indices = keyframes.keyframe_idx;

% Display the first few keyframe indices
disp('First 10 keyframe indices:');
disp(keyframe_indices(1:10));

% Method 2: Load from the CSV with keyframe column
% This loads the entire motion data with the keyframe indicator
motion_data = readtable('1749729939_motion_with_keyframes.csv');

% Extract only the keyframe rows
keyframe_rows = motion_data(motion_data.is_keyframe == 1, :);

% Display the number of keyframes
fprintf('Number of keyframes: %d\n', height(keyframe_rows));

% Example: Use keyframes in your simulation
% You can modify your existing simulation code to use these keyframes
% For example, to only animate the keyframes:

% In your simulate_recorded_motion.m script, you can add:
% 
% % Load keyframe indices
% keyframes = readtable('keyframe_indices.csv');
% keyframe_indices = keyframes.keyframe_idx + 1; % +1 for MATLAB 1-based indexing
% 
% % Then modify your animation loop to only use keyframes:
% for i = 1:length(keyframe_indices)
%     idx = keyframe_indices(i);
%     
%     % Update robot configurations using data at keyframe indices
%     config_left = set_robot_configuration(q_left_all(idx,:), config_left);
%     config_right = set_robot_configuration(q_right_all(idx,:), config_right);
%     
%     % Show robots
%     show(robot_left, config_left, "Visuals", "on", "Frames", "on", "FastUpdate", true, "PreservePlot", false, "Parent", axs);
%     show(robot_right, config_right, "Visuals", "on", "Frames", "on", "FastUpdate", true, "PreservePlot", false, "Parent", axs);
%     
%     % Update title with frame info
%     title(sprintf('Robot Motion Simulation - Keyframe %d/%d', i, length(keyframe_indices)), 'FontSize', 14);
%     
%     % Pause to control animation speed
%     pause(0.5);
%     
%     % Break if figure is closed
%     if ~ishandle(axs.Parent)
%         break;
%     end
% end