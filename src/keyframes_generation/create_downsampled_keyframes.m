% Script to create a downsampled version of keyframe indices
% This is useful when your simulation uses a step size for downsampling

% Parameters
step = 50;  % Must match the step size in your simulation script

% Load original keyframe indices
keyframes = readtable('keyframe_indices.csv');
original_indices = keyframes.keyframe_idx;

% Convert to downsampled indices
downsampled_indices = floor(original_indices / step) + 1;

% Remove duplicates (multiple keyframes might map to the same downsampled frame)
downsampled_indices = unique(downsampled_indices);

% Save to a new CSV file
downsampled_keyframes = table(downsampled_indices, 'VariableNames', {'keyframe_idx'});
writetable(downsampled_keyframes, 'keyframe_indices_downsampled.csv');

fprintf('Created downsampled keyframe indices: %d frames\n', height(downsampled_keyframes));
fprintf('Original keyframes: %d, Downsampled keyframes: %d\n', ...
    length(original_indices), length(downsampled_indices));