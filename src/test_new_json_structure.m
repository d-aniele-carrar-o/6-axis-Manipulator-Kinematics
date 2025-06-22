% Test script for new JSON structure
clc; close all;

timestamp = '25-06-21-04-58-07';
augmentation_id = 0;

fprintf('Testing new JSON structure...\n');

simple_traj_trans = false;

if simple_traj_trans
    try
        transform_trajectory(timestamp, augmentation_id);
        fprintf('✓ Basic transformation successful\n');
    catch ME
        fprintf('✗ Basic transformation failed: %s\n', ME.message);
    end

    disp("Press any key to continue")
    pause()

else
    % Test advanced transformation
    options.visualize = false;
    options.interaction_threshold = 0.25;
    options.simulate = true;

    try
        advanced_trajectory_transform(timestamp, augmentation_id, options);
        fprintf('✓ Advanced transformation successful\n');
    catch ME
        fprintf('✗ Advanced transformation failed: %s\n', ME.message);
    end

end
