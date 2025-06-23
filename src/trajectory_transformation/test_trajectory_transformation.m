% Test script for trajectory transformation
clear; clc; close all;

% Run the project setup
run_project;

% Test parameters
timestamp = '25-06-21-04-58-07';
augmentation_id = 1;  % Try different IDs (1-10)

fprintf('Testing trajectory transformation...\n');
fprintf('Timestamp: %s\n', timestamp);
fprintf('Augmentation ID: %d\n', augmentation_id);

try
    % Transform trajectory
    transform_trajectory(timestamp, augmentation_id);
    
    fprintf('\nTrajectory transformation completed successfully!\n');
    fprintf('Check the visualization to see original vs transformed trajectories.\n');
    
catch ME
    fprintf('Error during trajectory transformation:\n');
    fprintf('%s\n', ME.message);
    
    % Try to provide helpful debugging info
    parameters(1);
    json_file = fullfile(augmented_demos_path, timestamp, sprintf('augmented_demos_%s.json', timestamp));
    
    if ~exist(json_file, 'file')
        fprintf('\nDebugging info:\n');
        fprintf('JSON file not found: %s\n', json_file);
        fprintf('Available augmented demo folders:\n');
        folders = dir(augmented_demos_path);
        for i = 1:length(folders)
            if folders(i).isdir && ~startsWith(folders(i).name, '.')
                fprintf('  %s\n', folders(i).name);
            end
        end
    end
end
