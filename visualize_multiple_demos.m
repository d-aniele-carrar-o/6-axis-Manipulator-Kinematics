%% Visualize Multiple Demonstrations from JSON
% This script visualizes multiple demonstrations from a JSON file
% with interactive navigation between demonstrations

% Add all project directories to the path
addpath(genpath('src'));

% Path to the JSON file
json_file_path = '/Users/danielecarraro/Documents/VSCODE/data/output/yoto/25-06-24-21-30-34_demo.json';

% Check if file exists
if ~exist(json_file_path, 'file')
    error('JSON file not found: %s\nPlease update the path in this script.', json_file_path);
end

% Call the visualization function
visualize_multiple_demos(json_file_path);

fprintf('Visualizing multiple demonstrations from: %s\n', json_file_path);
fprintf('Controls:\n');
fprintf('  Right Arrow / Space - Next demonstration\n');
fprintf('  Left Arrow          - Previous demonstration\n');
fprintf('  Escape              - Exit visualization\n');