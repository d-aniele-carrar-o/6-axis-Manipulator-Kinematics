%% Visualize Demo JSON Data
% This script visualizes multiple demonstrations from a JSON file

% Add all project directories to the path
addpath(genpath('src'));

% Path to the JSON file
json_file_path = '/Users/danielecarraro/Documents/VSCODE/data/output/yoto/25-06-24-21-30-34_demo.json';

% Check if file exists
if ~exist(json_file_path, 'file')
    error('JSON file not found: %s\nPlease update the path in this script.', json_file_path);
end

% Call the enhanced visualization function for multiple demonstrations
visualize_multiple_demos(json_file_path);

fprintf('Visualizing multiple demonstrations from: %s\n', json_file_path);