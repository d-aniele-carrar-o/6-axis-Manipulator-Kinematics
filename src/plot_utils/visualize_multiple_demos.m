function visualize_multiple_demos(json_file_path)
% VISUALIZE_MULTIPLE_DEMOS Interactive visualization of multiple demonstrations
%
% This function reads a JSON file containing multiple demonstrations and
% displays them one at a time with navigation buttons.
%
% Input:
%   json_file_path - Path to the JSON file containing multiple demonstrations

% Check if file exists
if ~exist(json_file_path, 'file')
    error('JSON file not found: %s', json_file_path);
end

% Read and parse JSON file
try
    fid = fopen(json_file_path, 'r');
    raw_data = fread(fid, inf, 'uint8=>char')';
    fclose(fid);
    data = jsondecode(raw_data);
catch e
    error('Error reading or parsing JSON file: %s', e.message);
end

if isempty(data)
    error('No demonstrations found in JSON file');
end

% Create figure with navigation controls
fig = figure('Name', 'Demo Navigator', 'Position', [100, 100, 1200, 900]);

% Create axes for 3D plot
ax = axes('Parent', fig, 'Position', [0.1, 0.15, 0.8, 0.75]);
hold(ax, 'on');
grid(ax, 'on');
axis(ax, 'equal');
view(ax, 3);
xlabel(ax, 'X');
ylabel(ax, 'Y');
zlabel(ax, 'Z');

% Create navigation buttons
btn_prev = uicontrol('Style', 'pushbutton', 'String', '◀ Previous', ...
    'Position', [50, 50, 100, 40], 'FontSize', 12);
btn_next = uicontrol('Style', 'pushbutton', 'String', 'Next ▶', ...
    'Position', [170, 50, 100, 40], 'FontSize', 12);
txt_info = uicontrol('Style', 'text', 'String', '', ...
    'Position', [300, 50, 300, 40], 'FontSize', 12, 'HorizontalAlignment', 'left');

% Store data in figure's UserData
fig.UserData = struct('data', data, 'current_demo', 1, 'ax', ax, 'txt_info', txt_info);

% Set button callbacks
set(btn_prev, 'Callback', @(src, evt) navigate_demo(fig, -1));
set(btn_next, 'Callback', @(src, evt) navigate_demo(fig, 1));

% Display first demonstration
plot_single_demo(fig, 1);

% Make plot interactive
rotate3d(ax, 'on');

fprintf('Loaded %d demonstrations. Use navigation buttons to browse.\n', length(data));
end

function navigate_demo(fig, direction)
% Navigate between demonstrations
userData = fig.UserData;
current = userData.current_demo + direction;
total = length(userData.data);

% Wrap around
if current < 1
    current = total;
elseif current > total
    current = 1;
end

userData.current_demo = current;
fig.UserData = userData;

plot_single_demo(fig, current);
end

function plot_single_demo(fig, demo_idx)
% Plot a single demonstration
userData = fig.UserData;
data = userData.data;
ax = userData.ax;
txt_info = userData.txt_info;

% Clear previous plot
cla(ax);
hold(ax, 'on');
grid(ax, 'on');
axis(ax, 'equal');
view(ax, 3);
xlabel(ax, 'X');
ylabel(ax, 'Y');
zlabel(ax, 'Z');

% Update info text
set(txt_info, 'String', sprintf('Demo %d of %d', demo_idx, length(data)));
title(ax, sprintf('Demonstration %d', demo_idx));

% Extract current demonstration
demo = data(demo_idx);

% Plot pointcloud
if isfield(demo, 'obs') && isfield(demo.obs, 'pcl')
    pcl = demo.obs.pcl;
    
    % Convert cell array to matrix if needed
    if iscell(pcl)
        pcl_matrix = zeros(length(pcl), 3);
        for j = 1:length(pcl)
            pcl_matrix(j,:) = pcl{j};
        end
    else
        pcl_matrix = pcl;
    end
    
    % Plot pointcloud
    scatter3(ax, pcl_matrix(:,1), pcl_matrix(:,2), pcl_matrix(:,3), ...
             5, 'b', 'filled', 'MarkerFaceAlpha', 0.6);
end

% Plot robot poses
if isfield(demo, 'actions')
    legend_handles = [];
    legend_entries = {};
    
    % Process left arm poses
    if isfield(demo.actions, 'left') && ~isempty(demo.actions.left)
        left_poses = demo.actions.left;
        left_positions = zeros(length(left_poses), 3);
        
        for j = 1:length(left_poses)
            trans = left_poses(j).trans;
            rot = left_poses(j).rot;
            left_positions(j,:) = trans;
            
            % Convert rotation vector to rotation matrix
            angle = norm(rot);
            if angle > 0
                axs = rot / angle;
            else
                axs = [1, 0, 0];
            end
            R = angle_axis2rotm(angle, axs);
            
            % Plot coordinate frame for left arm (red)
            plot_frame(ax, trans, R, 0.05, 'r');
            
            % Add keyframe label
            text(trans(1), trans(2), trans(3), sprintf('L%d', j), ...
                 'Color', 'r', 'FontSize', 10, 'FontWeight', 'bold', 'Parent', ax);
        end
        
        % Plot path for left arm
        h_left = plot3(ax, left_positions(:,1), left_positions(:,2), left_positions(:,3), ...
                      '-r', 'LineWidth', 3, 'MarkerSize', 8);
        legend_handles = [legend_handles, h_left];
        legend_entries{end+1} = 'Left Arm';
    end
    
    % Process right arm poses
    if isfield(demo.actions, 'right') && ~isempty(demo.actions.right)
        right_poses = demo.actions.right;
        right_positions = zeros(length(right_poses), 3);
        
        for j = 1:length(right_poses)
            trans = right_poses(j).trans;
            rot = right_poses(j).rot;
            right_positions(j,:) = trans;
            
            % Convert rotation vector to rotation matrix
            angle = norm(rot);
            if angle > 0
                axs = rot / angle;
            else
                axs = [1, 0, 0];
            end
            R = angle_axis2rotm(angle, axs);
            
            % Plot coordinate frame for right arm (green)
            plot_frame(ax, trans, R, 0.05, 'g');
            
            % Add keyframe label
            text(trans(1), trans(2), trans(3), sprintf('R%d', j), ...
                 'Color', 'g', 'FontSize', 10, 'FontWeight', 'bold', 'Parent', ax);
        end
        
        % Plot path for right arm
        h_right = plot3(ax, right_positions(:,1), right_positions(:,2), right_positions(:,3), ...
                       '--g', 'LineWidth', 3, 'MarkerSize', 8);
        legend_handles = [legend_handles, h_right];
        legend_entries{end+1} = 'Right Arm';
    end
    
    % Add pointcloud to legend if it exists
    if isfield(demo, 'obs') && isfield(demo.obs, 'pcl')
        h_pcl = scatter3(ax, NaN, NaN, NaN, 50, 'b', 'filled', 'MarkerFaceAlpha', 0.6);
        legend_handles = [h_pcl, legend_handles];
        legend_entries = [{'Pointcloud'}, legend_entries];
    end
    
    % Create legend if we have entries
    if ~isempty(legend_handles)
        legend(ax, legend_handles, legend_entries, 'Location', 'best');
    end
end
end

function plot_frame(ax, origin, R, scale, color)
% Plot a coordinate frame at the specified origin with the given rotation matrix
quiver3(ax, origin(1), origin(2), origin(3), ...
    R(1,1)*scale, R(2,1)*scale, R(3,1)*scale, ...
    0, 'Color', color, 'LineWidth', 2);
quiver3(ax, origin(1), origin(2), origin(3), ...
    R(1,2)*scale, R(2,2)*scale, R(3,2)*scale, ...
    0, 'Color', color, 'LineWidth', 2);
quiver3(ax, origin(1), origin(2), origin(3), ...
    R(1,3)*scale, R(2,3)*scale, R(3,3)*scale, ...
    0, 'Color', color, 'LineWidth', 2);
end