function visualize_multiple_demos(json_file_path, varargin)
% VISUALIZE_MULTIPLE_DEMOS Interactive visualization of multiple demonstrations
%
% This function reads a JSON file containing multiple demonstrations and
% displays them one at a time with navigation buttons.
%
% Input:
%   json_file_path - Path to the JSON file containing multiple demonstrations
%   'single_object_mode' - (optional) boolean, if true, loads only one object
%                         and 2 closest keyframes for trajectory analysis
%
% Example:
%   visualize_multiple_demos('demo.json')  % Normal mode
%   visualize_multiple_demos('demo.json', 'single_object_mode', true)  % Single object mode

if nargin < 2, single_object_mode = true; end
if nargin < 1, json_file_path = '/Users/danielecarraro/Documents/VSCODE/data/output/yoto/liftbox_preprocessed_aug100x.json'; end

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
fig.UserData = struct('data', data, 'current_demo', 1, 'ax', ax, 'txt_info', txt_info, 'single_object_mode', single_object_mode);

% Set button callbacks
set(btn_prev, 'Callback', @(src, evt) navigate_demo(fig, -1));
set(btn_next, 'Callback', @(src, evt) navigate_demo(fig, 1));

% Display first demonstration
plot_single_demo(fig, 1);

% Make plot interactive
rotate3d(ax, 'on');

if single_object_mode
    fprintf('Loaded %d demonstrations in SINGLE OBJECT MODE. Showing only one object with 2 closest keyframes.\n', length(data));
else
    fprintf('Loaded %d demonstrations. Use navigation buttons to browse.\n', length(data));
end
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
single_object_mode = userData.single_object_mode;

% Clear previous plot
cla(ax);
hold(ax, 'on');
grid(ax, 'on');
axis(ax, 'equal');
view(ax, [90, 90]);  % Top view
xlabel(ax, 'X');
ylabel(ax, 'Y');
zlabel(ax, 'Z');

% Update info text
if single_object_mode
    set(txt_info, 'String', sprintf('Demo %d of %d (Single Object Mode)', demo_idx, length(data)));
    title(ax, sprintf('Demonstration %d - Single Object Analysis', demo_idx));
else
    set(txt_info, 'String', sprintf('Demo %d of %d', demo_idx, length(data)));
    title(ax, sprintf('Demonstration %d', demo_idx));
end

% Extract current demonstration
demo = data(demo_idx);

% Plot pointcloud and get centroids
pcl_plotted = false;
current_centroid = [];
original_centroid = [];

if isfield(demo, 'obs')
    % Try obj_pcl first
    if isfield(demo.obs, 'obj_pcl') && ~isempty(demo.obs.obj_pcl)
        obj_pcl = demo.obs.obj_pcl;
        fprintf('Found obj_pcl with %d points\n', length(obj_pcl));
        pcl_matrix = convert_pcl_to_matrix(obj_pcl);
        [current_centroid, original_centroid] = plot_object_pointcloud_with_centroids(ax, pcl_matrix, demo, data, demo_idx);
        pcl_plotted = true;
    % Try pcl as fallback
    elseif isfield(demo.obs, 'pcl') && ~isempty(demo.obs.pcl)
        pcl = demo.obs.pcl;
        fprintf('Found pcl with %d points\n', length(pcl));
        pcl_matrix = convert_pcl_to_matrix(pcl);
        [current_centroid, original_centroid] = plot_object_pointcloud_with_centroids(ax, pcl_matrix, demo, data, demo_idx);
        pcl_plotted = true;
    end
end

if ~pcl_plotted
    fprintf('No pointcloud data found in demo %d\n', demo_idx);
end

% Plot robot poses
if isfield(demo, 'actions')
    legend_handles = [];
    legend_entries = {};
    
    % Process left arm poses
    if isfield(demo.actions, 'left') && ~isempty(demo.actions.left)
        left_poses = demo.actions.left;
        
        if single_object_mode
            % Find object center for distance calculation
            object_center = get_object_center(demo);
            
            % Find 2 closest keyframes to object
            closest_indices = find_closest_keyframes(left_poses, object_center, 2);
            selected_poses = left_poses(closest_indices);
            
            fprintf('Left arm: Selected keyframes %s (closest to object)\n', mat2str(closest_indices));
            
            % Plot original keyframes in grey if not first demo
            if demo_idx > 1 && isfield(data(1).actions, 'left')
                plot_original_keyframes(ax, data(1).actions.left, closest_indices, 'left', original_centroid);
            end
        else
            selected_poses = left_poses;
            closest_indices = 1:length(left_poses);
        end
        
        left_positions = zeros(length(selected_poses), 3);
        
        for j = 1:length(selected_poses)
            trans = selected_poses(j).trans;
            rot = selected_poses(j).rot;
            left_positions(j,:) = trans;
            
            % Convert rotation vector to rotation matrix
            angle = norm(rot);
            if angle > 0
                axs = rot / angle;
            else
                axs = [1, 0, 0];
            end
            R = angle_axis2rotm(angle, axs);
            
            % Plot triad for left arm (red)
            T_matrix = [R, trans; 0, 0, 0, 1];
            triad('Parent', ax, 'Matrix', T_matrix, 'Scale', 0.05, 'LineWidth', 3);
            
            % Add keyframe label with extraction method
            if single_object_mode
                label_text = sprintf('L%d(#%d)', j, closest_indices(j));
                if isfield(selected_poses(j), 'extraction_method')
                    label_text = sprintf('%s-%s', label_text, selected_poses(j).extraction_method);
                end
                text(trans(1), trans(2), trans(3)+0.02, label_text, ...
                     'Color', 'r', 'FontSize', 9, 'FontWeight', 'bold', 'Parent', ax);
            else
                label_text = sprintf('L%d', j);
                if isfield(selected_poses(j), 'extraction_method')
                    label_text = sprintf('%s-%s', label_text, selected_poses(j).extraction_method);
                end
                text(trans(1), trans(2), trans(3)+0.02, label_text, ...
                     'Color', 'r', 'FontSize', 9, 'FontWeight', 'bold', 'Parent', ax);
            end
            
            % Draw line from keyframe to current object centroid
            if ~isempty(current_centroid)
                plot3(ax, [trans(1), current_centroid(1)], [trans(2), current_centroid(2)], [trans(3), current_centroid(3)], ...
                      '-', 'Color', [0.5, 0.8, 1], 'LineWidth', 1.5);
            end
        end
        
        % Plot path for left arm (dashed)
        h_left = plot3(ax, left_positions(:,1), left_positions(:,2), left_positions(:,3), ...
                      '--r', 'LineWidth', 2, 'MarkerSize', 8);
        legend_handles = [legend_handles, h_left];
        legend_entries{end+1} = 'Left Arm';
    end
    
    % Process right arm poses
    if isfield(demo.actions, 'right') && ~isempty(demo.actions.right)
        right_poses = demo.actions.right;
        
        if single_object_mode
            % Find object center for distance calculation
            object_center = get_object_center(demo);
            
            % Find 2 closest keyframes to object
            closest_indices = find_closest_keyframes(right_poses, object_center, 2);
            selected_poses = right_poses(closest_indices);
            
            fprintf('Right arm: Selected keyframes %s (closest to object)\n', mat2str(closest_indices));
            
            % Plot original keyframes in grey if not first demo
            if demo_idx > 1 && isfield(data(1).actions, 'right')
                plot_original_keyframes(ax, data(1).actions.right, closest_indices, 'right', original_centroid);
            end
        else
            selected_poses = right_poses;
            closest_indices = 1:length(right_poses);
        end
        
        right_positions = zeros(length(selected_poses), 3);
        
        for j = 1:length(selected_poses)
            trans = selected_poses(j).trans;
            rot = selected_poses(j).rot;
            right_positions(j,:) = trans;
            
            % Convert rotation vector to rotation matrix
            angle = norm(rot);
            if angle > 0
                axs = rot / angle;
            else
                axs = [1, 0, 0];
            end
            R = angle_axis2rotm(angle, axs);
            
            % Plot triad for right arm (green)
            T_matrix = [R, trans; 0, 0, 0, 1];
            triad('Parent', ax, 'Matrix', T_matrix, 'Scale', 0.05, 'LineWidth', 3);
            
            % Add keyframe label with extraction method
            if single_object_mode
                label_text = sprintf('R%d(#%d)', j, closest_indices(j));
                if isfield(selected_poses(j), 'extraction_method')
                    label_text = sprintf('%s-%s', label_text, selected_poses(j).extraction_method);
                end
                text(trans(1), trans(2), trans(3)+0.02, label_text, ...
                     'Color', 'g', 'FontSize', 9, 'FontWeight', 'bold', 'Parent', ax);
            else
                label_text = sprintf('R%d', j);
                if isfield(selected_poses(j), 'extraction_method')
                    label_text = sprintf('%s-%s', label_text, selected_poses(j).extraction_method);
                end
                text(trans(1), trans(2), trans(3)+0.02, label_text, ...
                     'Color', 'g', 'FontSize', 9, 'FontWeight', 'bold', 'Parent', ax);
            end
            
            % Draw line from keyframe to current object centroid
            if ~isempty(current_centroid)
                plot3(ax, [trans(1), current_centroid(1)], [trans(2), current_centroid(2)], [trans(3), current_centroid(3)], ...
                      '-', 'Color', [0.5, 0.8, 1], 'LineWidth', 1.5);
            end
        end
        
        % Plot path for right arm (dashed)
        h_right = plot3(ax, right_positions(:,1), right_positions(:,2), right_positions(:,3), ...
                       '--g', 'LineWidth', 2, 'MarkerSize', 8);
        legend_handles = [legend_handles, h_right];
        legend_entries{end+1} = 'Right Arm';
    end
    
    % Add object pointcloud to legend
    if isfield(demo, 'obs') && isfield(demo.obs, 'obj_pcl')
        h_obj = scatter3(ax, NaN, NaN, NaN, 10, 'b', 'filled', 'MarkerFaceAlpha', 0.6);
        legend_handles = [h_obj, legend_handles];
        legend_entries = [{'Object'}, legend_entries];
    end
    
    % Add original elements to legend if shown
    if demo_idx > 1 && isfield(data(1), 'obs') && isfield(data(1).obs, 'obj_pcl')
        h_orig_obj = scatter3(ax, NaN, NaN, NaN, 8, [0.5 0.5 0.5], 'filled', 'MarkerFaceAlpha', 0.4);
        legend_handles = [legend_handles, h_orig_obj];
        legend_entries{end+1} = 'Original Object';
    end
    
    if single_object_mode && demo_idx > 1
        h_orig_kf = plot3(ax, NaN, NaN, NaN, 'Color', [0.5 0.5 0.5], 'LineWidth', 2);
        legend_handles = [legend_handles, h_orig_kf];
        legend_entries{end+1} = 'Original Keyframes';
    end
    
    % Create legend if we have entries
    if ~isempty(legend_handles)
        legend(ax, legend_handles, legend_entries, 'Location', 'best');
    end
end
end



function object_center = get_object_center(demo)
% Get object center from closest object pointcloud
if isfield(demo, 'obs') && isfield(demo.obs, 'obj_pcl')
    obj_pcl = demo.obs.obj_pcl;
    if iscell(obj_pcl)
        obj_pcl_matrix = zeros(length(obj_pcl), 3);
        for j = 1:length(obj_pcl)
            obj_pcl_matrix(j,:) = obj_pcl{j};
        end
    else
        obj_pcl_matrix = obj_pcl;
    end
    
    % Try to find closest object, fallback to centroid of all points
    try
        scene_pc = pointCloud(obj_pcl_matrix);
        object_clusters = cluster_scene_objects(scene_pc);
        if ~isempty(object_clusters)
            closest_obj_pc = find_closest_object_to_keyframes(object_clusters, demo);
            if ~isempty(closest_obj_pc)
                object_center = mean(closest_obj_pc.Location, 1);
                return;
            end
        end
    catch
    end
    
    % Fallback to centroid of all points
    object_center = mean(obj_pcl_matrix, 1);
else
    object_center = [0, 0, 0];
end
end

function pcl_matrix = convert_pcl_to_matrix(pcl_data)
% Convert pointcloud data to matrix format
if iscell(pcl_data)
    pcl_matrix = zeros(length(pcl_data), 3);
    for j = 1:length(pcl_data)
        if iscell(pcl_data{j})
            pcl_matrix(j,:) = [pcl_data{j}{:}];
        else
            pcl_matrix(j,:) = pcl_data{j};
        end
    end
else
    pcl_matrix = pcl_data;
end
end

function [current_centroid, original_centroid] = plot_object_pointcloud_with_centroids(ax, pcl_matrix, demo, data, demo_idx)
% Plot object pointcloud with clustering and return centroids
current_centroid = [];
original_centroid = [];

% Plot current object
if size(pcl_matrix, 1) < 10
    % Too few points, plot directly
    scatter3(ax, pcl_matrix(:,1), pcl_matrix(:,2), pcl_matrix(:,3), ...
             10, 'b', 'filled', 'MarkerFaceAlpha', 0.6);
    current_centroid = mean(pcl_matrix, 1);
else
    % Try clustering
    try
        scene_pc = pointCloud(pcl_matrix);
        object_clusters = cluster_scene_objects(scene_pc);
        
        if ~isempty(object_clusters)
            closest_obj_pc = find_closest_object_to_keyframes(object_clusters, demo);
            if ~isempty(closest_obj_pc)
                scatter3(ax, closest_obj_pc.Location(:,1), closest_obj_pc.Location(:,2), closest_obj_pc.Location(:,3), ...
                         10, 'b', 'filled', 'MarkerFaceAlpha', 0.6);
                current_centroid = mean(closest_obj_pc.Location, 1);
            else
                % Use first cluster if no closest found
                scatter3(ax, object_clusters{1}.Location(:,1), object_clusters{1}.Location(:,2), object_clusters{1}.Location(:,3), ...
                         10, 'b', 'filled', 'MarkerFaceAlpha', 0.6);
                current_centroid = mean(object_clusters{1}.Location, 1);
            end
        else
            % No clusters found, plot all
            scatter3(ax, pcl_matrix(:,1), pcl_matrix(:,2), pcl_matrix(:,3), ...
                     10, 'b', 'filled', 'MarkerFaceAlpha', 0.6);
            current_centroid = mean(pcl_matrix, 1);
        end
    catch
        % Clustering failed, plot all points
        scatter3(ax, pcl_matrix(:,1), pcl_matrix(:,2), pcl_matrix(:,3), ...
                 10, 'b', 'filled', 'MarkerFaceAlpha', 0.6);
        current_centroid = mean(pcl_matrix, 1);
    end
end

% Plot current object centroid
if ~isempty(current_centroid)
    scatter3(ax, current_centroid(1), current_centroid(2), current_centroid(3), ...
             50, 'b', 'filled', 'MarkerEdgeColor', 'k', 'LineWidth', 2);
end

% Plot original closest object if not first demo
if demo_idx > 1 && isfield(data(1), 'obs')
    orig_pcl_matrix = [];
    if isfield(data(1).obs, 'obj_pcl') && ~isempty(data(1).obs.obj_pcl)
        orig_pcl_matrix = convert_pcl_to_matrix(data(1).obs.obj_pcl);
    elseif isfield(data(1).obs, 'pcl') && ~isempty(data(1).obs.pcl)
        orig_pcl_matrix = convert_pcl_to_matrix(data(1).obs.pcl);
    end
    
    if ~isempty(orig_pcl_matrix)
        % Apply same clustering logic to original object
        if size(orig_pcl_matrix, 1) < 10
            % Too few points, plot directly
            scatter3(ax, orig_pcl_matrix(:,1), orig_pcl_matrix(:,2), orig_pcl_matrix(:,3), ...
                     8, [0.5 0.5 0.5], 'filled', 'MarkerFaceAlpha', 0.4);
            original_centroid = mean(orig_pcl_matrix, 1);
        else
            % Try clustering for original object
            try
                orig_scene_pc = pointCloud(orig_pcl_matrix);
                orig_object_clusters = cluster_scene_objects(orig_scene_pc);
                
                if ~isempty(orig_object_clusters)
                    orig_closest_obj_pc = find_closest_object_to_keyframes(orig_object_clusters, data(1));
                    if ~isempty(orig_closest_obj_pc)
                        scatter3(ax, orig_closest_obj_pc.Location(:,1), orig_closest_obj_pc.Location(:,2), orig_closest_obj_pc.Location(:,3), ...
                                 8, [0.5 0.5 0.5], 'filled', 'MarkerFaceAlpha', 0.4);
                        original_centroid = mean(orig_closest_obj_pc.Location, 1);
                    else
                        % Use first cluster if no closest found
                        scatter3(ax, orig_object_clusters{1}.Location(:,1), orig_object_clusters{1}.Location(:,2), orig_object_clusters{1}.Location(:,3), ...
                                 8, [0.5 0.5 0.5], 'filled', 'MarkerFaceAlpha', 0.4);
                        original_centroid = mean(orig_object_clusters{1}.Location, 1);
                    end
                else
                    % No clusters found, plot all
                    scatter3(ax, orig_pcl_matrix(:,1), orig_pcl_matrix(:,2), orig_pcl_matrix(:,3), ...
                             8, [0.5 0.5 0.5], 'filled', 'MarkerFaceAlpha', 0.4);
                    original_centroid = mean(orig_pcl_matrix, 1);
                end
            catch
                % Clustering failed, plot all points
                scatter3(ax, orig_pcl_matrix(:,1), orig_pcl_matrix(:,2), orig_pcl_matrix(:,3), ...
                         8, [0.5 0.5 0.5], 'filled', 'MarkerFaceAlpha', 0.4);
                original_centroid = mean(orig_pcl_matrix, 1);
            end
        end
        
        % Plot original object centroid
        if ~isempty(original_centroid)
            scatter3(ax, original_centroid(1), original_centroid(2), original_centroid(3), ...
                     40, [0.5 0.5 0.5], 'filled', 'MarkerEdgeColor', 'k', 'LineWidth', 2);
        end
    end
end
end

function closest_obj_pc = find_closest_object_to_keyframes(object_clusters, demo)
% Find the object cluster closest to the keyframes
closest_obj_pc = [];

if ~isfield(demo, 'actions') || isempty(object_clusters)
    return;
end

% Collect all keyframe positions
keyframe_positions = [];
if isfield(demo.actions, 'left') && ~isempty(demo.actions.left)
    for i = 1:length(demo.actions.left)
        keyframe_positions = [keyframe_positions; demo.actions.left(i).trans'];
    end
end
if isfield(demo.actions, 'right') && ~isempty(demo.actions.right)
    for i = 1:length(demo.actions.right)
        keyframe_positions = [keyframe_positions; demo.actions.right(i).trans'];
    end
end

if isempty(keyframe_positions)
    return;
end

% Find closest object cluster
min_distance = inf;
for i = 1:length(object_clusters)
    obj_center = mean(object_clusters{i}.Location, 1);
    
    % Calculate minimum distance to any keyframe
    distances = sqrt(sum((keyframe_positions - obj_center).^2, 2));
    min_dist_to_obj = min(distances);
    
    if min_dist_to_obj < min_distance
        min_distance = min_dist_to_obj;
        closest_obj_pc = object_clusters{i};
    end
end
end

function plot_original_keyframes(ax, original_poses, indices, arm_name, original_centroid)
% Plot original keyframes in grey using triads
for j = 1:length(indices)
    if indices(j) <= length(original_poses)
        trans = original_poses(indices(j)).trans;
        rot = original_poses(indices(j)).rot;
        
        % Convert rotation vector to rotation matrix
        angle = norm(rot);
        if angle > 0
            axs = rot / angle;
        else
            axs = [1, 0, 0];
        end
        R = angle_axis2rotm(angle, axs);
        
        % Plot triad in grey
        T_matrix = [R, trans; 0, 0, 0, 1];
        h_triad = triad('Parent', ax, 'Matrix', T_matrix, 'Scale', 0.03, 'LineWidth', 2);
        
        % Change triad colors to grey
        children = get(h_triad, 'Children');
        for k = 1:length(children)
            set(children(k), 'Color', [0.5 0.5 0.5]);
        end
        
        % Add label with extraction method
        label_text = sprintf('Orig_%s%d', upper(arm_name(1)), j);
        if isfield(original_poses(indices(j)), 'extraction_method')
            label_text = sprintf('%s-%s', label_text, original_poses(indices(j)).extraction_method);
        end
        text(trans(1), trans(2), trans(3)-0.02, label_text, ...
             'Color', [0.5 0.5 0.5], 'FontSize', 8, 'Parent', ax);
        
        % Draw line from original keyframe to original centroid
        if ~isempty(original_centroid)
            plot3(ax, [trans(1), original_centroid(1)], [trans(2), original_centroid(2)], [trans(3), original_centroid(3)], ...
                  '-', 'Color', [0.5 0.5 0.5], 'LineWidth', 1);
        end
    end
end
end

function closest_indices = find_closest_keyframes(poses, object_center, num_keyframes)
% Find the N closest keyframes to the object center
distances = zeros(length(poses), 1);

for i = 1:length(poses)
    pose_position = poses(i).trans;
    distances(i) = norm(pose_position - object_center);
end

[~, sorted_indices] = sort(distances);
closest_indices = sorted_indices(1:min(num_keyframes, length(poses)));
closest_indices = sort(closest_indices);  % Keep original order
end