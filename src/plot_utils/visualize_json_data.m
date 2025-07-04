function visualize_json_data(json_file_path)
% VISUALIZE_JSON_DATA Visualizes pointclouds and robot poses from a JSON file
%
% This function reads a JSON file containing pointcloud data and robot poses,
% then visualizes them in an interactive 3D plot. The pointcloud is displayed
% as scattered points, and the robot poses are shown as coordinate frames with
% different colors for left and right arms.
%
% Input:
%   json_file_path - Path to the JSON file containing the data
%
% JSON structure expected:
% [
%   {
%     "obs": {
%       "pcl": [[x1, y1, z1], [x2, y2, z2], ..., [xn, yn, zn]]
%     },
%     "actions": {
%       "left": [
%         {"trans": [px1, py1, pz1], "rot": [rx1, ry1, rz1], "gripper": gx1},
%         {"trans": [px2, py2, pz2], "rot": [rx2, ry2, rz2], "gripper": gx2},
%         ...
%       ],
%       "right": [
%         {"trans": [px1, py1, pz1], "rot": [rx1, ry1, rz1], "gripper": gx1},
%         {"trans": [px2, py2, pz2], "rot": [rx2, ry2, rz2], "gripper": gx2},
%         ...
%       ]
%     }
%   }
% ]

if nargin < 1
    json_file_path = '/Users/danielecarraro/Documents/VSCODE/data/output/yoto/25-06-24-21-38-38_liftbox.json';
end
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

% Create figure
fig = figure('Name', 'Pointcloud and Robot Poses', 'Position', [100, 100, 1000, 800]);
ax = axes('Parent', fig);
hold(ax, 'on');
grid(ax, 'on');
axis equal;
view(ax, 3);
xlabel('X');
ylabel('Y');
zlabel('Z');
title('Pointcloud and Robot Poses');

% Process each entry in the JSON array
for i = 1:length(data)
    % Extract pointcloud data
    if isfield(data(i), 'obs') && isfield(data(i).obs, 'pcl')
        pcl = data(i).obs.pcl;
        
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
        scatter3(pcl_matrix(:,1), pcl_matrix(:,2), pcl_matrix(:,3), 5, 'b', 'filled', 'MarkerFaceAlpha', 0.5);
    end
    
    % Extract and plot robot poses
    if isfield(data(i), 'actions')
        % Process left arm poses
        if isfield(data(i).actions, 'left')
            left_poses = data(i).actions.left;
            left_color = 'r'; % Consistent color for left arm
            
            % Store all left arm positions for path plotting
            left_positions = zeros(length(left_poses), 3);
            
            for j = 1:length(left_poses)
                trans = left_poses(j).trans;
                rot = left_poses(j).rot;
                
                % Store position for path plotting
                left_positions(j,:) = trans;
                
                % Convert rotation vector to rotation matrix
                % Extract angle and axis from rotation vector
                angle = norm(rot);
                if angle > 0
                    axs = rot / angle;
                else
                    axs = [1, 0, 0]; % Default axis if angle is zero
                end
                R = angle_axis2rotm(angle, axs);
                
                % Plot coordinate frame for left arm (red)
                plot_frame(trans, R, 0.05, left_color);
                
                % Add keyframe number
                text(trans(1), trans(2), trans(3), sprintf('L%d', j), 'Color', left_color, 'FontSize', 10, 'HorizontalAlignment', 'center');
            end
            
            % Plot path connecting consecutive left arm poses
            plot3(left_positions(:,1), left_positions(:,2), left_positions(:,3), '-', 'Color', left_color, 'LineWidth', 2);
        end
        
        % Process right arm poses
        if isfield(data(i).actions, 'right')
            right_poses = data(i).actions.right;
            right_color = 'g'; % Consistent color for right arm
            
            % Store all right arm positions for path plotting
            right_positions = zeros(length(right_poses), 3);
            
            for j = 1:length(right_poses)
                trans = right_poses(j).trans;
                rot = right_poses(j).rot;
                
                % Store position for path plotting
                right_positions(j,:) = trans;
                
                % Convert rotation vector to rotation matrix
                % Extract angle and axis from rotation vector
                angle = norm(rot);
                if angle > 0
                    axs = rot / angle;
                else
                    axs = [1, 0, 0]; % Default axis if angle is zero
                end
                R = angle_axis2rotm(angle, axs);
                
                % Plot coordinate frame for right arm (green)
                plot_frame(trans, R, 0.05, right_color);
                
                % Add keyframe number
                text(trans(1), trans(2), trans(3), sprintf('R%d', j), 'Color', right_color, 'FontSize', 10, 'HorizontalAlignment', 'center');
            end
            
            % Plot path connecting consecutive right arm poses
            plot3(right_positions(:,1), right_positions(:,2), right_positions(:,3), '-', 'Color', right_color, 'LineWidth', 2);
        end
    end
end

% Add legend with conditional entries based on what's actually plotted
legend_entries = {};
legend_handles = [];

% Check what elements were actually plotted
children = get(gca, 'Children');
types = get(children, 'Type');

% Add pointcloud to legend if it exists
if any(strcmp(types, 'scatter'))
    pcl_handle = findobj(gca, 'Type', 'scatter');
    legend_handles = [legend_handles, pcl_handle(1)];
    legend_entries{end+1} = 'Pointcloud';
end

% Add left arm elements to legend if they exist
if isfield(data(i).actions, 'left') && ~isempty(data(i).actions.left)
    % Find left arm path line
    left_line = findobj(gca, 'Type', 'line', 'Color', 'r');
    if ~isempty(left_line)
        legend_handles = [legend_handles, left_line(1)];
        legend_entries{end+1} = 'Left Arm Path';
    end
    
    % Find left arm frame (use one of the quiver objects)
    left_frame = findobj(gca, 'Type', 'quiver', 'Color', 'r');
    if ~isempty(left_frame)
        legend_handles = [legend_handles, left_frame(1)];
        legend_entries{end+1} = 'Left Arm Poses';
    end
end

% Add right arm elements to legend if they exist
if isfield(data(i).actions, 'right') && ~isempty(data(i).actions.right)
    % Find right arm path line
    right_line = findobj(gca, 'Type', 'line', 'Color', 'g');
    if ~isempty(right_line)
        legend_handles = [legend_handles, right_line(1)];
        legend_entries{end+1} = 'Right Arm Path';
    end
    
    % Find right arm frame (use one of the quiver objects)
    right_frame = findobj(gca, 'Type', 'quiver', 'Color', 'g');
    if ~isempty(right_frame)
        legend_handles = [legend_handles, right_frame(1)];
        legend_entries{end+1} = 'Right Arm Poses';
    end
end

% Create legend if we have any entries
if ~isempty(legend_handles)
    legend(legend_handles, legend_entries, 'Location', 'best');
end

% Make plot interactive
rotate3d on;

end

function plot_frame(origin, R, scale, color)
% Plot a coordinate frame at the specified origin with the given rotation matrix
% origin: [x, y, z] position
% R: 3x3 rotation matrix
% scale: size of the coordinate frame axes
% color: color of the frame (e.g., 'r' for red, 'g' for green)

% X-axis (red)
quiver3(origin(1), origin(2), origin(3), ...
    R(1,1)*scale, R(2,1)*scale, R(3,1)*scale, ...
    0, 'Color', color, 'LineWidth', 2);

% Y-axis (green)
quiver3(origin(1), origin(2), origin(3), ...
    R(1,2)*scale, R(2,2)*scale, R(3,2)*scale, ...
    0, 'Color', color, 'LineWidth', 2);

% Z-axis (blue)
quiver3(origin(1), origin(2), origin(3), ...
    R(1,3)*scale, R(2,3)*scale, R(3,3)*scale, ...
    0, 'Color', color, 'LineWidth', 2);
end
