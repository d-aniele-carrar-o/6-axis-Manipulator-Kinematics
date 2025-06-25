function [traj_handle] = plot_robot_trajectory(q_trajectory, keyframe_indices, keyframe_names, line_style, world_rf, step, robot_id, axs, display_name, use_triad)
% Plot robot end-effector trajectory and keyframes
%
% Parameters:
%   q_trajectory     - Joint configurations [Nx6]
%   keyframe_indices - Indices of keyframes in original data
%   keyframe_names   - Cell array of keyframe names (optional)
%   line_style       - Line style for trajectory (e.g., 'r-')
%   world_rf         - Boolean, true to use world reference frame
%   step             - Downsampling step used when creating q_trajectory
%   robot_id         - Robot ID (1 or 2)
%   axs              - Axes handle for plotting
%   display_name     - Name for legend
%   use_triad        - Boolean, true to use triad for keyframe visualization
    
    % Default parameters
    if nargin < 10, use_triad = false; end
    if nargin < 9, display_name = []; end
    if nargin < 8, axs = gca; end
    if nargin < 7, robot_id = 1; end
    if nargin < 6, step = 1; end
    if nargin < 5, world_rf = true; end
    if nargin < 4, line_style = 'r-'; end

    % Preallocate trajectory array
    ee_positions = zeros(size(q_trajectory, 1), 3);
    ee_rotations = zeros(size(q_trajectory, 1), 3, 3);
    
    % Compute end-effector positions for each configuration
    for i = 1:size(q_trajectory, 1)
        if world_rf
            [T_w_e, ~] = direct_kinematics(q_trajectory(i,:), robot_id);
            ee_positions(i, :) = T_w_e(1:3, 4)';
            ee_rotations(i, :, :) = T_w_e(1:3, 1:3);
        else
            [~, T_e] = direct_kinematics(q_trajectory(i,:), robot_id);
            ee_positions(i, :) = T_e(1:3, 4)';
            ee_rotations(i, :, :) = T_e(1:3, 1:3);
        end
    end
    
    % Plot trajectory
    if isempty(display_name)
        traj_handle = plot3(ee_positions(:,1), ee_positions(:,2), ee_positions(:,3), line_style, 'LineWidth', 2, 'Parent', axs);
    else
        traj_handle = plot3(ee_positions(:,1), ee_positions(:,2), ee_positions(:,3), line_style, 'LineWidth', 2, 'DisplayName', display_name, 'Parent', axs);
    end
    
    % Highlight keyframes if available
    if ~isempty(keyframe_indices)
        for i = 1:length(keyframe_indices)
            % Convert original index to downsampled index
            idx = min(floor(keyframe_indices(i) / step) + 1, size(ee_positions, 1));
            
            % Get position and orientation
            pos = ee_positions(idx,:)';
            rot = reshape(ee_rotations(idx,:,:), [3,3]);
            
            % Create transformation matrix
            T = [rot, pos; 0 0 0 1];
            
            % Visualize keyframe
            if use_triad
                % Use triad for coordinate frame visualization
                triad('Parent', axs, 'Scale', 0.05, 'LineWidth', 2, 'Matrix', T, ...
                      'Tag', sprintf('Keyframe %d Robot %d', i, robot_id));
            else
                % Simple marker
                scatter3(pos(1), pos(2), pos(3), 100, 'k*', 'LineWidth', 2, 'Parent', axs);
            end
            
            % Add text label with background
            color = line_style(1); % Use first character of line_style as color
            if ~isempty(keyframe_names)
                if iscell(keyframe_names) && i <= length(keyframe_names)
                    label_text = keyframe_names{i};
                elseif istable(keyframe_names) || isstring(keyframe_names)
                    if i <= length(keyframe_names)
                        label_text = char(keyframe_names(i));
                    else
                        label_text = sprintf('KF %d', i);
                    end
                else
                    label_text = sprintf('KF %d', i);
                end
            else
                label_text = sprintf('KF %d', i);
            end
            
            % Create text with white background and colored border
            text(pos(1), pos(2), pos(3) + 0.08, label_text, ...
                'Color', color, 'FontSize', 10, 'FontWeight', 'bold', 'Parent', axs, ...
                'HorizontalAlignment', 'center', 'BackgroundColor', 'white', ...
                'EdgeColor', color, 'Margin', 2);
        end
    end
end
