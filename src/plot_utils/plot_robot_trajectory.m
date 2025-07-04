function [traj_handle] = plot_robot_trajectory(q_trajectory, keyframes_data, robot_id, axs, line_style, world_rf, use_triad, use_transformed)
% Plot robot end-effector trajectory and keyframes
%
% Parameters:
%   q_trajectory     - Joint configurations [Nx6]
%   keyframes_data   - Structure containing keyframe information with fields:
%     .available     - Boolean indicating if keyframes are available
%     .indices       - Indices of keyframes in original data
%     .names         - Names of keyframes
%     .q_left        - Joint configurations for left robot at keyframes [Kx6]
%     .q_right       - Joint configurations for right robot at keyframes [Kx6]
%   robot_id         - Robot ID (1 or 2)
%   axs              - Axes handle for plotting
%   line_style       - Line style for trajectory (e.g., 'r-')
%   world_rf         - Boolean, true to use world reference frame
%   use_triad        - Boolean, true to use triad for keyframe visualization
    
    % Default parameters
    if nargin < 8, use_transformed = false; end
    if nargin < 7, use_triad = false; end
    if nargin < 6, world_rf = true; end
    if nargin < 5, line_style = 'r-'; end
    if nargin < 4, axs = gca; end
    if nargin < 3, robot_id = 1; end
    if nargin < 2
        keyframes_data.available = false;
    end

    % Compute end-effector positions for each configuration
    [ee_pos, ee_rot] = get_end_effector_trajectory(q_trajectory, robot_id, world_rf);
    
    % Plot trajectory
    traj_handle = plot3(ee_pos(:,1), ee_pos(:,2), ee_pos(:,3), line_style, 'LineWidth', 2, 'Parent', axs);
    
    % Highlight keyframes if available
    if keyframes_data.available
        local_idx = 1;
        for i = 1:length(keyframes_data.indices)
            % Extract robot number from keyframe name
            kf_name = keyframes_data.names{i};
            
            % Process boundary keyframe
            if strcmp(kf_name, 'boundary')
                plot_keyframe(i, local_idx, keyframes_data, ee_pos, ee_rot, axs, line_style(1), robot_id, false, use_triad);
                local_idx = local_idx + 1;
                continue;
            end
            
            robot_match = regexp(kf_name, 'grasp_r(\d+)', 'tokens');
            if ~isempty(robot_match)
                % Process grasp keyframes
                kf_robot_num = str2double(robot_match{1}{1});
            
                % Map robot number to robot_id (4=left=1, 2=right=2)
                kf_robot_id = (kf_robot_num == 4) * 1 + (kf_robot_num == 2) * 2;

                % Only plot keyframe if it belongs to current robot
                if kf_robot_id ~= robot_id
                    continue;
                end
            end
            
            % Always process other keyframes (kinematic, dynamic, geometric)
            plot_keyframe(i, local_idx, keyframes_data, ee_pos, ee_rot, axs, line_style(1), robot_id, use_transformed, use_triad);
            local_idx = local_idx + 1;
        
        end
    end
end

function plot_keyframe(i, local_idx, keyframes_data, ee_pos, ee_rot, axs, color, robot_id, use_transformed, use_triad)
    % Extract robot number from keyframe name
    kf_name = keyframes_data.names{i};

    % Get keyframe index wrt q_trajectory
    kf_idx = keyframes_data.indices(i);

    if ~use_transformed
        % Get position and orientation
        pos = ee_pos(kf_idx,:)';
        rot = reshape(ee_rot(kf_idx,:,:), [3,3]);
        
        % Create transformation matrix
        T = [rot, pos; 0 0 0 1];
    else
        if robot_id == 1
            T = squeeze(keyframes_data.transformed_keyframes_l(local_idx,:,:));
        else
            T = squeeze(keyframes_data.transformed_keyframes_r(local_idx,:,:));
        end
        pos = T(1:3,4);
    end
    
    % Visualize keyframe
    if use_triad
        % Use triad for coordinate frame visualization
        triad('Parent', axs, 'Scale', 0.05, 'LineWidth', 2, 'Matrix', T, ...
                'Tag', sprintf('Keyframe %d Robot %d', i, robot_id));
    else
        % Simple marker
        scatter3(pos(1), pos(2), pos(3), 100, 'k*', 'LineWidth', 2, 'Parent', axs);
    end
    
    % Create text with white background and colored border
    % text(pos(1), pos(2), pos(3) + 0.08, kf_name, ...
    %     'Color', color, 'FontSize', 10, 'FontWeight', 'bold', 'Parent', axs, ...
    %     'HorizontalAlignment', 'center', 'BackgroundColor', 'white', ...
    %     'EdgeColor', color, 'Margin', 2);
end
