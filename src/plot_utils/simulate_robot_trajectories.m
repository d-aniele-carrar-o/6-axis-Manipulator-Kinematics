function simulate_robot_trajectories(robot_left, robot_right, config_left, config_right, ...
                                     q_l_all, q_r_all, keyframe_indices, keyframe_names, step, axs, ...
                                     traj_left_handle, traj_right_handle)
    % Simulate robot trajectories with keyframe stops and greying executed paths
    
    % Get keyframe indices for downsampled data
    kf_indices_downsampled = floor(keyframe_indices / step) + 1;
    kf_indices_downsampled = min(kf_indices_downsampled, size(q_l_all, 1));
    
    % Animation parameters
    pause_time = 0.05; % Seconds between frames
    
    % Get trajectory data
    left_x  = get(traj_left_handle, 'XData');
    left_y  = get(traj_left_handle, 'YData');
    left_z  = get(traj_left_handle, 'ZData');
    right_x = get(traj_right_handle, 'XData');
    right_y = get(traj_right_handle, 'YData');
    right_z = get(traj_right_handle, 'ZData');
    
    % Delete original trajectories
    delete(traj_left_handle);
    delete(traj_right_handle);
    
    % Store trajectory plot handles for clearing
    traj_plots = [];
    
    fprintf('Animating %d trajectory points...\n', size(q_l_all, 1));
    
    for i = 1:size(q_l_all, 1)
        % Update robot configurations
        config_left  = set_robot_configuration(q_l_all(i,:), config_left);
        config_right = set_robot_configuration(q_r_all(i,:), config_right);
        
        % Show updated robot positions
        show(robot_left, config_left, "Visuals", "on", "Frames", "off", ...
             "FastUpdate", true, "PreservePlot", false, "Parent", axs);
        show(robot_right, config_right, "Visuals", "on", "Frames", "off", ...
             "FastUpdate", true, "PreservePlot", false, "Parent", axs);
        
        % Add end-effector triads
        [T_w_e_l, ~] = direct_kinematics(q_l_all(i,:), 1);
        [T_w_e_r, ~] = direct_kinematics(q_r_all(i,:), 2);
        
        % Delete previous triads if they exist
        delete(findobj(axs, 'Tag', 'EE_Left'));
        delete(findobj(axs, 'Tag', 'EE_Right'));
        
        % Create new triads
        triad('Parent', axs, 'Scale', 0.08, 'LineWidth', 3, 'Matrix', T_w_e_l, 'Tag', 'EE_Left');
        triad('Parent', axs, 'Scale', 0.08, 'LineWidth', 3, 'Matrix', T_w_e_r, 'Tag', 'EE_Right');
        
        % Clear previous trajectory plots
        if ~isempty(traj_plots)
            delete(traj_plots(isvalid(traj_plots)));
        end
        traj_plots = [];
        
        % Draw trajectory segments
        hold(axs, 'on');
        if i > 1
            % Draw executed (grey) portions
            h1 = plot3(axs, left_x(1:i),  left_y(1:i),  left_z(1:i),  'Color', [0.5 0.5 0.5], 'LineWidth', 1);
            h2 = plot3(axs, right_x(1:i), right_y(1:i), right_z(1:i), 'Color', [0.5 0.5 0.5], 'LineWidth', 1);
            traj_plots = [traj_plots, h1, h2];
        end
        
        % Draw remaining (colored) portions
        if i < length(left_x)
            h3 = plot3(axs, left_x(i:end),  left_y(i:end),  left_z(i:end),  'r-', 'LineWidth', 2);
            h4 = plot3(axs, right_x(i:end), right_y(i:end), right_z(i:end), 'b-', 'LineWidth', 2);
            traj_plots = [traj_plots, h3, h4];
        end
        
        % Check if current index is a keyframe
        if any(kf_indices_downsampled == i)
            kf_num = find(kf_indices_downsampled == i);
            if ~isempty(keyframe_names) && all(iscell(keyframe_names)) && all(kf_num <= length(keyframe_names))
                kf_name = keyframe_names{kf_num};
                fprintf('Reached keyframe %d (%s) - pausing, press a key to continue...\n', kf_num, kf_name);
            else
                fprintf('Reached keyframe %d - pausing, press a key to continue...\n', kf_num);
            end
            pause();
        else
            pause(pause_time);
        end
        
        drawnow;
    end
    
    fprintf('Trajectory simulation completed\n');
end
