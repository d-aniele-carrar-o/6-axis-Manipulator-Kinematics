function letter_writing(text, scale, start_pos)
    % LETTER_WRITING - Generate trajectories for writing text with manipulator
    parameters(1);

    if nargin < 1, text = 'H'; end
    if nargin < 2, scale = 0.05; end
    if nargin < 3
        if manipulator == "UR3e", start_pos = [-0.1, 0.2, 0.2]; end
        if manipulator == "3Dprinted", start_pos = [0.2, 0, 0.15]; end
    end
    fprintf('Starting letter writing for: %s\n', text);
    fprintf('Scale: %.3f, Start position: [%.3f, %.3f, %.3f]\n', scale, start_pos);
    
    % Letter path definitions (normalized coordinates)
    letters = containers.Map();
    letters('A') = [0,0; 0.5,1; 1,0; NaN,NaN; 0.25,0.5; 0.75,0.5];
    letters('B') = [0,0; 0,1; 0.6,1; 0.6,0.5; 0,0.5; 0.6,0.5; 0.6,0; 0,0];
    letters('C') = [1,0.2; 0.8,0; 0.2,0; 0,0.2; 0,0.8; 0.2,1; 0.8,1; 1,0.8];
    letters('E') = [1,0; 0,0; 0,1; 1,1; NaN,NaN; 0,0.5; 0.6,0.5];
    letters('H') = [0,0; 0,1; NaN,NaN; 1,0; 1,1; NaN,NaN; 0,0.5; 1,0.5];
    letters('L') = [0,1; 0,0; 1,0];
    letters('O') = [0.2,0; 0,0.2; 0,0.8; 0.2,1; 0.8,1; 1,0.8; 1,0.2; 0.8,0; 0.2,0];
    
    % Generate trajectory points
    all_points = [];
    pen_states = [];
    current_x = start_pos(1);
    
    text = char(text); % Convert string to char array
    
    for i = 1:length(text)
        letter = upper(text(i));
        fprintf('Processing letter: %c\n', letter);
        
        if letter == ' '
            current_x = current_x + 1.5 * scale;
            continue;
        end
        
        if isKey(letters, letter)
            coords = letters(letter);
            fprintf('  Found %d points for letter %c\n', size(coords,1), letter);
            
            % Scale and translate coordinates
            scaled_coords = coords * scale;
            scaled_coords(:,1) = scaled_coords(:,1) + current_x;
            scaled_coords(:,2) = scaled_coords(:,2) + start_pos(2);
            
            % Add z-coordinate
            letter_points = [scaled_coords, repmat(start_pos(3), size(scaled_coords,1), 1)];
            
            % Determine pen up/down states
            pen_down = ~isnan(letter_points(:,1));
            
            all_points = [all_points; letter_points];
            pen_states = [pen_states; pen_down];
            
            current_x = current_x + 1.2 * scale;
        else
            fprintf('  Letter %c not found in dictionary\n', letter);
        end
    end
    
    fprintf('Total points generated: %d\n', size(all_points,1));
    
    % Visualize letter paths
    if ~isempty(all_points)
        figure;
        hold on;
        for i = 1:length(pen_states)
            if pen_states(i)
                plot3(all_points(i,1), all_points(i,2), all_points(i,3), 'b.-', 'MarkerSize', 8);
            else
                plot3(all_points(i,1), all_points(i,2), all_points(i,3), 'r.', 'MarkerSize', 8);
            end
        end
        xlabel('X'); ylabel('Y'); zlabel('Z');
        title('Letter Writing Path (Blue=pen down, Red=pen up)');
        grid on; axis equal;
        
        write_trajectory(all_points, pen_states);
    else
        fprintf('No points generated!\n');
    end
end

function write_trajectory(points, pen_states)
    fprintf('\n[write_trajectory] Starting with %d points\n', size(points,1));
    parameters(0);
    
    if isempty(points)
        fprintf('[write_trajectory] points is empty\n');
        return;
    end
    
    % Use task space trajectory
    if manipulator == "UR3e", q0 = [-pi/2, -pi/3, pi/3, -pi/2, -pi/2, 0]; end
    if manipulator == "3Dprinted", q0 = [0, pi/2, 0, 0, 0, 0]; end
    [T_start, ~] = direct_kinematics(q0);
    % R_fixed = eul2rotm_custom([pi, 0, 0]); % Keep same orientation
    R_fixed = T_start(1:3,1:3); % Keep same orientation
    
    % Create task space waypoints with pen up/down control
    waypoints = [];
    times = [];
    time = 0.5;
    waypoint_count = 0;
    
    i = 1;
    while i <= length(points)
        if pen_states(i) % Pen down - start of stroke
            % Move to start of stroke (pen up)
            if waypoint_count > 0
                waypoint_count = waypoint_count + 1;
                stroke_start = points(i,:) + [0,0,0.01];
                T_approach = [R_fixed, stroke_start'; 0,0,0,1];
                waypoints(waypoint_count,:,:) = T_approach;
                times = [times, time];
            end
            
            % Follow stroke
            while i <= length(points) && pen_states(i)
                waypoint_count = waypoint_count + 1;
                T_waypoint = [R_fixed, points(i,:)'; 0,0,0,1];
                waypoints(waypoint_count,:,:) = T_waypoint;
                times = [times, time];
                i = i + 1;
            end
            
            % Lift pen after stroke
            if i <= length(points)
                waypoint_count = waypoint_count + 1;
                lift_pos = points(i-1,:) + [0,0,0.05];
                T_lift = [R_fixed, lift_pos'; 0,0,0,1];
                waypoints(waypoint_count,:,:) = T_lift;
                times = [times, time/2];
            end
        else
            i = i + 1;
        end
    end
    
    % Execute trajectory
    if ~isempty(waypoints)
        fprintf('Executing trajectory with %d waypoints\n', size(waypoints,1));
        [t_traj, p_traj, v_traj] = multipoint_trajectory(q0, waypoints, times);
        
        % Option 1: Simulate in MATLAB
        % simulate(robot, config, t_traj, p_traj, [], v_traj, q0, NaN, false);
        
        % Option 2: Send to real robot
        send_to_robot(t_traj, p_traj);
    else
        fprintf('No valid waypoints generated!\n');
    end
end
