function [time, positions, velocities] = quintic_trajectory( qi, qf, ti, tf, robot_id )
    if nargin < 4
        robot_id = 1;
    end
    parameters(1, robot_id)

    % Trajectory duration (if max vel or max acc are not violated)
    T  = tf - ti;
    
    % Adjust given trajectory period if maximum velocity is overcome
    if     space == "joint"
        [T, ~, ~]     = get_velocity( qi, qf, T, 0 );
    elseif space == "task"
        [~, Ti]       = direct_kinematics( qi, robot_id );
        [T, ~, angle] = get_velocity( Ti, qf, T, 0 );
    end
    
    % For IDK time vector is created during the creation of the trajectory,
    % as it does not normally coincide with the given period
    if kinematics == "IK"
        time      = linspace( 0, T, T/dt+1 );
        last_time = 0;
    else
        time      = 0;
        last_time = 0;
    end

    positions  = [];
    velocities = [];

    if kinematics == "IK"
        
        % Compute desired position and velocity from selected desired trajectory
        if space == "joint"
            for t=time
                % fprintf( "computing intermediate viapoints - t=%.4f\n", t )

                [q_d, q_dot_d] = quintic_polynomial( t, 0, T, qi, qf, zeros(1,6), zeros(1,6), zeros(1,6), zeros(1,6) );
                
                positions  = [positions;  q_d];
                velocities = [velocities; q_dot_d];
            end

        elseif space == "task"
            positions  = [positions;  Ti];
            velocities = [velocities; zeros(1,6)];
            
            for t=time(2:end)
                % fprintf( "computing intermediate viapoints - t=%.4f\n", t )

                [P, V]        = quintic_polynomial( t, 0, T, [0, Ti(1:3,4)', 0], [T, qf(1:3,4)', angle], zeros(1,5), zeros(1,5), zeros(1,5), zeros(1,5) );
                quintic_time  = P(1);
                [rotm, omega] = angle_axis_lerp( quintic_time, Ti(1:3,1:3), qf(1:3,1:3), T, quintic_time-last_time );
                
                positions  = [positions; 
                              rotm,  P(2:4)'; 
                              0,  0,  0,  1];
                velocities = [velocities;
                              V(2:4), omega'];
                last_time  = quintic_time;
            end

        end
    
    elseif kinematics == "IDK"
        % Compute desired position and velocity from selected desired trajectory
        
        % TODO: add "joint" case where q_i and q_f are expressed in joint space
        
        % Current end-effector pose
        Te      = Ti;
        last_Te = zeros( 4 );
        
        positions  = [positions;  qi];
        velocities = [velocities; zeros(1,6)];
        
        % Current joint configuration
        q_curr  = positions(end,:);
        
        % Initial error between current and desired poses
        [delta_pos, delta_ang, ~, ~] = compute_distance( Te, last_Te, qf );
        
        while (delta_pos > precision_pos || delta_ang > precision_orient) && (count > 0 || time(end) < T-2*dt)
            % Compute desired position and velocity from selected desired trajectory
            time          = [time, time(end)+dt];
            % fprintf( "computing intermediate viapoints - t=%.4f\n", time(end) )

            [P, V]        = quintic_polynomial( time(end), 0, T, [0, Ti(1:3,4)', 0], [T, qf(1:3,4)', angle], zeros(1,5), zeros(1,5), zeros(1,5), zeros(1,5) );
            quintic_time  = P(1);
            [rotm, omega] = angle_axis_lerp( quintic_time, Ti(1:3,1:3), qf(1:3,1:3), T, quintic_time-last_time );
        
            Td            = [rotm,  P(2:4)'; 0, 0, 0, 1];
            vd            = [V(2:4)'; omega];
            last_time     = quintic_time;
            
            % Compute desired joint configuration using inverse differential kinematics
            J                       = Jacobian_cpp( Te, q_curr, AL, A, D, TH );
            [q_next, limited_q_dot] = inverse_differential_kinematics_cpp( q_curr, Te, Td, vd, J, dt, max_vel );

            % Update current end-effector pose
            last_Te = Te;
            [~, Te] = direct_kinematics( q_next, robot_id );
            
            % Compute position and orientation "distance" from previous position, if below threshold -> stop trajectory
            [delta_pos, delta_ang, delta_pos_last, delta_ang_last] = compute_distance( Te, last_Te, qf );

            % Count if 5 consecutive computed poses are too similar -> exit
            if delta_pos_last < precision_pos*dt && delta_ang_last < precision_orient*dt && time(end) > T-2*dt
                count = count - 1;
            else
                count = 5;
            end
            
            % For debug purposes
            if verbose
                count
                limited_q_dot'
                q_next
                Te
                Td
                vd'
                [delta_pos, delta_ang]
                [delta_pos_last, delta_ang_last]
                fprintf("-----------------------------------------------------\n")
                % pause()
            end
            
            % Update current joint configuration
            positions  = [positions;  q_next'];
            velocities = [velocities; limited_q_dot'];
            q_curr = q_next;

        end
        
    end

end
