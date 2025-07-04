function [time, positions, velocities] = LSPB_trajectory( ti, tf, blend_perc, qi, qf )
    parameters(1)

    % Trajectory duration (if max vel or max acc are not violated)
    T  = tf - ti;
    tb = T*blend_perc;
    
    % Adjust given trajectory period if maximum velocity is overcome
    if     space == "joint"
        [T, v, ~]     = get_velocity( qi, qf, T, tb );
    elseif space == "task"
        Ti            = direct_kinematics_cpp( qi, AL, A, D, TH );
        [T, v, angle] = get_velocity( Ti, qf, T, tb );
    end
    
    % For IDK time vector is created during the creation of the trajectory,
    % as it does not normally coincide with the given period
    if kinematics == "IK"
        time      = linspace( ti, ti+T, (ti+T)/dt+1 );
        last_time = time(1);
    else
        time      = ti;
        last_time = ti;
    end

    positions  = [];
    velocities = [];

    if kinematics == "IK"

        % Compute desired position and velocity from selected desired trajectory
        if space == "joint"
            for t=time
                % fprintf( "computing intermediate viapoints - t=%.4f\n", t )

                [q_d, q_dot_d] = LSPB( t, ti, ti+T, tb, qi, qf, v );
                
                positions  = [positions;  q_d'];
                velocities = [velocities; q_dot_d'];
            end

        elseif space == "task"
            positions  = [positions;  Ti];
            velocities = [velocities; zeros(1,6)];
            
            for t=time(2:end)
                % fprintf( "computing intermediate viapoints - t=%.4f\n", t )

                [P, V]        = LSPB( t, ti, ti+T, tb, [ti, Ti(1:3,4)', 0], [ti+T, qf(1:3,4)', angle], [T/(T-tb), v] );
                LSPB_time     = P(1);
                [rotm, omega] = angle_axis_lerp( LSPB_time, Ti(1:3,1:3), qf(1:3,1:3), T, LSPB_time-last_time );
                
                positions  = [positions; 
                              rotm,  P(2:4); 
                              0,  0,  0,  1];
                velocities = [velocities;
                              V(2:4)', omega'];
                last_time  = LSPB_time;
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
            fprintf( "computing intermediate viapoints - t=%.4f\n", time(end) )

            [P, V]        = LSPB( time(end), ti, ti+T, tb, [ti, Ti(1:3,4)', 0], [ti+T, qf(1:3,4)', angle], [T/(T-tb), v] );
            LSPB_time     = P(1);
            [rotm, omega] = angle_axis_lerp( LSPB_time, Ti(1:3,1:3), qf(1:3,1:3), T, LSPB_time-last_time );
            
            Td            = [rotm,  P(2:4); 0, 0, 0, 1];
            vd            = [V(2:4)', omega'];
            last_time     = LSPB_time;
            
            % Compute desired joint configuration using inverse differential kinematics
            J                       = Jacobian_cpp( Te, q_curr, AL, A, D, TH );
            [q_next, limited_q_dot] = inverse_differential_kinematics_cpp( q_curr, Te, Td, vd, J, dt, max_vel );

            % Update current end-effector pose (for feed-back action)
            last_Te = Te;
            Te      = direct_kinematics_cpp( q_next, AL, A, D, TH );
            
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

