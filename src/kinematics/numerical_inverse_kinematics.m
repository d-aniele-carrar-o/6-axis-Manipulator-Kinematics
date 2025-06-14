function [times, positions, velocities] = numerical_inverse_kinematics(qi, Ti, Tf, ti, tf)
    parameters(1);

    % Current end-effector pose
    Te      = Ti;
    
    T = tf - ti;
    [~, ~, angle] = get_velocity( Ti, Tf, T, 0 );

    % Current joint configuration
    q_curr  = qi;
    last_time = ti;

    N = T / dt;
    times = linspace(ti, tf, N);
    positions = zeros(N, 6);
    velocities = zeros(N, 6);

    for i = 1:N
        [P, V]        = cubic_polynomial( times(i), ti, ti+T, [ti, Ti(1:3,4)', 0], [ti+T, Tf(1:3,4)', angle], zeros(1,5), zeros(1,5) );
        cubic_time    = P(1);
        [rotm, omega] = angle_axis_lerp( cubic_time, Ti(1:3,1:3), Tf(1:3,1:3), T, cubic_time-last_time );
    
        Td            = [rotm,  P(2:4)'; 0, 0, 0, 1];
        vd            = [V(2:4)'; omega];
        last_time     = cubic_time;
        
        % Compute desired joint configuration using numerical inverse kinematics
        J               = Jacobian_cpp( Te, q_curr, AL, A, D, TH );
        [q_next, q_dot] = num_inv_kin( q_curr, Te, Td, vd, J );

        % Update current end-effector pose
        [~, Te] = direct_kinematics( q_next );
        
        % For debug purposes
        if verbose
            count
            q_dot_lim = limited_q_dot'
            next_q = q_next'
            Te
            Td
            vel_d = vd'
            pos_err = delta_pos
            angle_error = delta_ang
            [delta_pos_last, delta_ang_last]
            fprintf("-----------------------------------------------------\n")
            pause()
        end
        
        % Update current joint configuration
        positions(i,:)  = q_next';
        velocities(i,:) = q_dot';
        q_curr = q_next;

    end

end