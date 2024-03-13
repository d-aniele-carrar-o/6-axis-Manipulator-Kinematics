function [positions, velocities] = generate_trajectory( q0, Tf, T, traj_type )
    % TO ADD: INITIAL VELOCITY IF DIFFERENT FROM ZERO 
    % (like for ex when adding viapoint in the traj in which the ee do NOT have to come to a rest)
    
    % Make the script indipendent of the manipulator passing the functions used as parameters (like Palop)
    params

    verbose = true;
    
    time   = 0:dt:T;
    N      = length(time);

    poss   = zeros( 4, 4, N );
    vels   = zeros( N, 6 );

    T0     = direct_kinematics_cpp( q0, AL, A, D, TH );
    Te     = zeros( 4 );
    Te     = T0;
    last_Te = zeros( 4 );

    poss(:,:,1) = Te;
    vels(1,:)   = zeros( 1, 6 );

    positions   = q0;
    velocities  = zeros( 1, 6 );

    max_vel   = 3;
    precision_pos    = 0.001;
    precision_orient = 0.001;

    % [delta_pos, delta_ang] = compute_distance( Te, Tf );
    [delta_pos, delta_ang, ~, ~] = compute_distance( Te, last_Te, Tf );
    
    % TODO: set Ks proportional to distance to final position in order to
    % arrive with position and orientation at the same time (more or less)
    K_p = 10;
    K_o = 10;

    q_curr = positions(1,:);

    last_t  = time(1);

    i     = 2;
    count = 5;

    if traj_type == "cubic"
        while (delta_pos > precision_pos || delta_ang > precision_orient) && count > 0
            % Compute desired position and velocity from selected desired trajectory
            [cubic_time, ~, ~]   = cubic_trajectory( time(i), T0, Tf, 0, T );
            [cubic_time, Td, vd] = cubic_trajectory( time(i), T0, Tf, cubic_time-last_t, T );
            last_t = cubic_time;
            
            % Compute velocity which should achieve the desired end-effector pose
            
            % Compute desired joint configuration using inverse differential kinematics
            % [q_next, limited_q_dot] = UR5_inverse_differential_kinematics( q_curr, Te, Td, vd, K_p, K_o, max_vel );
            % Use cpp code
            J = Jacobian_cpp( Te, q_curr, AL, A, D, TH );
            [q_next, limited_q_dot] = inverse_differential_kinematics_cpp( q_curr, Te, Td, vd, J, dt, AL, A, D );

            % writematrix( J, 'J_cpp.xls', 'Range', strcat( strcat('A', num2str(n)), strcat(':F', num2str(n+6)) ));

            last_Te = Te;
            Te      = direct_kinematics_cpp( q_next, AL, A, D, TH );
            
            % Compute position and orientation "distance" from previous position, if below threshold -> stop trajectory
            [delta_pos, delta_ang, delta_pos_last, delta_ang_last] = compute_distance( Te, last_Te, Td );
            
            % Count if 5 consecutive computed poses are too similar -> exit
            if delta_pos_last < precision_pos && delta_ang_last < precision_orient
                count = count - 1;
            else
                count = 5;
            end
            
            fprintf( "compute intermediate viapoints - t=%.4f\n", time(i) )

            % Debug
            if verbose
                i
                count
                limited_q_dot'
                vd'
                [delta_pos, delta_ang]
                [delta_pos_last, delta_ang_last]
                fprintf("-----------------------------------------------------\n")
                % pause()
            end
            % ---------------------------------------------------------------------------------------------------------------------
            
            % Update current joint configuration
            positions  = [positions; q_next'];
            velocities = [velocities; limited_q_dot'];
            q_curr = q_next';

            i = min( i + 1, N );
        end
    elseif traj_type == "quintic"
        while (delta_pos > precision_pos || delta_ang > precision_orient) && count > 0
            % Compute desired position and velocity from selected desired trajectory
            fprintf("start\n")
            % TODO: rewrite better this time + desired pose/vel
            [quintic_time, ~, ~]   = quintic_trajectory( time(i), T0, Tf, 0, T );
            [quintic_time, Td, vd] = quintic_trajectory( time(i), T0, Tf, quintic_time-last_t, T );
            last_t = quintic_time;
            fprintf("quintic_trajectory\n")
            % Compute velocity which should achieve the desired end-effector pose
            
            % Compute desired joint configuration using inverse differential kinematics
            % [q_next, limited_q_dot] = UR5_inverse_differential_kinematics( q_curr, Te, Td, vd, K_p, K_o, max_vel );
            % Use cpp code 
            J = Jacobian_cpp( Te, q_curr, AL, A, D, TH );
            [q_next, limited_q_dot] = inverse_differential_kinematics_cpp( q_curr, Te, Td, vd, J, dt, AL, A, D );
            fprintf("UR5_inverse_differential_kinematics_cpp\n")
            last_Te = Te;
            Te      = direct_kinematics_cpp( q_next, AL, A, D, TH );
            
            % Compute position and orientation "distance" from previous position, if below threshold -> stop trajectory
            [delta_pos, delta_ang, delta_pos_last, delta_ang_last] = compute_distance( Te, last_Te, Td );
            
            % Count if 5 consecutive computed poses are too similar -> exit
            if delta_pos_last < precision_pos/10 && delta_ang_last < precision_orient/10
                count = count - 1;
            else
                count = 5;
            end
            
            fprintf( "compute intermediate viapoints - t=%.4f\n", time(i) )

            % Debug
            if verbose
                i
                limited_q_dot'
                fprintf( "compute intermediate viapoints - end\n" )
                Te
                Td
                vd'
                [delta_pos, delta_ang]
                [delta_pos_last, delta_ang_last]
                fprintf("-----------------------------------------------------\n")
                %pause()
            end
            % ---------------------------------------------------------------------------------------------------------------------
            
            % Update current joint configuration
            positions  = [positions; q_next'];
            velocities = [velocities; limited_q_dot'];
            q_curr = q_next;

            i = min( i + 1, N );
        end
    elseif traj_type == "LSPB"
        while (delta_pos > precision_pos || delta_ang > precision_orient) && count > 0
            % Compute desired position and velocity from selected desired trajectory 
            [LSPB_time, ~, ~]   = LSPB_trajectory( time(i), T0, Tf, 0, T );
            [LSPB_time, Td, vd] = LSPB_trajectory( time(i), T0, Tf, LSPB_time-last_t, T );
            last_t = LSPB_time;
            
            % Compute velocity which should achieve the desired end-effector pose
            
            % Compute desired joint configuration using inverse differential kinematics
            % [q_next, limited_q_dot] = UR5_inverse_differential_kinematics( q_curr, Te, Td, vd, K_p, K_o, max_vel );
            % Use cpp code
            J = Jacobian_cpp( Te, q_curr, AL, A, D, TH );
            [q_next, limited_q_dot] = inverse_differential_kinematics_cpp( q_curr, Te, Td, vd, J, dt, AL, A, D );


            last_Te = Te;
            Te      = direct_kinematics_cpp( q_next, AL, A, D, TH );
            
            % Compute position and orientation "distance" from previous position, if below threshold -> stop trajectory
            [delta_pos, delta_ang, delta_pos_last, delta_ang_last] = compute_distance( Te, last_Te, Td );
            
            % Count if 5 consecutive computed poses are too similar -> exit
            if delta_pos_last < precision_pos/10 && delta_ang_last < precision_orient/10
                count = count - 1;
            else
                count = 5;
            end
            
            fprintf( "compute intermediate viapoints - t=%.4f\n", time(i) )
            
            % Debug
            if verbose
                i
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
            % ---------------------------------------------------------------------------------------------------------------------
            
            % Update current joint configuration

            % TODO: preallocate output matrices
            positions  = [positions; q_next'];
            velocities = [velocities; limited_q_dot'];
            q_curr = q_next;

            i = min( i + 1, N );
        end
    else
        fprintf("Trajectory type not yet supported.\n")
        return
    end
    
end

