function [positions, velocities] = generate_trajectory( q0, T_f, T, traj_type )
    % THINK ABOUT HOW TO ADD FEED-FORWARD TERM INTO INVERSE DIFF KIN
    params

    T0 = UR5_direct_kinematics( q0 );
    
    N    = T / dt;
    time = linspace( 0, T, N );
    poss = zeros( 4, 4, N );
    vels = zeros( N, 6 );

    last = 0;

    if traj_type == "cubic"
        for i=1:N
            [pos, vel]    = cubic_polynomial( time(i), 0, T, T0(1:3,4), T_f(1:3,4), zeros(3,1), zeros(3,1) );
            cubic_time    = cubic_polynomial( time(i), 0, T, 0, T, 0, 0 );
            [rotm, omega] = angle_axis_lerp( cubic_time, T0(1:3,1:3), T_f(1:3,1:3), T, cubic_time-last );
            last = cubic_time;

            poss(:,:,i) = [rotm, pos; 0,0,0,1];
            vels(i,:)   = [vel; omega];
        end
    end

    positions  = q0;
    velocities = zeros( 1, 6 );
    
    precision = 0.001;
    max_vel   = 2;
    
    q_curr = positions(1,:);
    for i=2:N
        T_d = poss(:,:,i);
        
        fprintf( "t=%.4f - compute intermediate viapoints - start\n", time(i) )
        [q, q_dot] = UR5_inverse_differential_kinematics( q_curr, T_d, precision, max_vel );
        fprintf( "compute intermediate viapoints - end\n" )

        positions  = [positions; q];
        velocities = [velocities; q_dot];

        q_curr = positions(end,:);
    end

end

