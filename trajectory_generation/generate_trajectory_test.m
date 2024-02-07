function [positions, velocities] = generate_trajectory_test( T_i, T_f, T, traj_type )
    params

    N    = T / dt;
    time = linspace( 0, T, N );
    positions  = zeros( 4, 4, N );
    velocities = zeros( N, 6 );

    last = 0;

    if traj_type == "cubic"
        for i=1:N
            [pos, vel]    = cubic_polynomial( time(i), 0, T, T_i(1:3,4), T_f(1:3,4), zeros(3,1), zeros(3,1) );
            cubic_time    = cubic_polynomial( time(i), 0, T, 0, T, 0, 0 );
            [rotm, omega] = angle_axis_lerp( cubic_time, T_i(1:3,1:3), T_f(1:3,1:3), T, cubic_time-last );
            last = cubic_time;

            positions(:,:,i) = [rotm, pos; 0,0,0,1];
            velocities(i,:)  = [vel; omega];
        end
    end
end

