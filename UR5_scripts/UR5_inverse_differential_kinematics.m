function [q, q_dot] = UR5_inverse_differential_kinematics( q0, T_d, precision, max_vel )
    q     = [];
    q_dot = [];

    T = UR5_direct_kinematics( q0 );
    
    delta = norm( T - T_d );

    q(1,:)     = q0;
    q_dot(1,:) = zeros( 1, 6 );

    next_q = q0';
    while delta > precision
        q_curr  = next_q;

        step    = get_step( T, T_d );

        J       = Jacobian( q_curr );

        min_svd = svd(J);

        if min_svd(end) < 0.005
            damping   = 0.04;
            theta_dot = J' / (J * J' + damping^2 * eye(6)) * step;
        else
            % INSERT HERE FEED-FORWARD TERM
            theta_dot = J \ step;
        end
        
        max_vel_check_positive = theta_dot >  max_vel;
        max_vel_check_negative = theta_dot < -max_vel;

        limited_q_dot = max_vel_check_positive * max_vel + ~(max_vel_check_positive + max_vel_check_negative) ...
                            .* theta_dot + max_vel_check_negative * -max_vel;

        next_q = q_curr + limited_q_dot;

        q     = [q; next_q'];
        q_dot = [q_dot; limited_q_dot'];

        T     = UR5_direct_kinematics( next_q );
        delta = norm( T - T_d );
    end
end

