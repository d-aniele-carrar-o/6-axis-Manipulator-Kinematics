function [q_d, q_dot_d, q_ddot_d] = quintic_polynomial( t, ti, tf, q_i, q_f, q_dot_i, q_dot_f, q_ddot_i, q_ddot_f )
    % general form of quintic polynomial:
    %      q(t-ti) =  a0 +  a1(t-ti) +   a2(t-ti)^2 +   a3(t-ti)^3 +  a4(t-ti)^4 +  a5(t-ti)^5
    %  q_dot(t-ti) =  a1 + 2a2(t-ti) +  3a3(t-ti)^2 +  3a4(t-ti)^3 + 4a5(t-ti)^4
    % q_ddot(t-ti) = 2a2 + 6a3(t-ti) + 12a4(t-ti)^2 + 20a5(t-ti)^3

    if t < ti
        q_d      = q_i;
        q_dot_d  = q_dot_i;
        q_ddot_d = q_ddot_i;
    elseif t >= tf
        q_d      = q_f;
        q_dot_d  = q_dot_f;
        q_ddot_d = q_ddot_f;
    else
        % solve the general case with as a linear system:
        T = [[1, ti,  ti^2,    ti^3,    ti^4,    ti^5];
             [1, tf,  tf^2,    tf^3,    tf^4,    tf^5];
             [0,   1, 2*ti,  3*ti^2,  4*ti^3,  5*ti^4];
             [0,   1, 2*tf,  3*tf^2,  4*tf^3,  5*tf^4];
             [0,   0,     2,   6*ti, 12*ti^2, 20*ti^3];
             [0,   0,     2,   6*tf, 12*tf^2, 20*tf^3]
            ];
        
        % vector containing unknowns [a0, a1, a2, a3, a4, a5] for each field in given
        % q_i/q_f (could be [x,y,z] or joint variables [q1,q2,...])
        A = T \ [q_i; q_f; q_dot_i; q_dot_f; q_ddot_i; q_ddot_f];
        
        % compute desired q and q_dot for given time t
        q_d      = A(1,:) +   A(2,:)*(t-ti) +    A(3,:)*(t-ti)^2 +    A(4,:)*(t-ti)^3 + A(5,:)*(t-ti)^4  + A(6,:)*(t-ti)^5;
        q_dot_d  = A(2,:) + 2*A(3,:)*(t-ti) +  3*A(4,:)*(t-ti)^2 +  4*A(5,:)*(t-ti)^3 + 5*A(6,:)*(t-ti)^4;
        q_ddot_d = A(3,:) + 6*A(4,:)*(t-ti) + 12*A(5,:)*(t-ti)^2 + 20*A(6,:)*(t-ti)^3 ;
    end
    
end
