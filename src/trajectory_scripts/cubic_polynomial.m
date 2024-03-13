function [q_d, q_dot_d] = cubic_polynomial( t, ti, tf, q_i, q_f, q_dot_i, q_dot_f )
    % general form of cubic polynomial:
    %     q(t-ti) = a0 +  a1(t-ti) +  a2(t-ti)^2 + a3(t-ti)^3
    % q_dot(t-ti) = a1 + 2a2(t-ti) + 3a3(t-ti)^2
    
    % q_d(t-ti) assuming q_dot_i = q_dot_f = 0, for any given ti, tf, q_i, and q_f
    % q_d = q_i + 3 * (q_f-q_i) / (tf-t) * (t-ti)^2 - 2 * (q_f-q_i) / (tf-ti)^3 * (t-ti)^3;
    
    if t < ti
        q_d     = q_i;
        q_dot_d = q_dot_i;
    elseif t >= tf
        q_d     = q_f;
        q_dot_d = q_dot_f;
    else
        % solve the general case with as a linear system:
        T = [[1, ti, ti^2,   ti^3];
             [1, tf, tf^2,   tf^3];
             [0,  1, 2*ti, 3*ti^2];
             [0,  1, 2*tf, 3*tf^2]
            ];
        
        % vector containing unknowns [a0, a1, a2, a3] for each field in given
        % q_i/q_f (could be [x,y,z] or joint variables [q1,q2,...])
        A = T \ [q_i; q_f; q_dot_i; q_dot_f];
        
        % compute desired q and q_dot for given time t
        q_d     = A(1,:) +   A(2,:)*(t-ti) +   A(3,:)*(t-ti)^2 + A(4,:)*(t-ti)^3;
        q_dot_d = A(2,:) + 2*A(3,:)*(t-ti) + 3*A(4,:)*(t-ti)^2;
    end
    
end
