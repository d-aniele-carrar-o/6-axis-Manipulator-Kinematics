function [q_d, q_dot_d, q_ddot_d] = quintic_polynomial( t, t_i, t_f, q_i, q_f, q_dot_i, q_dot_f, q_ddot_i, q_ddot_f )
    % general form of quintic polynomial:
    %      q(t-ti) =  a0 +  a1(t-ti) +   a2(t-ti)^2 +   a3(t-ti)^3 +  a4(t-ti)^4 +  a5(t-ti)^5
    %  q_dot(t-ti) =  a1 + 2a2(t-ti) +  3a3(t-ti)^2 +  3a4(t-ti)^3 + 4a5(t-ti)^4
    % q_ddot(t-ti) = 2a2 + 6a3(t-ti) + 12a4(t-ti)^2 + 20a5(t-ti)^3

    
    if t >= t_f
        q_d      = q_f';
        q_dot_d  = q_dot_f';
        q_ddot_d = q_ddot_f';
    else
        % solve the general case with as a linear system:
        T = [[1, t_i, t_i^2,   t_i^3,    t_i^4,    t_i^5];
             [1, t_f, t_f^2,   t_f^3,    t_f^4,    t_f^5];
             [0,   1, 2*t_i, 3*t_i^2,  4*t_i^3,  5*t_i^4];
             [0,   1, 2*t_f, 3*t_f^2,  4*t_f^3,  5*t_f^4];
             [0,   0,     2,   6*t_i, 12*t_i^2, 20*t_i^3];
             [0,   0,     2,   6*t_f, 12*t_f^2, 20*t_f^3]
            ];
        
        % vector containing unknowns [a0, a1, a2, a3, a4, a5] for each field in given
        % q_i/q_f (could be [x,y,z] or joint variables [q1,q2,...])
        A = T \ [q_i'; q_f'; q_dot_i'; q_dot_f'; q_ddot_i'; q_ddot_f'];
        
        % compute desired q and q_dot for given time t
        q_d      = A(1,:) +   A(2,:)*(t-t_i) +    A(3,:)*(t-t_i)^2 +    A(4,:)*(t-t_i)^3 +   A(5,:)*(t-t_i)^4  + A(6,:)*(t-t_i)^5;
        q_dot_d  = A(2,:) + 2*A(3,:)*(t-t_i) +  3*A(4,:)*(t-t_i)^2 +  4*A(5,:)*(t-t_i)^3 + 5*A(6,:)*(t-t_i)^4;
        q_ddot_d = A(3,:) + 6*A(4,:)*(t-t_i) + 12*A(5,:)*(t-t_i)^2 + 20*A(6,:)*(t-t_i)^3 ;
    end
end
