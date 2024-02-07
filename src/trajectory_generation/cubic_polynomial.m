function [q_d, q_dot_d] = cubic_polynomial( t, t_i, t_f, q_i, q_f, q_dot_i, q_dot_f )
    % general form of cubic polynomial:
    %     q(t-ti) = a0 +  a1(t-ti) +  a2(t-ti)^2 + a3(t-ti)^3
    % q_dot(t-ti) = a1 + 2a2(t-ti) + 3a3(t-ti)^2
    
    % q_d(t-ti) assuming q_dot_i = q_dot_f = 0, for any given t_i, t_f, q_i, and q_f
    % q_d = q_i + 3 * (q_f-q_i) / (t_f-t) * (t-t_i)^2 - 2 * (q_f-q_i) / (t_f-t_i)^3 * (t-t_i)^3;
    
    % solve the general case with as a linear system:
    T = [[1, t_i, t_i^2,   t_i^3];
         [1, t_f, t_f^2,   t_f^3];
         [0,   1, 2*t_i, 3*t_i^2];
         [0,   1, 2*t_f, 3*t_f^2]
        ];
    
    % vector containing unknowns [a0, a1, a2, a3] for each field in given
    % q_i/q_f (could be [x,y,z] or joint variables [q1,q2,...])
    N = max(size(q_i));
    A = zeros( 4, N );
    for i=1:N
        A(:,i) = T \ [q_i(i); q_f(i); q_dot_i(i); q_dot_f(i)];
    end
    
    % compute desired q and q_dot for given time t
    q_d     = zeros( N, 1 );
    q_dot_d = zeros( N, 1 );
    for i=1:N
        q_d(i)     = A(1,i) +   A(2,i)*(t-t_i) +   A(3,i)*(t-t_i)^2 + A(4,i)*(t-t_i)^3;
        q_dot_d(i) = A(2,i) + 2*A(3,i)*(t-t_i) + 3*A(4,i)*(t-t_i)^2;
    end
end
