function [q_d, q_dot_d] = LSPB( t, t_i, t_f, t_b, q_i, q_f )
    % TODO: check if resulting constant velocity trait is above maximum allowed velocity 
    
    % Linear Segments with Parabolic Blends - trapezodial velocity profile
    
    T = t_f - t_i;

    v = (q_f - q_i) / (t_f - t_b);
    a = v / t_b;

    if t < t_b
        q_d     = q_i' + a'/2 * t^2;
        q_dot_d = a'*t;
    elseif t < T-t_b
        q_d     = (q_f' + q_i' - v'*T) / 2 + v'*t;
        q_dot_d = v';
    elseif t < T
        q_d     = q_f' - a'/2 * T^2 + a' * T*t - a'/2 * t^2;
        q_dot_d = v' - a'*(t - T + t_b);
    else
        q_d     = q_f';
        q_dot_d = zeros( 1, 6 );
    end


    % assume general 2nd order polynomial formula for first quadratic segment
    %     q(t) = a0 +  a1t + a2t^2
    % q_dot(t) = a1 + 2a2t
    
    % for t_i = 0:
    %        q(0) = q_i  =>  a0 = q_i
    %    q_dot(0) =   0  =>  a1 = 0
    
    % let's define a new 2nd order polynomìial general formula for second quadratic segment:
    %     q(t) = x0 +  x1t + x2t^2
    % q_dot(t) = x1 + 2x2t
    
    % at this point the unkowns are: a2, x0, x1, x2
    % let's define a linear system of equations
    % x0 + x1t_f       +  x2t_f^2                                    = q_f
    % x0 + x1(t_f-t_b) +  x2(t_f-t_b)^2 - a2[t_b^2 - 2t_b(t_f-2t_b)] = q_i 
    %      x1          + 2x2t_f                                      = 0
    %      x1          + 2x2(t_f-t_b)   - 2a2t_b                     = 0

    % t_m = t_f-t_i;
    % 
    % A = [
    %      [1,       t_m,       t_m^2,                        0];
    %      [1, (t_m-t_b), (t_m-t_b)^2, -t_b^2-2*t_b*(t_m-2*t_b)];
    %      [0,         1,       2*t_m,                        0];
    %      [0,         1, 2*(t_m-t_b),                   -2*t_b]
    %     ];
    % 
    % N = max(size(q_i));
    % b = [q_f'; q_i'; zeros(1,N); zeros(1,N)];
    % 
    % % res: [x0, x1, x2, a2]
    % res = A \ b;
    % 
    % if t <= t_b
    %     q_d     = q_i' + res(4,:)*t^2;
    %     q_dot_d = 2*res(4,:)*t;
    % elseif t <= t_m-t_b
    %     q_d     = q_i' + res(4,:)*t_b^2 + 2*res(4,:)*t_b*(t-t_b);
    %     q_dot_d = 2*res(4,:)*t_b;
    % elseif t <= t_m
    %     q_d     = res(1,:) + res(2,:)*t + res(3,:)*t^2;
    %     q_dot_d = res(2,:) + 2*res(3)*t;
    % else
    %     q_d     = q_f';
    %     % q_dot_d = zeros(N,1);
    %     q_d     = q_f;
    %     q_dot_d = q_dot_f;
    % end
end
