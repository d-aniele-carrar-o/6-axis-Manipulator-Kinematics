function [step] = get_step( T_i, T_f )
    T = inv(T_f) * T_i;
    T = T_f / T_i;
    R = T(1:3,1:3);
    
    R_d = R-R';
    cross = [R_d(3,2); R_d(1,3); R_d(2,1)];
    theta = atan2( sqrt( cross(1)^2 + cross(2)^2 + cross(3)^2 ), trace(R)+1 );

    if round(theta, 5) == 0
        w = [0; 0; 0];
    else
        w = theta / (2*sin(theta)) * cross;
    end

    v = T_f(1:3,4) - T_i(1:3,4);

    step = [v; w];
end
