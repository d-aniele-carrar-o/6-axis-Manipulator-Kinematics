function [th] = RRR_IK( T04 )
    RRR_params

    % theta 1 -------------------------------------------
    q_1 = atan2( T04(2,4), T04(1,4) );
    
    % theta 2 -------------------------------------------
    T14      = inv(RRR_trasf_i_1_i(1,q_1)) * T04;
    P14      = T14(1:3,4);
    l_P14    = hypot( P14(1), P14(3) );
    
    cos_beta = (D(5)^2-A(3)^2-l_P14^2) / (-2*A(3)*l_P14);
    sin_beta = sqrt( 1 - cos_beta^2 );
    
    beta  = atan2( sin_beta, cos_beta );
    alpha = atan2( P14(3), P14(1) );
    q_2   = alpha + beta;
    
    % theta 3 -------------------------------------------
    sin_gamma = l_P14*sin_beta / D(5);
    gamma     = atan2( sin_gamma, sqrt(1-sin_gamma^2) );
    q_3       = pi/2 - gamma;

    th = [q_1, q_2, q_3];
end

