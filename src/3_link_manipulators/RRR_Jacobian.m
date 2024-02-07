function [J] = RRR_Jacobian( q )
    T01 =       RRR_trasf_i_1_i( 1, q(1) );
    T02 = T01 * RRR_trasf_i_1_i( 2, q(2) );
    T03 = T02 * RRR_trasf_i_1_i( 3, q(3) );
    T04 = T03 * RRR_trasf_i_1_i( 4, 0 );

    P04 = T04(1:3,4);
    
    z_0_i  = [    T01(1:3,3),     T02(1:3,3),     T03(1:3,3)];
    P_dist = [P04-T01(1:3,4), P04-T02(1:3,4), P04-T03(1:3,4)];
    
    % Rotational velocity:
    J_o = z_0_i;
    
    % Linear velocity:
    J_v = cross( z_0_i, P_dist, 1 );
    
    % Complete geometric Jacobian;
    J = [J_v; J_o];
end

