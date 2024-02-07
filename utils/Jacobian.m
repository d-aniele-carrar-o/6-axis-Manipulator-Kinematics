% Computation of Geometric Jacobian of a six-revolute-axes manipulator
function [J] = Jacobian( th )
    
    T01 = transf_i_1_i( 1, th(1) );
    T02 = T01 * transf_i_1_i( 2, th(2) );
    T03 = T02 * transf_i_1_i( 3, th(3) );
    T04 = T03 * transf_i_1_i( 4, th(4) );
    T05 = T04 * transf_i_1_i( 5, th(5) );
    T06 = T05 * transf_i_1_i( 6, th(6) );
    
    P06   = T06(1:3,4);
    zetas = [    T01(1:3,3),     T02(1:3,3),     T03(1:3,3),     T04(1:3,3),     T05(1:3,3),     T06(1:3,3)];
    dists = [P06-T01(1:3,4), P06-T02(1:3,4), P06-T03(1:3,4), P06-T04(1:3,4), P06-T05(1:3,4), P06-T06(1:3,4)];

    J_o = zetas;
    J_v = cross( zetas, dists, 1 );

    J   = [J_v; J_o];
    