clc

% SYMBOLIC SOLUTIONS
syms d1 d4 a2 q1 q2 q3 X Y Z r11 r12 r13 r21 r22 r23 r31 r32 r33

% D-H parameters
% i:  0     1   2     3   4
al = [0., pi/2, 0., pi/2, 0.];
a  = [0.,   0., a2,   0., 0.];
d  = [0.,   d1, 0.,   0., d4];
th = [0.,   q1, q2,   q3, 0.];

% Direct Kinematics
T01 = transf_i_1_i_3_link( 1, al, a, d, th )
T12 = transf_i_1_i_3_link( 2, al, a, d, th )
T23 = transf_i_1_i_3_link( 3, al, a, d, th )
T34 = transf_i_1_i_3_link( 4, al, a, d, th )
T02 = T01*T12
T03 = T02*T23
T04 = T03*T34

P04 = T04(1:3,4)
P04m = [X Y Z]
R04m = [r11, r12, r13; r21, r22, r23; r31, r32, r33]

% INVERSE KIN
q_1 = atan2( r12, -r22 )

P14      = inv(T01) * T04
l_P14    = hypot( P14(1), P14(3) )
cos_beta = (d4^2-a2^2-l_P14^2) / (-2*a2*l_P14)
sin_beta = sqrt( 1 - cos_beta^2 )

beta  = atan2( sin_beta, cos_beta )
alpha = atan2( P14(3), P14(1) )
q_2   = alpha + beta

sin_gamma = l_P14*sin_beta / d4
gamma = atan2( sin_gamma, sqrt(1-sin_gamma^2) )
q_3 = pi - gamma

% Jacobian
xi     = [1, 1, 1];
z_0_i  = [    T01(1:3,3), T02(1:3,3), T03(1:3,3)]
P_dist = [P04-T01(1:3,4), P04-T02(1:3,4), P04-T03(1:3,4)]

% Rotational velocity:
J_o = xi .* z_0_i

% Linear velocity:
J_v = ~xi .* z_0_i + xi .* cross( z_0_i, P_dist, 1 )

% Complete geometric Jacobian;
J = [J_v; J_o]
