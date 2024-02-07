clc

% SYMBOLIC SOLUTIONS
syms d2 d4 q1 q2 q3 X Y Z q2_1 q2_2

% D-H parameters
% i:  0       1      2   3   4
al = [0., -pi/2, -pi/2, 0., 0.];
a  = [0.,    0.,    0., 0., 0.];
d  = [0.,    0.,    d2, q3, d4];
th = [0.,    q1,    q2, 0., 0.];

% Direct Kinematics
T01 = transf_i_1_i( 1, al, a, d, th )
T12 = transf_i_1_i( 2, al, a, d, th )
T23 = transf_i_1_i( 3, al, a, d, th )
T34 = transf_i_1_i( 4, al, a, d, th )
T02 = T01*T12
T03 = T02*T23
T04 = T03*T34

P04 = T04(1:3,4)
P04m = [X Y Z]

% INVERSE KIN
q_3 = (-2*d4 + sqrt( 4*d4^2 - 4*( d4^2 + d2^2 - P04m(1)^2 - P04m(2)^2 - P04m(3)^2 ) ))/2

sin_th2 = sqrt( (P04m(1)^2 + P04m(2)^2 - d2^2) / (q_3 + d4)^2 )
q_2_1   = -atan2( sin_th2, sqrt(1-sin_th2^2) )
q_2_2   = -q_2_1

alph  = atan2( -P04m(1), P04m(2) )
beta1 = atan2( (q3+d4)*sin(q2_2), d2 )
beta2 = atan2( (q3+d4)*sin(q2_1), d2 )

q_1_1 = alph + beta1
q_1_2 = alph + beta2


% Jacobian
xi     = [1, 1, 0];
z_0_i  = [T01(1:3,3), T02(1:3,3), T03(1:3,3)]
P_dist = [P04-T01(1:3,4), P04-T02(1:3,4), P04-T03(1:3,4)]

% Rotational velocity:
J_o = xi .* z_0_i

% Linear velocity:
J_v = ~xi .* z_0_i + xi .* cross( z_0_i, P_dist, 1 )

% Complete geometric Jacobian;
J = [J_o; J_v]



%% NUMERIC IMPLEMENTATION

draw = true;

if draw
    alfa = 340;
    beta = 140;
    l = 1;
    axs  = axes( 'XLim', [-l l], 'YLim', [-l l], 'ZLim', [-l 0.2] );
    view( alfa, beta ); grid on;
    handles(1) = axs;
end


% DH parameters
%        i:    0      1      2   3    4
AL =          [0, -pi/2, -pi/2,  0,   0];
 A =          [0,     0,     0,  0,   0];
 D = @(q3)    [0,     0,   0.5, q3, 0.5];
TH = @(q1,q2) [0,    q1,    q2,  0,   0];

q = [pi/4, -pi/4, 0.3];

% DIRECT KIN
T01 =       transf_i_1_i( 1, AL, A, D(q(3)), TH(q(1),q(2)) )
T02 = T01 * transf_i_1_i( 2, AL, A, D(q(3)), TH(q(1),q(2)) )
T03 = T02 * transf_i_1_i( 3, AL, A, D(q(3)), TH(q(1),q(2)) )
T04 = T03 * transf_i_1_i( 4, AL, A, D(q(3)), TH(q(1),q(2)) )

if draw
    [~, handlesR] = DK_draw( AL, A, D(q(3)), TH(q(1),q(2)), handles, true );
    pause()
end

% INVERSE KIN
D_fix = D(q(3))

P04 = T04(1:3,4)

q_3 = (-2*D_fix(5) + sqrt( 4*D_fix(5)^2 - 4*( D_fix(5)^2 + D_fix(3)^2 - P04(1)^2 - P04(2)^2 - P04(3)^2 ) ))/2

sin_th2 = sqrt( (P04(1)^2 + P04(2)^2 - D_fix(3)^2) / (q_3 + D_fix(5))^2 );
q_2_1   = -atan2( sin_th2, sqrt(1-sin_th2^2) )
q_2_2   = -q_2_1

alph  = atan2( -P04(1), P04(2) );
beta1 = atan2( (q_3+D_fix(5))*sin(q_2_2), D_fix(3) );
beta2 = atan2( (q_3+D_fix(5))*sin(q_2_1), D_fix(3) );

q_1_1 = alph + beta1
q_1_2 = alph + beta2


% inverse check
if draw
    [T1, ~] = DK_draw( AL, A, D(q_3), TH(q_1_1, q_2_1), handles, true )
    pause()
    [T2, ~] = DK_draw( AL, A, D(q_3), TH(q_1_2, q_2_2), handles, true )
end


% Geometric Jacobian
% Manipulator: REVOLUTE - REVOLUTE - PRISMATIC

xi     = [1, 1, 0, 0];
z_0_i  = [T01(1:3,3), T02(1:3,3), T03(1:3,3), T04(1:3,3)];
P_dist = [P04-T01(1:3,4), P04-T02(1:3,4), P04-T03(1:3,4), P04-T04(1:3,4)];

% Rotational velocity:
J_o = xi .* z_0_i

% Linear velocity:
J_v = ~xi .* z_0_i + xi .* cross( z_0_i, P_dist, 1 )

% Complete geometric Jacobian;
J = [J_o; J_v]





