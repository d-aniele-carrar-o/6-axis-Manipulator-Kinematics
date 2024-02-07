clc

% SYMBOLIC SOLUTIONS
syms q1 q2 q3 a2 d4 X Y Z A B C


% D-H parameters
% i:  0   1       2      3  4
al = [0., 0., -pi/2, -pi/2, 0.];
a  = [0., 0.,   -a2,    0., 0.];
d  = [0., q1,    0.,    0., d4];
th = [0., 0.,    q2,    q3, 0.];



% Direct Kinematics
T01 = transf_i_1_i( 1, al, a, d, th )
T12 = transf_i_1_i( 2, al, a, d, th )
T23 = transf_i_1_i( 3, al, a, d, th )
T34 = transf_i_1_i( 4, al, a, d, th )
T02 = T01*T12
T03 = T02*T23
T04 = T03*T34

P04  = T04(1:3,4)
P04m = [X Y Z]

% INVERSE KIN
q_2 = atan2( -P04m(2), -P04m(1) )

if cos(q2) ~= 0
    sin_q3 = (P04m(1) - cos(q2)*a2) / (-cos(q2)*d4)
else
    sin_q3 = (P04m(2) - sin(q2)*a2) / (-sin(q2)*d4)
end

cos_q3 = sqrt(1-sin(q3)^2)
q3_1   = atan2( sin(q3), cos(q3) )
q3_2   = -q3_1 - pi

q1_1 = P04m(3) + d4*cos(q3)
q1_2 = P04m(3) - d4*cos(q3)


% Geometric Jacobian
xi = [0, 1, 1];

z_0_i = [T01(1:3,3), T02(1:3,3), T03(1:3,3)]
P_dist  = [P04-T01(1:3,4), P04-T02(1:3,4), P04-T03(1:3,4)]

J_o   = xi .* z_0_i

J_v   = ~xi .* z_0_i + xi .* cross( z_0_i, P_dist, 1 )

J = [J_v; J_o]

%% NUMERICAL SOLUTION

clc

draw = true;

if draw
    alfa = 340;
    beta = 140;
    l = 1;
    axs  = axes( 'XLim', [-l l], 'YLim', [-l l], 'ZLim', [-l l] );
    view( alfa, beta ); grid on;
    handles(1) = axs;
end

% DH parameters
%        i:    0      1    2   3
AL =          [0., 0., -pi/2, -pi/2,  0.];
 A =          [0., 0.,  -0.5,    0.,  0.];
 D = @(q1)    [0., q1,    0.,    0., 0.3];
TH = @(q2,q3) [0., 0.,    q2,    q3,  0.];

q = [0.5, -pi/4, -pi/4];

% DIRECT KIN
T01 =       transf_i_1_i( 1, AL, A, D(q(1)), TH(q(2),q(3)) );
T02 = T01 * transf_i_1_i( 2, AL, A, D(q(1)), TH(q(2),q(3)) );
T03 = T02 * transf_i_1_i( 3, AL, A, D(q(1)), TH(q(2),q(3)) );
T04 = T03 * transf_i_1_i( 4, AL, A, D(q(1)), TH(q(2),q(3)) )


if draw
    DK_draw( AL, A, D(q(1)), TH(q(2),q(3)), handles, true );
    pause()
end

% INVERSE KIN
P04 = T04(1:3,4)
D_fix = D(q(1));

q2 = atan2( -P04(2), -P04(1) )

if cos(q2) ~= 0
    sin_q3 = (P04(1) - cos(q2)*A(3)) / (-cos(q2)*D_fix(5));
else
    sin_q3 = (P04(2) - sin(q2)*A(3)) / (-sin(q2)*D_fix(5));
end

cos_q3 = sqrt(1-sin_q3^2);
q3_1 = atan2( sin_q3, cos_q3 )
q3_2 = -q3_1 - pi

q1_1 = P04(3) + D_fix(5)*cos_q3
q1_2 = P04(3) - D_fix(5)*cos_q3

% inverse check
if draw
    [T1, ~] = DK_draw( AL, A, D(q1_1), TH(q2,q3_1), handles, true )
    pause()
    [T2, ~] = DK_draw( AL, A, D(q1_2), TH(q2,q3_2), handles, true )
end



















