clc

% SYMBOLIC SOUTION
syms q1 q2 q3 a2 X Y Z q2_1 q2_2

% D-H parameter
% i:  0       1    2   3   4
al = [0., -pi/2,  0., 0.]
a  = [0.,    0., -a2, 0.]
d  = [0.,    q1,  0., q3]
th = [0.,    0.,  q2, 0.]

T01 = transf_i_1_i( 1, al, a, d, th )
T12 = transf_i_1_i( 2, al, a, d, th )
T23 = transf_i_1_i( 3, al, a, d, th )
T02 = T01*T12
T03 = T02*T23

P03  = T03(1:3,4)
P03m = [X Y Z]


% INVERSE KIN
% system of equations:
% P03m = P03

q3 = P03m(2)

cos_q2 = -P03m(1) / a2
sin_q2 = sqrt(1-cos_q2^2)
q_2_1 = atan2( sin_q2, cos_q2 )
q_2_2 = -q2_1

q1_1 = P03m(3) + a2*sin(q2_1)
q1_2 = P03m(3) - a2*sin(q2_2)


% Geometric Jacobian
xi = [0, 1, 0];

z_0_i = [T01(1:3,3), T02(1:3,3), T03(1:3,3)]
P_dist  = [P03-T01(1:3,4), P03-T02(1:3,4), P03-T03(1:3,4)]

J_o   = xi .* z_0_i

J_v   = ~xi .* z_0_i + xi .* cross( z_0_i, P_dist, 1 )

J = [J_v; J_o]


%% NUMERIC IMPLEMENTATION
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
AL =          [0, -pi/2,    0,  0];
 A =          [0,     0, -0.5,  0];
 D = @(q1,q3) [0,    q1,    0, q3];
TH = @(q2)    [0,     0,   q2,  0];

q = [0.5, -pi/4, 0.3];

% DIRECT KIN
T01 =       transf_i_1_i( 1, AL, A, D(q(1),q(3)), TH(q(2)) );
T02 = T01 * transf_i_1_i( 2, AL, A, D(q(1),q(3)), TH(q(2)) );
T03 = T02 * transf_i_1_i( 3, AL, A, D(q(1),q(3)), TH(q(2)) )


if draw
    [~, handlesR] = DK_draw( AL, A, D(q(1),q(3)), TH(q(2)), handles, true );
    pause()
end

% INVERSE KIN
P03 = T03(1:3,4)

q3 = P03(2)

cos_q2 = P03(1) / A(3);
sin_q2 = sqrt(1-cos_q2^2);
q2_1 = atan2( sin_q2, cos_q2 )
q2_2 = -q2_1

q1_1 = P03(3) + A(3)*sin_q2
q1_2 = P03(3) - A(3)*sin_q2

% inverse check
if draw
    [T1, ~] = DK_draw( AL, A, D(q1_1, q3), TH(q2_1), handles, true )
    pause()
    [T2, ~] = DK_draw( AL, A, D(q1_2, q3), TH(q2_2), handles, true )
end





