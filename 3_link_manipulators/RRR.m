clc
params

draw = true;

if draw
    alfa = 340;
    beta = 140;
    l = 1;
    axs  = axes( 'XLim', [-l l], 'YLim', [-l l], 'ZLim', [-0.1 l] );
    view( alfa, beta ); grid on;
    handles(1) = axs;
end


q = [pi/4, 0, pi/6];

% DIRECT KIN
T04 = RRR_DK( q )

if draw
    [~, handlesR] = RRR_DK_draw( q, handles, true );
    pause()
end

% INVERSE KIN
th = RRR_IK( T04 )


% inverse check
if draw
    [T1, ] = RRR_DK_draw( th, handlesR, false )
end


% Geometric Jacobian
% Manipulator: REVOLUTE - REVOLUTE - REVOLUTE
J = RRR_Jacobian( q )


% generate path
p_f   = T04(1:3,4) + [0; -0.6; 0];
T_f   = [eye(3), p_f; 0,0,0,1];
N     = 20;
pts   = lerp_path( T04(1:3,4), p_f, N );

for i=1:N
    pts(:,i)
    T_d = [eye(3), pts(:,i); 0,0,0,1];
    th_d = RRR_IK( T_d )
    RRR_DK_draw( th_d, handles, true )
    pause(0.01)
end











