function [t, Td, vd] = LSPB_trajectory( time, Ti, Tf, dt, T )
    % Compute desired position and velocity from selected desired trajectory
    [A, B]        = LSPB( time, 0, T, 0.2*T, [0; Ti(1:3,4)], [T; Tf(1:3,4)] );
    
    LSPB_time     = A(1);
    
    [rotm, omega] = angle_axis_lerp( LSPB_time, Ti(1:3,1:3), Tf(1:3,1:3), T, dt );
    
    Td = [rotm,  A(2:4)'; 
           0, 0, 0, 1];
    vd = [B(2:4)'; omega];
    t  = LSPB_time;
end
