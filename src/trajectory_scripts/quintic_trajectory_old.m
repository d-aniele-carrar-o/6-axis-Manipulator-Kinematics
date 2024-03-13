function [t, Td, vd] = quintic_trajectory( time, Ti, Tf, dt, T )
    % Compute desired position and velocity from selected desired trajectory in task space
    [A, B, ~] = quintic_polynomial( time, 0, T, [0;Ti(1:3,4)], [T;Tf(1:3,4)], zeros(4,1), zeros(4,1), zeros(4,1), zeros(4,1) );
    
    quintic_time  = A(1);

    [rotm, omega] = angle_axis_lerp( quintic_time, Ti(1:3,1:3), Tf(1:3,1:3), T, dt );

    Td = [rotm,  A(2:4)'; 
           0, 0, 0, 1];
    vd = [B(2:4)'; omega];
    t  = quintic_time;
end
