function [T06] = ABB_direct_kinematics( q )
    % load manipulator parameters
    params

    % homogeneous transformation matrix from frame [i-1] to frame [i]:
    % [i-1]T[i] = Rot(x[i-1],alpha(i-1)) * Trasl(x[i-1],a[i-1]) * Rot(z[i],theta[i]) * Trasl(z[i],d[i])

    % homogeneous transformation matrix frame 0 -> 1
    T01 = @(q1) rot_tralsX( AL(1), A(1) ) * rot_traslZ( q1, D(2) );
    
    % homogeneous transformation matrix frame 1 -> 2
    T12 = @(q2) rot_tralsX( AL(2), A(2) ) * rot_traslZ( q2, D(3) );
    
    % homogeneous transformation matrix frame 2 -> 3
    T23 = @(q3) rot_tralsX( AL(3), A(3) ) * rot_traslZ( q3, D(4) );
    
    % homogeneous transformation matrix frame 3 -> 4
    T34 = @(q4) rot_tralsX( AL(4), A(4) ) * rot_traslZ( q4, D(5) );

    % homogeneous transformation matrix frame 4 -> 5
    T45 = @(q5) rot_tralsX( AL(5), A(5) ) * rot_traslZ( q5, D(6) );

    % homogeneous transformation matrix frame 5 -> 6
    T56 = @(q6) rot_tralsX( AL(6), A(6) ) * rot_traslZ( q6, D(7) );

    % check joint limits
    % q = check_limits( q, limits );

    % compute matrices with actual joint variables values q=[q1, q2, q3, q4, q5, q6]
    T01m   = T01(q(1));
    T12m   = T12(q(2));
    T23m   = T23(q(3));
    T34m   = T34(q(4));
    T45m   = T45(q(5));
    T56m   = T56(q(6));
    
    % compute total transformation matrix frame 0 -> 6
    T06 = T01m*T12m*T23m*T34m*T45m*T56m;
end

