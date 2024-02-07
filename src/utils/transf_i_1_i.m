% homogeneous transformation matrix from frame [i-1] to frame [i]
function [T] = transf_i_1_i( i, theta_i )
    % load manipulator parameters
    params
    
    T = rot_tralsX( AL(i), A(i) ) * rot_traslZ( theta_i, D(i+1) );
end
