% homogeneous transformation matrix from frame [i-1] to frame [i]
function [T] = transf_i_1_i( i, qi )
    % load manipulator parameters
    params
    
    T = rot_tralsX( AL(i), A(i) ) * rot_traslZ( qi + TH(i+1), D(i+1) );
end
