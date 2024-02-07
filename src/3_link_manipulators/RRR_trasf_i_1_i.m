% homogeneous transformation matrix from frame [i-1] to frame [i]
function [T] = RRR_trasf_i_1_i( i, q )
    RRR_params
    T = rot_tralsX( AL(i), A(i) ) * rot_traslZ( q, D(i+1) );
end
