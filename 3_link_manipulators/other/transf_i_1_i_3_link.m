% homogeneous transformation matrix from frame [i-1] to frame [i]
function [T] = transf_i_1_i_3_link( i, AL, A, D, TH )
    T = rot_tralsX( AL(i), A(i) ) * rot_traslZ( TH(i+1), D(i+1) );
end
