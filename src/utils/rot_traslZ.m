% homogeneous transformation T_z[i] = R(theta[i], z[i]) * T(d[i], z[i])
function [T] = rot_traslZ( angle, offset )
    T = [cos(angle), -sin(angle), 0,      0;
         sin(angle),  cos(angle), 0,      0;
                  0,          0, 1, offset;
                  0,          0, 0,      1];
end
