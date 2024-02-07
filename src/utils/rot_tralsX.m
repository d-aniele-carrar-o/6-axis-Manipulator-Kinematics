% homogeneous transformation T_x[i-1] = R(alpha[i-1], x[i-1]) * T(a[i-1], x[i-1])
function [T] = rot_tralsX( angle, offset )
    T = [1,          0,           0, offset;
         0, cos(angle), -sin(angle),      0;
         0, sin(angle),  cos(angle),      0;
         0,          0,           0,      1];
end
