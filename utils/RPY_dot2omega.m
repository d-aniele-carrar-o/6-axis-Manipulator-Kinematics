% transform rpy velocity [psi_dot, theta_dot, phi_dot] to angular velocity [omega_x, omega_y, omega_z]
function [omega] = RPY_dot2omega( rpy_dot )
    theta = rpy_dot(2);
    phi   = rpy_dot(3);

    T = [cos(theta)*cos(phi), -sin(phi), 0;
         cos(theta)*sin(phi),  cos(phi), 0;
                 -sin(theta),         0, 1];

    omega = T * rpy_dot;
end

