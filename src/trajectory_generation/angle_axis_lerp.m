function [rotm, omega] = angle_axis_lerp( t, rotm_i, rotm_f, T, dt )
    rotm_i_f      = rotm_i' * rotm_f;
    [angle, axis] = rotm2angle_axis( rotm_i_f );
    rotm_d        = angle_axis2rotm( angle*t/T, axis );
    
    rotm = rotm_i * rotm_d;

    if t > 0
        omega = rotm_i * (angle*dt * axis);
    else
        omega = [0;0;0];
    end
end
