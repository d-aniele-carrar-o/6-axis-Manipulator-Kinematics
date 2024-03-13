function [limited_q_dot] = get_limited_q_dot( q_dot, max_vel )
    max_vel_check_positive = q_dot >  max_vel;
    max_vel_check_negative = q_dot < -max_vel;

    limited_q_dot = max_vel_check_positive * max_vel + ~(max_vel_check_positive + max_vel_check_negative) ...
                      .* q_dot + max_vel_check_negative * -max_vel;
end
