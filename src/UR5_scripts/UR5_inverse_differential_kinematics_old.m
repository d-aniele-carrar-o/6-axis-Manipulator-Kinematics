function [q_next, limited_q_dot] = UR5_inverse_differential_kinematics( q_curr, Te, Td, vd, K_p, K_o, max_vel )
    params

    % Compute direction step (error) for position and orientation to desired pose
    step = get_step( Te, Td );
    
    J = Jacobian( q_curr );

    % TODO: add min singular value check before inverting Jacobian
    % min_svd = svd(J);
    
    % Inverse differential step: qdot_d = J^-1 * vd
    q_dot = (J + eye(6)*1e-5) \ [vd(1:3) + K_p*step(1:3); vd(4:6) + K_o*step(4:6)];
    % q_dot = J \ [vd(1:3) + K_p*step(1:3); vd(4:6) + K_o*step(4:6)];
    
    % Limit the maximum velocity to given value
    limited_q_dot = get_limited_q_dot( q_dot, max_vel );
    
    % Compute next end-effector position from current joint configuration and computed velocity
    q_next  = (q_curr + limited_q_dot'*dt)';
end
