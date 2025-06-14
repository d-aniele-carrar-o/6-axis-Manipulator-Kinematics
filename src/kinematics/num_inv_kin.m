function [q_next, q_dot] = num_inv_kin( q_curr, Te, Td, vd, J )
    % pos_err = Td(1:3,4) - Te(1:3,4);

    % Re = Te(1:3,1:3);
    % Rd = Td(1:3,1:3);
    % Red = Re' * Rd;

    % dtheta = acos( (Red(1,1)+Red(2,2)+Red(3,3)-1) / 2 )
    % r_hat = 1 / (2*sin(dtheta)) * [Red(3,2)-Red(2,3); Red(1,3)-Red(3,1); Red(2,1)-Red(1,2)]
    
    % o_err = dtheta * r_hat
    % o_err = Re * o_err;
    
    % e = [pos_err; o_err]

    % g  = J * e;
    % Wn = 0.0001 * 0.5 * (e' * e)
    % A  = J' * J + Wn
    
    % % Compute joint configuration
    % q_next = q_curr' + A \ g

    % % Compute joint velocities
    % q_dot = J \ vd;

    % Position error
    pos_err = Td(1:3,4) - Te(1:3,4);

    % Orientation error using angle-axis representation
    Re = Te(1:3,1:3);
    Rd = Td(1:3,1:3);
    Red = Re' * Rd;
    
    % Calculate angle of rotation
    dtheta = acos(max(min((trace(Red)-1)/2, 1), -1)); % Clamp to avoid numerical issues
    
    % Calculate axis of rotation (handle small angle case)
    if dtheta < 1e-6
        r_hat = [0; 0; 0];
    else
        r_hat = 1/(2*sin(dtheta)) * [Red(3,2)-Red(2,3); Red(1,3)-Red(3,1); Red(2,1)-Red(1,2)];
    end
    
    % Orientation error vector
    o_err = dtheta * r_hat;
    o_err = Re * o_err; % Transform to world frame
    
    % Combined error vector
    e = [pos_err; o_err];
    
    % Levenberg-Marquardt parameter (damping factor)
    lambda = 0.01;
    E = 0.5 * norm(e);
    Wn = lambda * E * eye(size(J,2));
    
    % Compute joint update
    delta_q = (J'*J + Wn) \ (J'*e);
    
    % Update joint configuration
    q_next = q_curr + delta_q';
    
    % Compute joint velocities
    q_dot = pinv(J) * vd; % Using pseudoinverse for better numerical stability

end
