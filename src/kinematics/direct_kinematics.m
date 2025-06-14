function [T_w_e, Te] = direct_kinematics( q, robot_id )
    % Load DH parameters
    if nargin < 2
        parameters(1);
    else
        parameters(1, robot_id);
    end

    % Compute DK from base_link to end-effector
    Te = direct_kinematics_cpp( q, AL, A, D, TH );

    % Add world2base transformation
    T_w_e = Trf_0 * Te;

end
