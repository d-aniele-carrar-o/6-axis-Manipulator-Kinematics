function [T_w_e, Te] = direct_kinematics( q )
    % Load DH parameters
    parameters(1);

    % Compute DK from base_link to end-effector
    Te = direct_kinematics_cpp( q, AL, A, D, TH );

    % Add world2base transformation
    T_w_e = Trf_0 * Te;

end
