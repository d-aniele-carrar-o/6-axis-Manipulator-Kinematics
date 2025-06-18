function T = direct_kinematics_wrapper(q, AL, A, D, TH)
% DIRECT_KINEMATICS_WRAPPER Wrapper for direct_kinematics_cpp function
%
% This function provides a fallback implementation of the direct kinematics
% calculation in case the MEX function is not available.
%
% Inputs:
%   q  - Joint configuration [q1, q2, q3, q4, q5, q6]
%   AL - Alpha DH parameters
%   A  - A DH parameters
%   D  - D DH parameters
%   TH - Theta offset DH parameters
%
% Output:
%   T - 4x4 homogeneous transformation matrix from base to end-effector

    % Try to use the MEX function if available
    try
        T = direct_kinematics_cpp(q, AL, A, D, TH);
        return;
    catch
        % If MEX function is not available, use MATLAB implementation
        fprintf('MEX function direct_kinematics_cpp not available, using MATLAB implementation.\n');
    end
    
    % MATLAB implementation of direct kinematics using DH parameters
    % Initialize transformation matrix
    T = eye(4);
    
    % Number of joints
    n = length(q);
    
    % Compute forward kinematics using DH parameters
    for i = 1:n
        % DH transformation matrix for joint i
        ct = cos(q(i) + TH(i+1));
        st = sin(q(i) + TH(i+1));
        ca = cos(AL(i+1));
        sa = sin(AL(i+1));
        
        % Standard DH transformation matrix
        Ti = [ct, -st*ca, st*sa, A(i+1)*ct;
              st, ct*ca, -ct*sa, A(i+1)*st;
              0, sa, ca, D(i+1);
              0, 0, 0, 1];
        
        % Accumulate transformation
        T = T * Ti;
    end
end