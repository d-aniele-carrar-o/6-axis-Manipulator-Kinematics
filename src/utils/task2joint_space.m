% Function for transforming task poses to joint configurations
% Input parameters:
% - qi              : initial joint configuration
% - task_positions  : end-effector poses in task space - [4x4xN]
% - task_velocities : task space end-effector velocities - [6xN]
% Output parameters:
% - qs              : joint configurations corresponding to given task
%                     space poses - [6xN]
% - q_dots          : joint space velocities corresponding to given task
%                     space velocities
function [qs, q_dots] = task2joint_space( qi, task_positions, task_velocities )
    parameters(1)
    
    N      = size( task_positions, 1 );

    qs     = zeros( N/4, 6 );
    q_dots = zeros( N/4, 6 );
    last_q = qi;
    
    for i=1:N/4
        % Extract end-effector pose
        Td = task_positions(4*(i-1)+1:4*i,:);
        
        % Compute corresponding joint configuration via IK
        if     manipulator == "UR5" || manipulator == "UR3e"
            H  = UR5_inverse_kinematics_cpp( Td(1:3,4), Td(1:3,1:3), AL, A, D );
        elseif manipulator == "ABB"
            H  = ABB_inverse_kinematics_cpp( Td(1:3,4), Td(1:3,1:3), AL, A, D );
        elseif manipulator == "custom"
            H  = Custom_manipulator_inverse_kinematics_cpp( Td(1:3,4), Td(1:3,1:3), AL, A, D, TH );
        end
        
        % Extract closer solution to previous joint configuration
        q = get_closer( H, last_q );
        
        % Velocities are given, transform them into joint space
        if nargin > 2
            % Compute joint velocities using inverse Jacobian
            J     = Jacobian_cpp( Td, q, AL, A, D, TH );
            q_dot = J \ task_velocities(i,:)';
            q_dots(i,:) = q_dot';
        end

        qs(i,:)     = q;
        last_q      = q;

    end
    
end

