% Function for transforming task poses to joint configurations
% Input parameters:
% - qi              : initial joint configuration
% - task_positions  : end-effector poses in task space - [4x4xN]
% Output parameters:
% - joint_positions : joint configurations corresponding to given task
%                     space poses - [6xN]
function [joint_positions] = task2joint_space( qi, task_positions )
    parameters(1)
    
    N               = max( size( task_positions ) );
    joint_positions = [];
    last_q          = qi;
    
    for i=1:N/4
        Td     = task_positions(4*(i-1)+1:4*i,:);
        
        if     manipulator == "UR5"
            H  = UR5_inverse_kinematics_cpp( Td(1:3,4), Td(1:3,1:3), AL, A, D );
        elseif manipulator == "ABB"
            H  = ABB_inverse_kinematics_cpp( Td(1:3,4), Td(1:3,1:3), AL, A, D );
        elseif manipulator == "custom"
            H  = Custom_manipulator_inverse_kinematics_cpp( Td(1:3,4), Td(1:3,1:3), AL, A, D );
        end
        
        q               = get_closer( H, last_q );
        joint_positions = [joint_positions; q];
        last_q          = q;
    end
    
end

