function [joint_positions] = task2joint_space( qi, task_positions )
    params
    
    N               = max( size( task_positions ) );
    joint_positions = [];
    last_q          = qi;
    
    for i=1:N/4
        Td     = task_positions(4*(i-1)+1:4*i,:);
        
        if manipulator     == "UR5"
            H  = UR5_inverse_kinematics_cpp( Td(1:3,4), Td(1:3,1:3), AL, A, D, TH );
        elseif manipularor == "ABB"
            % TODO: implement in cpp
            H  = ABB_inverse_kinematics( Td(1:3,4), Td(1:3,1:3), AL, A, D, TH );
        elseif manipulator == "custom"
            H  = Custom_manipulator_inverse_kinematics_cpp( Td(1:3,4), Td(1:3,1:3), AL, A, D, TH );
        end
        
        q               = get_closer( H, last_q );
        joint_positions = [joint_positions; q];
        last_q          = q;
    end

end

