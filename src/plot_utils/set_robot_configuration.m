% Function for setting real-robot urdf joint configuration
% Parameters:
% - q   : manipulator's joint configuration - [6x1] or [8x1] if gripper
function [config] = set_robot_configuration( q, config )

    % Number of joints: 6 for manipulator + eventual 2 for gripper
    N = length( q );
    
    % Set joint configuration of manipulator
    for i=1:N
        config(i).JointPosition = q(i);
    end
    
end
