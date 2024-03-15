% Function for computing pieces of trajectory from initial configuration
% through every given viapoint
% Parameters:
% - qi        : initial joint configuration
% - viapoints : viapoints through which trajectory will pass - could be
%               expressed in joint space - [6xN] - or task space - [4x4xN]
% times       : time intervals for each piece of trajectory [N+1] 
function [time, positions, velocities] = multipoint_trajectory( qi, viapoints, times )
    parameters(1)

    % Number of viapoints - pieces of trajectory to generate
    N = length( times ) - 1;
    
    % Determine viapoints space
    if size( viapoints, 1 ) > N
        % task space - [4x4xN] Hom-Transf matrices (end-effector poses)
        vp_space = "task";
    else
        % joint space - [6xN] joint configurations
        vp_space = "joint";
    end
    
    % Complete trajectory vectors
    time       = [];
    positions  = [];
    velocities = [];
    
    % Initial time
    ti = times(1)
    
    % TODO: use multithreading to compute every piece of trajectory at the same time
    for i=1:N
        % Set final time to be previous finish time + new time interval for
        % new piece of trajectory
        tf = ti + times(i+1)

        % Extract first viapoint - desired pose/configuration
        if     vp_space == "joint"
            % Desired configuration is already in joint space -> nothing to do
            qf = viapoints(i,:)
            
            if space == "taks"
                % Transform desired configuration from joint to task space
                qf = direct_kinematics_cpp( qf, AL, A, D, TH )

            end

        elseif vp_space == "task"
            Tf = viapoints(4*(i-1)+1:4*i,:);

            if     space == "joint"
                % Transform desired pose from task space to joint space
                if     manipulator == "UR5"
                    Hf = UR5_inverse_kinematics_cpp( Tf(1:3,4), Tf(1:3,1:3), AL, A, D );
                elseif manipulator == "ABB"
                    Hf = ABB_inverse_kinematics_cpp( Tf(1:3,4), Tf(1:3,1:3), AL, A, D );
                elseif manipulator == "custom"
                    Hf  = Custom_manipulator_inverse_kinematics_cpp( Tf(1:3,4), Tf(1:3,1:3), AL, A, D );
                end
                qf = get_closer( Hf, qi )

            elseif space == "task"
                % Desired pose is already in task space -> nothing to do
                qf = Tf

            end
        
        end

        % Generate trajectory from current joint configuration to desired pose
        fprintf("[multipoint_trajectory] generating piece of trajectory\n")
        % TODO: fix cubic & quintic not working for ti != 0
        [t, p, v] = generate_trajectory( qi, qf, 0, times(i+1) );
        time       = [time,    t+ti];
        positions  = [positions;  p];
        velocities = [velocities; v];

        % Set final (of just computed piece of trajectory) = initial (for next piece of trajectory) configuration/pose
        if space == "joint" || kinematics == "IDK"
            % In this case, trajectory is given in joint space -> save last configuration
            qf = p(end,:);

        else
            % Otherwise, last configuration is in task space (Homogeneous Transformation Matrix)
            % so it is necessary to convert it to joint space (through IK, different for each manipulator)
            if     manipulator == "UR5"
                Hf  = UR5_inverse_kinematics_cpp( Tf(1:3,4), Tf(1:3,1:3), AL, A, D );
            elseif manipulator == "ABB"
                Hf  = ABB_inverse_kinematics_cpp( Tf(1:3,4), Tf(1:3,1:3), AL, A, D );
            elseif manipulator == "custom"
                Hf  = Custom_manipulator_inverse_kinematics_cpp( Tf(1:3,4), Tf(1:3,1:3), AL, A, D );
            end

            qf = get_closer( Hf, qi );

        end
        
        % Set initial configuration as last of previous trajectory
        qi = qf;
        ti = t(end)

    end

end

