% Function for computing pieces of trajectory from initial configuration
% through every given viapoint
% Parameters:
% - qi        : initial joint configuration
% - viapoints : viapoints through which trajectory will pass - could be
%               expressed in joint space - [Nx6] - or task space - [Nx4x4]
% - times     : time intervals for each piece of trajectory [N]
function [time, positions, velocities] = multipoint_trajectory( qi, viapoints, times, robot_id )
    % Handle optional arguments
    if nargin < 4
        robot_id = 1;
    end
    parameters(1, robot_id)

    % Number of viapoints - pieces of trajectory to generate
    N = length( times );

    % Determine viapoints space:
    if size( viapoints, 2 ) == 4
        % task space - [Nx4x4] Hom-Transf matrices (end-effector poses)
        vp_space = "task";
    else
        % joint space - [Nx6] joint configurations
        vp_space = "joint";
    end
    
    % Complete trajectory vectors
    time       = [];
    positions  = [];
    velocities = [];
    
    % Initial time
    ti = 0;
    
    % TODO: use multithreading to compute every piece of trajectory at the same time
    % Loop over the given viapoints
    for i=1:N
        % Extract first viapoint - desired pose/configuration
        if     vp_space == "joint"
            % Desired configuration is already in joint space -> nothing to do
            qf = viapoints(:,i);
            
            if space == "taks"
                % Transform desired configuration from joint to task space
                [~, qf] = direct_kinematics( qf, robot_id );

            end

        elseif vp_space == "task"
            Tf = squeeze(viapoints(i,:,:));

            if     space == "joint"
                % Transform desired pose from task space to joint space
                if     manipulator == "UR5" || manipulator == "UR3e"
                    Hf = UR5_inverse_kinematics_cpp( Tf(1:3,4), Tf(1:3,1:3), AL, A, D );
                elseif manipulator == "ABB"
                    Hf = ABB_inverse_kinematics_cpp( Tf(1:3,4), Tf(1:3,1:3), AL, A, D );
                elseif manipulator == "custom"
                    Hf = Custom_manipulator_inverse_kinematics_cpp( Tf(1:3,4), Tf(1:3,1:3), AL, A, D, TH );
                end
                qf = get_closer( Hf, qi );

            elseif space == "task"
                % Desired pose is already in task space -> nothing to do
                qf = Tf;

            end
        
        end

        % Generate trajectory from current joint configuration to desired pose
        if verbose
            fprintf("[multipoint_trajectory] generating piece of trajectory for robot %d\n", robot_id)
        end
        [t, p, v] = generate_trajectory( qi, qf, ti, ti+times(i)-dt, robot_id );
        
        time       = [time,    t+ti];
        positions  = [positions;  p];
        velocities = [velocities; v];

        % Set final of just computed piece of trajectory equal to initial configuration/pose for next piece of trajectory
        if space == "joint" || kinematics == "IDK"
            % In this case, trajectory is given in joint space -> save last configuration
            qf = p(end,:);

        else
            % Otherwise, last configuration is in task space (Homogeneous Transformation Matrix)
            % so it is necessary to convert it to joint space (through IK, different for each manipulator)
            if     manipulator == "UR5" || manipulator == "UR3e"
                Hf  = UR5_inverse_kinematics_cpp( Tf(1:3,4), Tf(1:3,1:3), AL, A, D );
            elseif manipulator == "ABB"
                Hf  = ABB_inverse_kinematics_cpp( Tf(1:3,4), Tf(1:3,1:3), AL, A, D );
            elseif manipulator == "custom"
                Hf  = Custom_manipulator_inverse_kinematics_cpp( Tf(1:3,4), Tf(1:3,1:3), AL, A, D, TH );
            end
            
            qf = get_closer( Hf, qi );

        end
        
        % Set initial configuration as last of previous trajectory
        qi = qf;
        ti = ti + t(end);

    end

end

