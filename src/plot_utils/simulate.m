% Function for simulating manipulator trajectory
% Parameters:
% - time       : time steps for each position/velocity point - [1xN]
% - positions  : joint configurations or end-effector poses, for each time step
%                depending on space or kinematics - [6xN] or [4x4xN]
% - velocities : joint space or task space velocities for each time step - [6xN]
% - qi         : initial joint configuration
% - axs        : if passed, existing axes object on which to plot trajectory
% - existing_axes_given : true if exising axes is given; if false, create new axes object
function [qf, handlesR] = simulate( robot, config, time, positions, gripper_positions, velocities, qi, axs, existing_axes_given )
    parameters(1)
    
    % Transform, if needed, in order to plot position and velocity profile graphs
    if space == "joint" || kinematics == "IDK"
        % Compute end-effector positions in order to plot its trajectory together with the manipulator
        [task_poses, task_vels] = joint2task_space( positions, velocities );
        
        if plot_grahps
            % Plot position and velocity joint space trajectories
            plot_joint_trajectory_pos_and_vel( time, positions, velocities, "joint" );
            hold off;

            % Plot position and velocity task space trajectories
            plot_joint_trajectory_pos_and_vel( time, task_poses, task_vels, "task" );
            hold off;
        end

    elseif space == "task"
        % Transform the end-effector poses using IK of selected manipulator to joint space configurations
        [qs, q_dots] = task2joint_space( qi, positions, velocities );

        if plot_grahps
            % Plot position and velocity joint space trajectories
            plot_joint_trajectory_pos_and_vel( time, qs, q_dots, "joint" );
            hold off;

            % Plot position and velocity task space trajectories
            plot_joint_trajectory_pos_and_vel( time, positions, velocities, "task" );
            hold off;
        end

    end

    disp("[simulate] Press enter to start simulation")
    if real_robot
        % If needed, transform trajectory's positions into joint space
        if space == "task" && kinematics == "IK"
            positions = qs;
        end
        
        N = size( positions, 1 );
        
        % If gripper is present, add eventual gripper trajectory positions
        if gripper
            % If positions for gripper are not given, fill with zeros
            if isempty( gripper_positions )
                gripper_positions = zeros( N, 2 );
            end
            
            % Concatenate manipulator and gripper positions (8 joints total)
            p = [positions, gripper_positions];

        else
            p = positions;

        end

        % Run the simulation ======================================================================
        
        % Setup simulation environment
        config = set_robot_configuration( p(1,:), config );
        
        if ~existing_axes_given
            axs = show( robot, config, "Visuals", "on", "Frames", "on", "FastUpdate", true, "PreservePlot", false ); hold on;
        else
            show( robot, config, "Visuals", "on", "Frames", "on", "FastUpdate", true, "PreservePlot", false, "Parent", axs ); hold on;
        end

        xlim([-1,1]); ylim([-1,1]); zlim([0,1.2]);
        view(3); grid on;
        
        % Pre-allocate trajectory points storage
        traj_points = [];
        
        % Plot manipulator and initialize scatter plot for trajectory
        Te   = direct_kinematics_cpp( p(1,1:6), AL, A, D, TH );
        tmp  = (Trf_0 * Te(1:4,4))';
        p_ee = tmp(1:3);
        traj_points = p_ee;
        h_scatter = scatter3( p_ee(1), p_ee(2), p_ee(3), 10, 'r.', 'Parent', axs ); hold on;
        
        k = waitforbuttonpress;
        while k ~= 1
            k = waitforbuttonpress;
        end
        
        disp("[simulate] Simulation started.")
        for i=2:N
            config = set_robot_configuration( p(i,:), config );
            show( robot, config, "Visuals", "on", "Frames", "on", "FastUpdate", true, "PreservePlot", false, "Parent", axs ); hold on;
            Te   = direct_kinematics_cpp( p(i,1:6), AL, A, D, TH );
            tmp  = (Trf_0 * Te(1:4,4))';
            p_ee = tmp(1:3);
            if (mod(i, 5) == 0)
                traj_points = [traj_points; p_ee];
                set(h_scatter, 'XData', traj_points(:,1), 'YData', traj_points(:,2), 'ZData', traj_points(:,3));
            end
            waitfor( rate );
        end

        qf       = positions(end,:);
        handlesR = axs;
    
    else
        % Use matlab's plot3 function to simulate trajectory
        if space == "task" && kinematics == "IK"
            [qf, handlesR] = plot_trajectory( qs, axs, existing_axes_given );
        else
            [qf, handlesR] = plot_trajectory( positions, axs, existing_axes_given );
        end
        
    end
    
    disp("[simulate] Simulation ended.")

end
