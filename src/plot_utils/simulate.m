% Function for simulating manipulator trajectory
% Parameters:
% - time       : time steps for each position/velocity point - [1xN]
% - positions  : joint configurations or end-effector poses, for each time step
%                depending on space or kinematics - [6xN] or [4x4xN]
% - velocities : joint space or task space velocities for each time step - [6xN]
% - qi         : initial joint configuration
% - axs        : if passed, existing axes object on which to plot trajectory
% - existing_axes_given : true if exising axes is given; if false, create new axes object
function [qf, handlesR] = simulate( time, positions, gripper_positions, velocities, qi, axs, existing_axes_given )
    parameters(0)
    
    disp("[simulate] Press enter to start simulation")
    if real_robot
        
        % TODO: call plot_joint_trajectory_pos_and_vel if flag "plot_graphs" is set to true
        
        % If needed, transform trajectory's positions into joint space
        if space == "task" && kinematics == "IK"
            positions = task2joint_space( qi, positions );
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

        % Run the simulation
        
        % Setup simulation environment
        config = set_robot_configuration( p(1,:), config );
        
        if ~existing_axes_given
            axs = show( robot, config, "Visuals", "on", "Frames", "off", "FastUpdate", true, "PreservePlot", false ); hold on;
        else
            show( robot, config, "Visuals", "on", "Frames", "off", "FastUpdate", true, "PreservePlot", false, "Parent", axs ); hold on;
        end
        
        % Plot manipulator and scatter the positions of the end-effector to highlight trajectory 3D in space
        Te   = direct_kinematics_cpp( p(1,1:6), AL, A, D, TH );
        tmp  = (Trf_0 * Te(1:4,4))';
        p_ee = tmp(1:3);
        scatter3( p_ee(1), p_ee(2), p_ee(3), 10, 'r.', 'Parent', axs ); hold on;
        pause()
        
        disp("[simulate] Simulation started.")
        for i=2:N
            config = set_robot_configuration( p(i,:), config );
            show( robot, config, "Visuals", "on", "Frames", "off", "FastUpdate", true, "PreservePlot", false, "Parent", axs ); hold on;
            Te   = direct_kinematics_cpp( p(i,1:6), AL, A, D, TH );
            tmp  = (Trf_0 * Te(1:4,4))';
            p_ee = tmp(1:3);
            scatter3( p_ee(1), p_ee(2), p_ee(3), 10, 'r.', 'Parent', axs ); hold on;
            waitfor( rate );
        end

        qf       = positions(end,:);
        handlesR = axs;
    
    else
        % Use matlab's plot3 function to simulate trajectory
        [qf, handlesR] = plot_trajectory( time, positions, velocities, qi, axs, existing_axes_given );
        
    end
    
    disp("[simulate] Simulation ended.")

end
