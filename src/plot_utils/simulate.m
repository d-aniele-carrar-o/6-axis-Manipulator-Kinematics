function [qf, handlesR] = simulate( time, positions, gripper_positions, velocities, qi, axs, existing_axes_given )
    params

    if real_robot
        % Use robot's urdf to simulate
        config = robot.homeConfiguration;
        
        if space == "task" && kinematics == "IK"
            positions = task2joint_space( qi, positions );
        end
        
        N = size( positions, 1 );
        
        % If positions for gripper are not given, fill with zeros
        if isempty( gripper_positions )
            gripper_positions = zeros( N, 2 );
        end
        
        % Concatenate manipulator and gripper positions (8 joints total)
        p = [positions, gripper_positions];
        
        % Run the simulation
        disp("Press enter to start simulation")
        
        % Setup simulation environment
        config = set_robot_configuration( config, p(1,:), [pi/6, pi/6] );
        if ~existing_axes_given
            axs = show( robot, config, "Visuals", "on", "Frames", "off", "FastUpdate", true, "PreservePlot", false ); hold on;
        else
            show( robot, config, "Visuals", "on", "Frames", "off", "FastUpdate", true, "PreservePlot", false, "Parent", axs ); hold on;
        end
        Te   = direct_kinematics_cpp( p(1,1:6), AL, A, D, TH );
        tmp  = (Trf_0 * Te(1:4,4))';
        p_ee = tmp(1:3);
        scatter3( p_ee(1), p_ee(2), p_ee(3), 10, 'r.', 'Parent', axs ); hold on;
        pause()

        for i=2:N
            config = set_robot_configuration( config, p(i,:), [pi/6, pi/6] );
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
        % Use matlab's plot3 function to simulate
        [qf, handlesR] = plot_trajectory( time, positions, velocities, qi, axs, existing_axes_given );
    
    end

end
