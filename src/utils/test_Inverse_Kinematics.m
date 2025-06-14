function test_Inverse_Kinematics( q0, robot, config )
    parameters(1)

    % Plot the configuration of the manipulator
    axs = axes(); view(3); grid on;
    [~, Te, axs] = direct_kinematics_draw( robot, config, q0, axs, true );
    
    % Compute Inverse Kinematics for just computed pose of selected manipulator to check that each of
    % the eight resulting joint configurations results in the same pose and plot them
    if     manipulator == "UR5"
        H  = UR5_inverse_kinematics_cpp( Te(1:3,4), Te(1:3,1:3), AL, A, D );
    elseif manipulator == "UR3e"
        H  = UR5_inverse_kinematics_cpp( Te(1:3,4), Te(1:3,1:3), AL, A, D );
    elseif manipulator == "ABB"
        H  = ABB_inverse_kinematics_cpp( Te(1:3,4), Te(1:3,1:3), AL, A, D );
    elseif manipulator == "custom"
        H  = Custom_manipulator_inverse_kinematics_cpp( Te(1:3,4), Te(1:3,1:3), AL, A, D, TH );
    end

    pause()

    overlap_views = false;

    for i=1:8
        q_curr = H(i,:)
        [T, ~] = direct_kinematics_draw( robot, config, q_curr, axs, true )
        
        fprintf( "Press enter to continue\n" )
        pause()
    end

end
