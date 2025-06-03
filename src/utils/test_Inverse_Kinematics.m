function test_Inverse_Kinematics( q0 )
    parameters(0)

    % Plot the configuration of the manipulator
    axs = axes(); view(3); grid on;
    [Te, axs] = direct_kinematics_draw( q0, axs, true );
    
    % Compute Inverse Kinematics for just computed pose of selected manipulator to check that each of
    % the eight resulting joint configurations results in the same pose and plot them
    if     manipulator == "UR5"
        H  = UR5_inverse_kinematics_cpp( Te(1:3,4), Te(1:3,1:3), AL, A, D );
    elseif manipulator == "UR3e"
        config = set_robot_configuration(q0, config);
        ik = inverseKinematics("RigidBodyTree", robot);
        [H, ~] = ik("tool0", Te, ones(6,1), config);
        H.JointPosition
    elseif manipulator == "ABB"
        H  = ABB_inverse_kinematics_cpp( Te(1:3,4), Te(1:3,1:3), AL, A, D );
    elseif manipulator == "custom"
        H  = Custom_manipulator_inverse_kinematics_cpp( Te(1:3,4), Te(1:3,1:3), AL, A, D, TH );
    end

    if real_robot
        return
    end
    
    pause()
    
    overlap_views = false;
    for i=1:8
        if overlap_views
            [T, ~] = direct_kinematics_draw( H(i,:), axs, true )
        else
            [T, ~] = direct_kinematics_draw( H(i,:), axs, false )
        end
        
        fprintf( "Press enter to continue\n" )
        pause()
    end

end

