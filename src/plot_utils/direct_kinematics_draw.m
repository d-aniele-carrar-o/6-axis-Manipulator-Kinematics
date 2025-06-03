% Function for computing DK & plot manipulator configuration in 3D space
% Parameters:
% - q         : manipulator configuration - [6x1]
% - handles   : handles to the different transofmrations. 
%               the first element links to existing axes object
% - firstTime : if true, draw and save manipulator's configuration and axes; 
%               if false, update existing manipulator's configuration
function [Te, handlesR] = direct_kinematics_draw( q, handles, firstTime )
    parameters(1)

    % Compute DK
    Te  = direct_kinematics_cpp( q, AL, A, D, TH );

    if real_robot
        config = set_robot_configuration(q, config);
        if firstTime
            axs = show( robot, config, "Visuals", "on", "Frames", "off", "FastUpdate", true, "PreservePlot", false );
            handlesR(1) = axs;
        else
            show( robot, config, "Visuals", "on", "Frames", "off", "FastUpdate", true, "PreservePlot", false, "Parent", handles(1) );
        end
        return
    end
    
    % Extract axes object for plotting
    if firstTime
        axs = axes();
        xlim([-1,1]); ylim([-1,1]); zlim([-0.8,1.2]);
        view(3); grid on;
    else
        axs = handles(1);
    end

    % Number of joint variables
    N   = max(size( q ));
    
    % If first time draw, else update
    if firstTime
        handlesR(1) = axs;

        % Draw first frame (Frame 0)
        h  = triad( 'Parent', axs, 'linewidth', 3, 'scale', 0.1 );
        ti = hgtransform( 'Parent', h, 'Matrix', eye(4) );

        for i=1:N
            % Compute transformation matrix
            Ti  = transf_i_1_i_cpp( i-1, q(i), AL, A, D, TH );
            
            % Plot link
            Oi  = Ti(1:3,4);
            Opi = [A(i), 0, 0];
            plot3( [Opi(1), 0],     [Opi(2), 0],     [Opi(3), 0],     'Parent', ti, 'linestyle', '-', 'Color', [0.5 0.5 0.5], 'LineWidth', 3 );
            plot3( [Opi(1), Oi(1)], [Opi(2), Oi(2)], [Opi(3), Oi(3)], 'Parent', ti, 'linestyle', '-', 'Color', [0.5 0.5 0.5], 'LineWidth', 3 );
            
            % Plot frame
            ti = hgtransform( 'Parent', h, 'Matrix', Ti );
            h  = triad( 'Parent', ti, 'linewidth', 2, 'scale', 0.05 );
            
            % Save frame transformation
            handlesR(i+1) = ti;
        end

        % Draw end-effector link and frame, if existing
        if gripper
            % Compute transformation matrix
            Ti  = transf_i_1_i_cpp( 6, 0, AL, A, D, TH );
            
            % Plot link
            Oi  = Ti(1:3,4);
            plot3( [0, Oi(1)], [0, Oi(2)], [0, Oi(3)], 'Parent', ti, 'linestyle', '-', 'Color', [0.5 0.5 0.5], 'LineWidth', 3 );
            
            ti = hgtransform( 'Parent', h, 'Matrix', Ti );
            
            % Save frame transformation
            handlesR(8) = ti;
        end
        
         % Plot last frame
        h = triad( 'Parent', ti, 'linewidth', 2, 'scale', 0.1 );

    else
        handlesR(1) = axs;
        
        % Update existing configuration
        for i=1:N
            Ti  = transf_i_1_i_cpp( i-1, q(i), AL, A, D, TH );
            ti  = handles(i+1);
            set( ti, 'Matrix', Ti )
            handlesR(i+1) = ti;
        end
        
        % Draw end-effector link and frame, if existing
        if gripper
            Ti  = transf_i_1_i_cpp( 6, 0, AL, A, D, TH );
            ti  = handles(8);
            set( ti, 'Matrix', Ti );
            handlesR(8) = ti;
        end
        
        drawnow;

    end
    
end
