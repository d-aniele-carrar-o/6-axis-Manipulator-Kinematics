% Function for plotting given trajectory with 3d plot of manipulator
% Parameters:
% - time        : time steps at which the trajectory has been computed - [1xN]
% - positions   : positions of the trajectory corresponding to the time steps
%                 they could be expresses in task space [4x4xN] end-effector poses
%                 or joint space configurations [6xN]
% - velocities  : velocities of the trajectory corresponding to the time steps
%                 they could be expressed in task space [6xN] corresponding to [vx,vy,vz,wx,wy,wz] 
%                 or joint space [6xN] corresponding to [dq1, dq2, dq3, dq4, dq5, dq6]
% - qi          : initial joint configuration - [6x1]
% - axs         : axes() object, if given, plot on this axes
% - existing_axes_given : set true if axes is given, if false, function creates new axes for plotting
function [qf, axs] = plot_trajectory( positions, axs, existing_axes_given )
    parameters(1)

    % Number of generated viapoints
    N = max( size( positions ) );
    
    % Simulate trajectory
    if ~existing_axes_given
        figure
        axs = axes(); view(3); grid on;
    end

    % Draw manipulator simulation and end-effector trajectory
    [Te, handlesR] = direct_kinematics_draw( positions(1,:), axs, true );
    temp           = (Trf_0*Te(1:4,4))';
    pos            = temp(1:3);
    scatter3( pos(1), pos(2), pos(3), 10, 'r.',  "Parent", handlesR(1) );
    pause()
    disp("[simulate] Simulation started.")

    for i=1:N
        [Te, handlesR] = direct_kinematics_draw( positions(i,:), handlesR, false ); hold on;
        temp           = (Trf_0*Te(1:4,4))';
        pos            = temp(1:3);
        scatter3( pos(1), pos(2), pos(3), 10, 'r.',  "Parent", handlesR(1) ); hold on;
        waitfor( rate );
        
    end
    hold off;
    
    % Return last joint configuration - useful if after this trajectory there have to start with another one
    qf = positions(end,:);

end

