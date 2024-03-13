% Function for plotting given trajectory with 3d plot of manipulator
% Parameters:
% - time        : time steps at which the trajectory has been computed - 1xN
% - positions   : positions of the trajectory corresponding to the time steps
%                 they could be expresses in task space [4x4xN] end-effector poses
%                 or joint space configurations [6xN]
% - velocities  : velocities of the trajectory corresponding to the time steps
%                 they could be expressed in task space [6xN] corresponding to [vx,vy,vz,wx,wy,wz] 
%                 or joint space [6xN] corresponding to [dq1, dq2, dq3, dq4, dq5, dq6]
% - qi          : initial joint configuration [6x1]
% - space       : space in which the trajectory is expressed: either "task" or "joint"
% - plot_graphs : plot 2D graphs for position and velocity againts time, either true or false
function [qf, handlesR] = plot_trajectory( time, positions, velocities, qi, axs, existing_axes_given )
    params

    % Number of generated viapoints
    N = max( size( positions ) );
    
    if plot_grahps
        % Plot position and velocity joint space trajectories
        plot_joint_trajectory_pos_and_vel( time, positions, velocities );
    end
    
    % Transform, if needed, in order to plot position and velocity profiles
    % TODO: if IDK: put in output also task space pos-vel vectors (from desired ones inside function)
    if plot_grahps && kinematics == "IDK"
        task_poses = [];
        task_vels  = [];

        % Compute end-effector positions in order to plot its trajectory together with the manipulator
        for i=1:N
            Te         = direct_kinematics_cpp( positions(i,:), AL, A, D, TH );
            task_poses = [task_poses; Trf_0*Te];
            J          = Jacobian_cpp( Te, positions(i,:), AL, A, D, TH );
            task_vels  = [task_vels; (J * velocities(i,:)')'];
        end

        % Plot position and velocity joint space trajectories
        plot_joint_trajectory_pos_and_vel( time, task_poses, task_vels );

    elseif space == "task"
        % Transform the end-effector poses using IK of selected manipulator to joint space configurations
        
        % TODO: unify these two functions
        positions  = task2joint_space( qi, positions );
        velocities = get_joint_velocities( positions, velocities );
        N          = max( size( positions ) );

        if plot_grahps
            % Plot position and velocity task space trajectories
            plot_joint_trajectory_pos_and_vel( time, positions, velocities );
        end

    end
    

    % Simulate trajectory

    % Setup plot figure
    if ~existing_axes_given
        figure
        axs = axes();
        view(3); grid on;
    end

    % Draw manipulator simulation and end-effector trajectory
    [Te, handlesR] = direct_kinematics_draw( positions(1,:), axs, true );
    temp           = (Trf_0*Te(1:4,4))';
    pos            = temp(1:3);
    scatter3( pos(1), pos(2), pos(3), 10, 'r.',  "Parent", handlesR(1) );
    for i=2:N
        [Te, handlesR] = direct_kinematics_draw( positions(i,:), handlesR, false );
        temp           = (Trf_0*Te(1:4,4))';
        pos            = temp(1:3);
        scatter3( pos(1), pos(2), pos(3), 10, 'r.',  "Parent", handlesR(1) );
    end
    hold off;
    
    % Return last joint configuration, useful if after this trajectory we
    % have to start with another one (most of the times, this is the case)
    qf = positions(end,:);

end

