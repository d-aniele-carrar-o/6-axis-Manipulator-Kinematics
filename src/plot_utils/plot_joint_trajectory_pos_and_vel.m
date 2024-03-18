% Function for plotting trajectory's joint configurations/velocities and/or task space positions/angular velocities
% Parameters:
% - time       : time steps at which position/velocity points are given - [1xN]
% - positions  : joint configurations or end-effector poses - [6xN] or [4x4xN]
% - velocities : joint space or task space velocity points - [6xN]
function plot_joint_trajectory_pos_and_vel( time, positions, velocities, space )
    N = size( positions, 1 );
    
    % Joint space plot ----------------------------------------------------
    if space == "joint"
        % Plot joint space trajectory positions and velocities
        figure
        plot( time, reshape( positions(:,1), [], 1), '.-', time, reshape( positions(:,2), [], 1), '.-', time, reshape( positions(:,3), [], 1), '.-', ...
              time, reshape( positions(:,4), [], 1), '.-', time, reshape( positions(:,5), [], 1), '.-', time, reshape( positions(:,6), [], 1), '.-' )
        axis padded
        title( "Positions - Joint space" )
        legend( 'q1', 'q2', 'q3', 'q4', 'q5', 'q6' )
        
        % Plot joint space trajectory velocities
        figure
        plot( time, reshape( velocities(:,1), [], 1), '.-', time, reshape( velocities(:,2), [], 1), '.-', time, reshape( velocities(:,3), [], 1), '.-', ...
              time, reshape( velocities(:,4), [], 1), '.-', time, reshape( velocities(:,5), [], 1), '.-', time, reshape( velocities(:,6), [], 1), '.-' )
        axis padded
        title( "Velocities - Joint space" )
        legend( 'q1', 'q2', 'q3', 'q4', 'q5', 'q6' )
        hold off;
    
    % Taks space plot -----------------------------------------------------
    elseif space == "task"
        % Plot task space trajectory positions and velocities

        pos = zeros( N/4, 4 );
        
        for i=1:N/4
            Td       = positions(4*(i-1)+1:4*i,:);
            [ang, ~] = rotm2angle_axis( Td(1:3,1:3) );
            pos(i,:) = [Td(1:3,4)', ang];
        end

        figure
        plot( time, pos(:,1), '.-', time, pos(:,2), '.-', time, pos(:,3), '.-', time, pos(:,4), '.-' );
        axis padded
        title( "Positions - Task space" )
        legend( 'x', 'y', 'z', 'angle' )
        
        figure
        plot( time, reshape( velocities(:,1), [], 1), '.-', time, reshape( velocities(:,2), [], 1), '.-', time, reshape( velocities(:,3), [], 1), '.-', ...
              time, reshape( velocities(:,4), [], 1), '.-', time, reshape( velocities(:,5), [], 1), '.-', time, reshape( velocities(:,6), [], 1), '.-' );
        axis padded
        title( "Velocities - Task space" )
        legend( 'dx', 'dy', 'dz', 'omega_x', 'omega_y', 'omega_z' )
        hold off;

    end

end

