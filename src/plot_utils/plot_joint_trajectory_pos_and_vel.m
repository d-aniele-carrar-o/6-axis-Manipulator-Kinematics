function plot_joint_trajectory_pos_and_vel( time, positions, velocities )
    params
    
    Np = max(size( positions ));
    Nv = max(size( velocities ));

    if Np == Nv
        % Joint space plot ------------------------------------------------

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
    
    else
        % Joint space plot ------------------------------------------------

        % Plot task space trajectory positions and velocities

        pos = [];
        
        % TODO: add angle to plot (transform rotM to angle-axis)
        for i=1:Np/4
            Td       = positions(4*(i-1)+1:4*i,:);
            [ang, ~] = rotm2angle_axis( Td(1:3,1:3) );
            pos      = [pos; [Td(1:3,4)', ang]];
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

