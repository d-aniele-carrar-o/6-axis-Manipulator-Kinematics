function [time, positions, velocities] = generate_trajectory( qi, qf, ti, tf, robot_id )
    % TO ADD: INITIAL/FINAL VELOCITY IF DIFFERENT FROM ZERO 
    % (like for ex when adding viapoint in the traj in which the ee do NOT have to come to a rest)
    if nargin < 4
        robot_id = 1;
    end
    parameters(1, robot_id)

    if     traj_type == "cubic"
        [time, positions, velocities] = cubic_trajectory( qi, qf, ti, tf, robot_id );
    
    elseif traj_type == "quintic"
        [time, positions, velocities] = quintic_trajectory( qi, qf, ti, tf, robot_id );
    
    elseif traj_type == "LSPB"
        blend_perc = 0.2;
        [time, positions, velocities] = LSPB_trajectory( ti, tf, blend_perc, qi, qf );
    
    else
        fprintf("Trajectory type not yet supported.\n")
        return

    end
    
end

