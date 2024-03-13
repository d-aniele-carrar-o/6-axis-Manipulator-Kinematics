function [time, positions, velocities] = generate_trajectory( q0, Tf, ti, tf )
    % TO ADD: INITIAL/FINAL VELOCITY IF DIFFERENT FROM ZERO 
    % (like for ex when adding viapoint in the traj in which the ee do NOT have to come to a rest)
    
    params

    if     traj_type == "cubic"
        [time, positions, velocities] = cubic_trajectory( ti, tf, q0, Tf );
    
    elseif traj_type == "quintic"
        [time, positions, velocities] = quintic_trajectory( ti, tf, q0, Tf );
    
    elseif traj_type == "LSPB"
        blend_perc = 0.2;
        [time, positions, velocities] = LSPB_trajectory( ti, tf, blend_perc, q0, Tf );
    
    else
        fprintf("Trajectory type not yet supported.\n")
        return

    end
    
end

