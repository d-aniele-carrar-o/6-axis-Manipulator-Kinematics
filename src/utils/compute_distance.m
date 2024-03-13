function [max_pos_dist, max_orient_dist, max_pos_dist_last, max_orient_dist_last] = compute_distance( Te, last_Te, Td )
    % Position error: w.r.t. final and last positions
    max_pos_dist      = max( abs( Te(1:3,4) - Td(1:3,4) ) );
    max_pos_dist_last = max( abs( Te(1:3,4) - last_Te(1:3,4) ) );
    
    % Orientation error: w.r.t. final and last orientations
    [ang_e, ~]      = rotm2angle_axis( Te(1:3,1:3) );
    [ang_e_last, ~] = rotm2angle_axis( last_Te(1:3,1:3) );
    [ang_d, ~]      = rotm2angle_axis( Td(1:3,1:3) );
    
    max_orient_dist      = abs( ang_e - ang_d );
    max_orient_dist_last = abs( ang_e - ang_e_last );
end
