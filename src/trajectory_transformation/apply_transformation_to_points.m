function transformed_points = apply_transformation_to_points(points, centroid, transformation)
% Apply transformation to points relative to centroid

    % Step 1: Subtract centroid to center points at origin
    centered_points = points - centroid;
    
    % Step 2: Apply scaling (all dimensions)
    if transformation.scale_factor ~= 1
        centered_points = centered_points * transformation.scale_factor;
    end
    
    % Step 3: Apply rotation around Z axis
    if transformation.rotation_angle ~= 0
        angle_rad = deg2rad(transformation.rotation_angle);
        R = [cos(angle_rad), -sin(angle_rad), 0;
             sin(angle_rad),  cos(angle_rad), 0;
             0,               0,              1];
        centered_points = (R * centered_points')'; 
    end
    
    % Step 4: Add centroid back and apply translation
    transformed_points = centered_points + centroid + transformation.translation';
    
    return;
end
