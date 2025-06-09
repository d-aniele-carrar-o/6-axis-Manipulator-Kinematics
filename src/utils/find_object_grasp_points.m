function [grasp_points, grasp_orientations] = find_object_grasp_points(ptCloud)
    % FIND_OBJECT_GRASP_POINTS Finds optimal grasp points on a parallelepiped object
    %
    % This function analyzes a point cloud of a parallelepiped object and finds
    % the optimal grasp points on the side faces in the XY plane, regardless of 
    % the object's orientation in the scene.
    %
    % Inputs:
    %   ptCloud - A pointCloud object containing the object's points in world frame
    %
    % Outputs:
    %   grasp_points - [2×3] matrix with the coordinates of the two grasp points
    %   grasp_orientations - [2×3×3] matrix with the orientation matrices for each grasp point
    %                        where the z-axis points inward toward the object center
    
    % Extract point cloud data
    points = ptCloud.Location;
    
    % Find the object's centroid
    centroid = mean(points, 1);
    
    % Perform PCA to find the principal axes of the object
    centered_points = points - centroid;
    [coeff, ~, latent] = pca(centered_points);
    
    % Sort the axes by the variance (eigenvalues)
    [~, idx] = sort(latent, 'descend');
    coeff = coeff(:, idx);
    
    % Create a coordinate system for the object
    x_axis = coeff(:, 1);  % Longest dimension
    y_axis = coeff(:, 2);  % Second longest dimension
    z_axis = coeff(:, 3);  % Shortest dimension
    
    % Ensure we have a right-handed coordinate system
    if dot(cross(x_axis, y_axis), z_axis) < 0
        z_axis = -z_axis;
    end
    
    % Project points onto each axis to find the extents
    x_proj = centered_points * x_axis;
    y_proj = centered_points * y_axis;
    z_proj = centered_points * z_axis;
    
    % Find the min and max along each axis
    x_min = min(x_proj); x_max = max(x_proj);
    y_min = min(y_proj); y_max = max(y_proj);
    z_min = min(z_proj); z_max = max(z_proj);
    
    % Calculate dimensions
    x_length = x_max - x_min;
    y_length = y_max - y_min;
    
    % For side grasping in XY plane, we need to determine which horizontal dimension is smaller
    if x_length <= y_length
        % Grasp along the X-axis (shortest horizontal dimension)
        grasp_points = zeros(2, 3);
        grasp_points(1, :) = centroid + (x_min * x_axis');
        grasp_points(2, :) = centroid + (x_max * x_axis');
        
        % Adjust Z coordinate to be at the middle height of the object
        grasp_points(1, 3) = centroid(3);
        grasp_points(2, 3) = centroid(3);
        
        % Orientation matrices for each grasp point (z-axis points inward)
        grasp_orientations = zeros(2, 3, 3);
        grasp_orientations(1, :, 3) = x_axis;  % Z-axis points inward (along X)
        grasp_orientations(2, :, 3) = -x_axis; % Z-axis points inward (along -X)
    else
        % Grasp along the Y-axis (shortest horizontal dimension)
        grasp_points = zeros(2, 3);
        grasp_points(1, :) = centroid + (y_min * y_axis');
        grasp_points(2, :) = centroid + (y_max * y_axis');
        
        % Adjust Z coordinate to be at the middle height of the object
        grasp_points(1, 3) = centroid(3);
        grasp_points(2, 3) = centroid(3);
        
        % Orientation matrices for each grasp point (z-axis points inward)
        grasp_orientations = zeros(2, 3, 3);
        grasp_orientations(1, :, 3) = y_axis;  % Z-axis points inward (along Y)
        grasp_orientations(2, :, 3) = -y_axis; % Z-axis points inward (along -Y)
    end
    
    % Complete the orientation matrices with orthogonal x and y axes
    for i = 1:2
        z = squeeze(grasp_orientations(i, :, 3));
        
        % Use world Z as reference for x-axis calculation
        world_z = [0, 0, 1]';
        
        % Create x-axis orthogonal to z and aligned with world_z as much as possible
        x = cross(z, world_z);
        if norm(x) < 0.1  % If z is nearly parallel to world_z
            x = cross(z, [1, 0, 0]');
        end
        x = x / norm(x);
        
        % Create y-axis to complete right-handed system
        y = cross(z, x);
        y = y / norm(y);
        
        grasp_orientations(i, :, 1) = x;
        grasp_orientations(i, :, 2) = y;
    end
end