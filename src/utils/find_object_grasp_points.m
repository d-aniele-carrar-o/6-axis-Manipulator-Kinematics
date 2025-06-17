function [grasp_points, grasp_orientations] = find_object_grasp_points(ptCloud, Te_w_e_left, Te_w_e_right)
    % FIND_OBJECT_GRASP_POINTS Finds optimal grasp points on a parallelepiped object
    %
    % This function analyzes a point cloud of a parallelepiped object and finds
    % the optimal grasp points on the side faces in the XY plane, with orientations
    % that are as similar as possible to the initial robot poses.
    %
    % Inputs:
    %   ptCloud - A pointCloud object containing the object's points in world frame
    %   Te_w_e_left - (Optional) Initial left robot end-effector pose in world frame
    %   Te_w_e_right - (Optional) Initial right robot end-effector pose in world frame
    %
    % Outputs:
    %   grasp_points - [2×3] matrix with the coordinates of the two grasp points
    %   grasp_orientations - [2×3×3] matrix with the orientation matrices for each grasp point
    %                        where the z-axis points inward toward the object center
    
    % Handle optional arguments
    use_robot_orientations = false;
    if nargin >= 3
        use_robot_orientations = true;
        % Extract initial robot orientations
        R_left = Te_w_e_left(1:3,1:3);
        R_right = Te_w_e_right(1:3,1:3);
        
        % Get the x and y axes of each robot
        x_left = R_left(:,1);
        y_left = R_left(:,2);
        x_right = R_right(:,1);
        y_right = R_right(:,2);
        
        fprintf('Using initial robot orientations for grasp planning\n');
    end
    
    % Extract point cloud data
    points = ptCloud.Location;
    
    % Find the object's centroid
    centroid = mean(points, 1);
    
    % Perform PCA to find the principal axes of the object
    centered_points = points - centroid;
    [coeff, ~, latent] = pca(centered_points);
    
    % Sort the axes by the variance (eigenvalues)
    [~, idx] = sort(latent, 'descend');
    coeff = coeff(:,idx);
    
    % Create a coordinate system for the object
    % TODO: change this to take (at least) the Z direction to be the one most similar to the world's Z axes
    % basing only on object's dimensions can be dangerous
    x_axis = coeff(:,1);  % Longest dimension
    y_axis = coeff(:,2);  % Second longest dimension
    
    % Project points onto each axis to find the extents
    x_proj = centered_points * x_axis;
    y_proj = centered_points * y_axis;
    
    % Find the min and max along each axis
    x_min = min(x_proj); x_max = max(x_proj);
    y_min = min(y_proj); y_max = max(y_proj);
    
    % Define world Y axis
    world_y = [0; 1; 0];
    
    % Calculate alignment of object axes with world Y axis
    x_alignment = abs(dot(x_axis, world_y));
    y_alignment = abs(dot(y_axis, world_y));
    
    grasp_points = zeros(3, 2);
    grasp_orientations = zeros(3, 3, 2);

    % Choose the axis that is most aligned with world Y axis
    if x_alignment >= y_alignment
        % Grasp along the X-axis (most aligned with world Y)
        grasp_points(:,1) = centroid + (x_min * x_axis');
        grasp_points(:,2) = centroid + (x_max * x_axis');
        
        % Adjust Z coordinate to be at the middle height of the object
        grasp_points(3,1) = centroid(3);
        grasp_points(3,2) = centroid(3);
        
        % Orientation matrices for each grasp point (z-axis points inward)
        grasp_orientations(:,3,1) = x_axis';  % Z-axis points inward (along X)
        grasp_orientations(:,3,2) = -x_axis'; % Z-axis points inward (along -X)
        
        fprintf('Grasping along object X-axis (alignment with world Y: %.3f)\n', x_alignment);
    else
        % Grasp along the Y-axis (most aligned with world Y)
        grasp_points(:,1) = centroid + (y_min * y_axis');
        grasp_points(:,2) = centroid + (y_max * y_axis');
        
        % Adjust Z coordinate to be at the middle height of the object
        grasp_points(3,1) = centroid(3);
        grasp_points(3,2) = centroid(3);
        
        % Orientation matrices for each grasp point (z-axis points inward)
        grasp_orientations(:,3,1) = y_axis';  % Z-axis points inward (along Y)
        grasp_orientations(:,3,2) = -y_axis'; % Z-axis points inward (along -Y)
        
        fprintf('Grasping along object Y-axis (alignment with world Y: %.3f)\n', y_alignment);
    end
    
    % Complete the orientation matrices with orthogonal x and y axes
    for i = 1:2
        z = squeeze(grasp_orientations(:,3,i));
        
        if use_robot_orientations
            % Use the robot's initial orientation as reference
            if i == 1 % Left robot
                % Project the robot's x-axis onto the plane perpendicular to z
                x_proj = x_left - dot(x_left, z) * z;
                
                % If the projection is too small, use the robot's y-axis
                if norm(x_proj) < 0.1
                    x_proj = y_left - dot(y_left, z) * z;
                end
                
                % If still too small, use world Z as reference
                if norm(x_proj) < 0.1
                    world_z = [0; 0; 1];
                    % Ensure z is a column vector
                    if size(z, 1) == 1
                        z = z';
                    end
                    x_proj = cross(z, world_z);
                    
                    % If z is nearly parallel to world_z, use world X
                    if norm(x_proj) < 0.1
                        x_proj = cross(z, [1; 0; 0]);
                    end
                end
            else % Right robot
                % Project the robot's x-axis onto the plane perpendicular to z
                x_proj = x_right - dot(x_right, z) * z;
                
                % If the projection is too small, use the robot's y-axis
                if norm(x_proj) < 0.1
                    x_proj = y_right - dot(y_right, z) * z;
                end
                
                % If still too small, use world Z as reference
                if norm(x_proj) < 0.1
                    world_z = [0; 0; 1];
                    % Ensure z is a column vector
                    if size(z, 1) == 1
                        z = z';
                    end
                    x_proj = cross(z, world_z);
                    
                    % If z is nearly parallel to world_z, use world X
                    if norm(x_proj) < 0.1
                        x_proj = cross(z, [1; 0; 0]);
                    end
                end
            end
            
            % Normalize to get the x-axis
            x = x_proj / norm(x_proj);

        else
            % Use world Z as reference for x-axis calculation (original method)
            world_z = [0; 0; 1];
            
            % Ensure z is a column vector
            if size(z, 1) == 1
                z = z';
            end
            
            % Create x-axis orthogonal to z and aligned with world_z as much as possible
            x = cross(z, world_z);
            if norm(x) < 0.1  % If z is nearly parallel to world_z
                x = cross(z, [1; 0; 0]);
            end
            x = x / norm(x);
        end
        
        % Create y-axis to complete right-handed system
        % Make sure x is a vector, not a matrix
        if numel(x) > 3
            x = x(:,1);  % Take just the first column if it's a matrix
        end
        
        % Ensure x and z are column vectors with the same dimensions
        if size(x,1) ~= size(z,1)
            if size(x,1) == 1
                x = x';
            end
            if size(z,1) == 1
                z = z';
            end
        end
        
        y = cross(z, x);
        y = y / norm(y);

        grasp_orientations(:, 1, i) = x;
        grasp_orientations(:, 2, i) = y;
    end

end
