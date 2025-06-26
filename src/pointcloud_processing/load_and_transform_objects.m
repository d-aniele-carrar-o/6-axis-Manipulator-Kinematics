function object_data = load_and_transform_objects(object_paths, transformations)
% Load and transform objects based on given transformations
    parameters(1); % Get data folder path
    
    % Initialize data structures
    num_objects = length(object_paths);
    object_data = struct();
    object_data.original_pcs = cell(num_objects, 1);
    object_data.original_centroids = zeros(num_objects, 3);
    object_data.transformed_pcs = cell(num_objects, 1);
    object_data.transformed_centroids = zeros(num_objects, 3);
    object_data.paths = object_paths;
    
    % Load original point clouds and compute centroids
    for i = 1:num_objects
        % Load point cloud
        pc_path = fullfile(data_folder, object_paths{i});
        ptCloud = pcread(pc_path);
        object_data.original_pcs{i} = ptCloud;
        
        % Compute centroid
        object_data.original_centroids(i,:) = mean(ptCloud.Location, 1);
    end
    
    % Apply transformations to objects
    for i = 1:num_objects
        if i <= length(transformations)
            T = transformations(i);
            original_pc = object_data.original_pcs{i}.Location;
            centroid = object_data.original_centroids(i,:);
            
            % Create transformed point cloud by applying transformations
            transformed_pc = apply_transformation_to_points(original_pc, centroid, T);
            
            % Store transformed point cloud and its centroid
            transformed_pc_obj = pointCloud(transformed_pc);
            object_data.transformed_pcs{i} = transformed_pc_obj;
            object_data.transformed_centroids(i,:) = mean(transformed_pc, 1);
        else
            % No transformation for this object
            object_data.transformed_pcs{i} = object_data.original_pcs{i};
            object_data.transformed_centroids(i,:) = object_data.original_centroids(i,:);
        end
    end
end
