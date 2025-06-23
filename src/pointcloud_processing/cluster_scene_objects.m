function [objectPointClouds] = cluster_scene_objects(scenePointCloud)
% CLUSTER_SCENE_OBJECTS Extracts individual objects from a scene point cloud
%   This function uses clustering to separate individual objects in a scene
%
%   Input:
%       scenePointCloud - Point cloud of the scene containing multiple objects
%
%   Output:
%       objectPointClouds - Cell array of point clouds, each representing a single object

    % Parameters for clustering
    minDistance = 0.02; % 2cm minimum distance between clusters
    minPoints = 100;    % Minimum points to consider as an object
    
    % Perform Euclidean clustering
    [labels, numClusters] = pcsegdist(scenePointCloud, minDistance, 'NumClusterPoints', minPoints);
    
    % Initialize cell array to store individual object point clouds
    objectPointClouds = cell(1, numClusters);
    
    % Extract each cluster as a separate point cloud
    for i = 1:numClusters
        % Extract points belonging to this cluster
        indices = find(labels == i);
        
        % Create a new point cloud for this object
        objectPointClouds{i} = select(scenePointCloud, indices);
        
        % Display information about the extracted object
        fprintf('Extracted object %d with %d points\n', i, numel(indices));
    end
    
    % If no objects were found, warn the user
    if numClusters == 0
        warning('No objects detected in the scene.');
    end
end