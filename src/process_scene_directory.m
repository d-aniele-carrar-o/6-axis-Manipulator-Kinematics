function [allGraspPoints, allGraspOrientations] = process_scene_directory(dirPath, Te_w_e_left, Te_w_e_right, tableParams)
% PROCESS_SCENE_DIRECTORY Process all point cloud files in a directory
%   This function processes all point cloud files in a directory, clusters objects
%   in each scene, and computes grasp points for each object
%
%   Input:
%       dirPath - Path to directory containing scene point cloud files
%       Te_w_e_left - Left robot end-effector pose
%       Te_w_e_right - Right robot end-effector pose
%       tableParams - Table parameters for calibration
%
%   Output:
%       allGraspPoints - Cell array of grasp points for each scene
%       allGraspOrientations - Cell array of grasp orientations for each scene

    % Get all point cloud files in the directory
    files = dir(fullfile(dirPath, '*.ply'));
    
    % Check if any files were found
    if isempty(files)
        fprintf('\n===== WARNING: NO FILES FOUND =====\n');
        fprintf('No .ply files found in directory: %s\n', dirPath);
        allGraspPoints = {};
        allGraspOrientations = {};
        return;
    end
    
    % Initialize output cell arrays
    numScenes = length(files);
    allGraspPoints = cell(1, numScenes);
    allGraspOrientations = cell(1, numScenes);
    
    fprintf('\n===== PROCESSING %d SCENES =====\n', numScenes);
    
    % Process each scene
    for i = 1:numScenes
        % Get full path to the scene file
        scenePath = fullfile(dirPath, files(i).name);
        fprintf('\n----- Processing Scene %d/%d: %s -----\n', i, numScenes, files(i).name);
        
        try
            % For simplicity, we'll use a calibration scene for all files
            % Get the calibration scene path
            project_root = fileparts(fileparts(mfilename('fullpath')));
            calibScenePath = fullfile(project_root, 'output', 'segmented_objects', '25-06-07-11-21-29', 'table_surface.ply');
            
            % Calibrate camera and transform scene point cloud
            fprintf('Calibrating scene...\n');
            [~, scenePC_world, ~] = calibrate_camera(calibScenePath, scenePath, tableParams, false);
            
            % Cluster objects in the scene
            fprintf('Clustering objects in scene...\n');
            objectPCs = cluster_scene_objects(scenePC_world);
            
            % Check if any objects were found
            if isempty(objectPCs)
                fprintf('No objects found in scene %d\n', i);
                continue;
            end
            
            % Initialize grasp points and orientations for this scene
            sceneGraspPoints = cell(1, length(objectPCs));
            sceneGraspOrientations = cell(1, length(objectPCs));
            
            % Process each object in the scene
            for j = 1:length(objectPCs)
                fprintf('Finding grasp points for object %d/%d...\n', j, length(objectPCs));
                % Find grasp points for this object
                [grasp_points, grasp_orientations] = find_object_grasp_points(objectPCs{j}, Te_w_e_left, Te_w_e_right);
                
                % Store results
                sceneGraspPoints{j} = grasp_points;
                sceneGraspOrientations{j} = grasp_orientations;
            end
            
            % Store results for this scene
            allGraspPoints{i} = sceneGraspPoints;
            allGraspOrientations{i} = sceneGraspOrientations;
            
        catch e
            fprintf('Error processing scene %d: %s\n', i, e.message);
        end
    end
    
    % Remove empty cells
    allGraspPoints = allGraspPoints(~cellfun('isempty', allGraspPoints));
    allGraspOrientations = allGraspOrientations(~cellfun('isempty', allGraspOrientations));
    
    fprintf('\n===== COMPLETED SCENE PROCESSING =====\n');
    fprintf('Successfully processed %d scenes\n', length(allGraspPoints));
end
