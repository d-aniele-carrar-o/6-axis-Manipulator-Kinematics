function [scenePath, objectPath] = find_pointcloud_files(timestamp)
    % Search in data folder
    parameters(1);
    scene_files  = dir(fullfile(data_folder, '**', '*table*.ply'));
    object_files = dir(fullfile(data_folder, '**', '*object*.ply'));
    
    scenePath = '';
    objectPath = '';
    
    fprintf('Found %d table files, %d object files\n', length(scene_files), length(object_files));
    
    % Find files with matching timestamp
    for i = 1:length(scene_files)
        if contains(scene_files(i).name, timestamp) || contains(scene_files(i).folder, timestamp)
            scenePath = fullfile(scene_files(i).folder, scene_files(i).name);
            break;
        end
    end
    
    for i = 1:length(object_files)
        if contains(object_files(i).name, timestamp) || contains(object_files(i).folder, timestamp)
            objectPath = fullfile(object_files(i).folder, object_files(i).name);
            break;
        end
    end
end
