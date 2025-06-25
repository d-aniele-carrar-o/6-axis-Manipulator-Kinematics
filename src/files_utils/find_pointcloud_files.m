function [scenePath, objectPath] = find_pointcloud_files(timestamp)
    % FIND_POINTCLOUD_FILES Find scene and object pointcloud files by timestamp
    % If exact match not found, returns the closest timestamp
    %
    % Parameters:
    %   timestamp - Timestamp string in format 'yy-MM-DD-hh-mm-ss'
    %
    % Returns:
    %   scenePath  - Path to scene pointcloud file
    %   objectPath - Path to object pointcloud file
    
    % Search in data folder
    parameters(1);
    scene_files  = dir(fullfile(data_folder, '**', '*table*.ply'));
    object_files = dir(fullfile(data_folder, '**', '*object*.ply'));
    
    scenePath = '';
    objectPath = '';
    
    fprintf('Found %d table files, %d object files\n', length(scene_files), length(object_files));
    
    % Try exact match first
    if ~isempty(timestamp)
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
    
    % If no exact match found, find closest timestamp
    if isempty(scenePath) && ~isempty(timestamp)
        fprintf('No exact match found for timestamp %s, searching for closest...\n', timestamp);
        target_time = parse_timestamp(timestamp);
        
        if ~isnan(target_time)
            % Find closest scene file
            best_scene_diff = inf;
            for i = 1:length(scene_files)
                file_timestamp = extract_timestamp_from_filename(scene_files(i).name);
                if isempty(file_timestamp)
                    % Try to extract from folder name
                    [~, folder_name, ~] = fileparts(scene_files(i).folder);
                    file_timestamp = extract_timestamp_from_filename(folder_name);
                end
                
                if ~isempty(file_timestamp)
                    file_time = parse_timestamp(file_timestamp);
                    if ~isnan(file_time)
                        diff = abs(file_time - target_time);
                        if diff < best_scene_diff
                            best_scene_diff = diff;
                            scenePath = fullfile(scene_files(i).folder, scene_files(i).name);
                        end
                    end
                end
            end
            
            % Find closest object file
            best_obj_diff = inf;
            for i = 1:length(object_files)
                file_timestamp = extract_timestamp_from_filename(object_files(i).name);
                if isempty(file_timestamp)
                    % Try to extract from folder name
                    [~, folder_name, ~] = fileparts(object_files(i).folder);
                    file_timestamp = extract_timestamp_from_filename(folder_name);
                end
                
                if ~isempty(file_timestamp)
                    file_time = parse_timestamp(file_timestamp);
                    if ~isnan(file_time)
                        diff = abs(file_time - target_time);
                        if diff < best_obj_diff
                            best_obj_diff = diff;
                            objectPath = fullfile(object_files(i).folder, object_files(i).name);
                        end
                    end
                end
            end
            
            if ~isempty(scenePath)
                fprintf('Found closest scene file: %s\n', scenePath);
            end
            if ~isempty(objectPath)
                fprintf('Found closest object file: %s\n', objectPath);
            end
        end
    end
end
