function motion_file = find_closest_motion_file(target_timestamp)
    parameters(1);
    
    files = dir(fullfile(data_folder, '**', '*_motion*'));
    
    if isempty(files)
        motion_file = '';
        return;
    end
    
    target_time = parse_timestamp(target_timestamp);
    if isnan(target_time)
        motion_file = '';
        return;
    end
    
    best_diff = inf;
    motion_file = '';
    for i = 1:length(files)
        file_timestamp = extract_timestamp_from_filename(files(i).name);
        if ~isempty(file_timestamp)
            file_time = parse_timestamp(file_timestamp);
            if ~isnan(file_time)
                diff = abs(file_time - target_time);
                if diff < best_diff
                    best_diff = diff;
                    motion_file = fullfile(files(i).folder, files(i).name);
                end
            end
        end
    end
end
