function timestamp_str = extract_timestamp_from_filename(filename)
    % Extract YY-MM-DD-hh-mm pattern from filename
    pattern = '\d{2}-\d{2}-\d{2}-\d{2}-\d{2}';
    match = regexp(filename, pattern, 'match');
    if ~isempty(match)
        timestamp_str = [match{1}, '-00']; % Add seconds
    else
        % Try to extract Linux timestamp (assuming it's a number at the start of the filename)
        linux_pattern = '^(\d+)_';
        linux_match = regexp(filename, linux_pattern, 'tokens');
        if ~isempty(linux_match)
            linux_timestamp = str2double(linux_match{1}{1});
            % Convert Linux timestamp to datetime
            dt = datetime(linux_timestamp, 'ConvertFrom', 'posixtime');
            % Format datetime to required string format
            timestamp_str = datestr(dt, 'yy-mm-dd-HH-MM-SS');
        else
            timestamp_str = '';
        end
    end
end
