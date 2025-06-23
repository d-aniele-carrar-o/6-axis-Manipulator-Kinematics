function timestamp_str = extract_timestamp_from_filename(filename)
    % Extract YY-MM-DD-hh-mm pattern from filename
    pattern = '\d{2}-\d{2}-\d{2}-\d{2}-\d{2}';
    match = regexp(filename, pattern, 'match');
    if ~isempty(match)
        timestamp_str = [match{1}, '-00']; % Add seconds
    else
        timestamp_str = '';
    end
end
