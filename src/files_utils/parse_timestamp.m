function time_val = parse_timestamp(timestamp_str)
    try
        dt = datetime(timestamp_str, 'InputFormat', 'yy-MM-dd-HH-mm-ss');
        time_val = posixtime(dt);
    catch
        time_val = NaN;
    end
end
