function data = readNPY(filename)
% READNPY Read NumPy .npy files in MATLAB
% Simple implementation for basic .npy file reading
%
% Input:
%   filename - Path to .npy file
%
% Output:
%   data - Array data from the .npy file

fid = fopen(filename, 'r');
if fid == -1
    error('Could not open file: %s', filename);
end

try
    % Read magic string
    magic = fread(fid, 6, 'uint8');
    if ~isequal(magic', [147 'NUMPY'])
        error('Not a valid .npy file');
    end
    
    % Read version
    major_version = fread(fid, 1, 'uint8');
    minor_version = fread(fid, 1, 'uint8');
    
    % Read header length
    if major_version == 1
        header_len = fread(fid, 1, 'uint16');
    else
        header_len = fread(fid, 1, 'uint32');
    end
    
    % Read header
    header = char(fread(fid, header_len, 'uint8')');
    
    % Parse header to get shape and dtype
    % Simple parsing for float64 arrays
    if contains(header, 'float64') || contains(header, '<f8')
        dtype = 'double';
    elseif contains(header, 'float32') || contains(header, '<f4')
        dtype = 'single';
    else
        error('Unsupported data type in .npy file');
    end
    
    % Extract shape
    shape_start = strfind(header, '(');
    shape_end = strfind(header, ')');
    if ~isempty(shape_start) && ~isempty(shape_end)
        shape_str = header(shape_start(1)+1:shape_end(1)-1);
        shape_str = strrep(shape_str, ' ', '');
        if endsWith(shape_str, ',')
            shape_str = shape_str(1:end-1);
        end
        shape_parts = split(shape_str, ',');
        shape = cellfun(@str2double, shape_parts);
    else
        error('Could not parse array shape from header');
    end
    
    % Read data
    data = fread(fid, prod(shape), dtype);
    data = reshape(data, shape');
    
    fclose(fid);
    
catch ME
    fclose(fid);
    rethrow(ME);
end

end