function tform_cam_to_world = load_camera_calibration(calibration_path)
% LOAD_CAMERA_CALIBRATION Load camera calibration from Python output
%
% This function loads the camera-to-world transformation matrix generated
% by the Python calibration script and converts it to MATLAB format.
%
% Input:
%   calibration_path - Path to the calibration file (without extension)
%                     Default: 'output/camera_calibration'
%
% Output:
%   tform_cam_to_world - rigidtform3d object for camera-to-world transformation

if nargin < 1
    % Default path relative to project root
    project_root = fileparts(fileparts(fileparts(mfilename('fullpath'))));
    calibration_path = fullfile(project_root, 'output', 'camera_calibration');
end

% Try to load the transformation matrix
json_file = [calibration_path, '.json'];

if exist(json_file, 'file')
    % Load from JSON file
    fid = fopen(json_file, 'r');
    raw = fread(fid, inf);
    str = char(raw');
    fclose(fid);
    
    data = jsondecode(str);
    T_cam_to_world = reshape(data.transformation_matrix, 4, 4)';
    fprintf('Loaded camera calibration from: %s\n', json_file);
    
else
    error('Camera calibration file not found at: %s', calibration_path);
end

% Convert to MATLAB rigidtform3d object
R = T_cam_to_world(1:3, 1:3);
t = T_cam_to_world(1:3, 4);
tform_cam_to_world = rigidtform3d(R, t');

% Display calibration info
fprintf('Camera-to-world transformation loaded:\n');
fprintf('  Translation: [%.3f, %.3f, %.3f] m\n', t);
fprintf('  Camera height: %.3f m\n', t(3));

end
