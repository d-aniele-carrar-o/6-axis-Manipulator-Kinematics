clc; close all;
%% LOGIC SECTION

% Import useful parameters
parameters(1);

% Import pointcloud object
timestamp = '25-06-24-21-38-38';
[scenePath, objectPath] = find_pointcloud_files(timestamp);

if isempty(objectPath)
    error('No object pointcloud found for timestamp %s', timestamp);
end

% Load first object pointcloud
[objectDir, ~, ~] = fileparts(objectPath);
objectFiles = dir(fullfile(objectDir, '*object*.ply'));
if isempty(objectFiles)
    error('No object files found');
end

objPath = fullfile(objectFiles(1).folder, objectFiles(1).name);
fprintf('Loading object from: %s\n', objPath);
objectPC = pcread(objPath);

% Apply same transformation as in simulate_recorded_motion
translation = [0, -0.025, 0];
objectPC = pctransform(objectPC, rigid3d(eye(3), translation));

% Extract object properties using PCA
points = objectPC.Location;
[center, dims, orientation] = extract_object_properties(points);
w = dims(1); h = dims(2); d = dims(3);

% Color original pointcloud
objectPC.Color = repmat(uint8([0.6 0.4 0.2] * 255), objectPC.Count, 1);

fprintf('Object: center: [%.3f,%.3f,%.3f], dims: [%.3f,%.3f,%.3f]\n', center, dims);
Tbox = rot_frame_3d(orientation, center);

% Random transformation and second box (wrt world frame)
trasl = [-0.4 + 0.8*rand+0.2; -0.5 + 1.0*rand; 0.0];
dang = -30 + 60*rand;
rot = deg2rad(dang);
fprintf('Aug: transl: [%.3f,%.3f,%.3f], rot: %.1f°\n', trasl(1), trasl(2), trasl(3), dang);
% Create relative transformation using rot_frame_3d
Taug = rot_frame_3d(rot, trasl);

% Apply transformation (world frame)
Tbox2 = Taug * Tbox;
fprintf('Box: new center: [%.3f,%.3f,%.3f]\n', Tbox2(1,4), Tbox2(2,4), Tbox2(3,4));

% Transform pointcloud the same way (apply Taug directly since PC is in world coordinates)
objectPC2 = pctransform(objectPC, rigid3d(Taug(1:3,1:3)', Taug(1:3,4)'));
objectPC2.Color = repmat(uint8([0.5 0.5 0.5] * 255), objectPC2.Count, 1);

% Robots' base poses wrt world
Tbr = rot_frame_3d(0, [0,  0.8, tableHeight]);
Tbl = rot_frame_3d(0, [0, -0.8, tableHeight]);

% End-effector poses wrt world (based on the box position, PREGRASP)
Ter = Tbox * rot_frame_3d(pi, [-0.05,  h/2+0.2, d]);
Tel = Tbox * rot_frame_3d( 0, [ 0.05, -h/2-0.2, d]);

% End-effector poses wrt world (based on the box position, GRASP)
Ter_g = Tbox * rot_frame_3d(pi, [-0.05,  h/2+0.05, d]);
Tel_g = Tbox * rot_frame_3d( 0, [ 0.05, -h/2-0.05, d]);

% Apply same transformation to end-effector poses (original and transformed)
Ter2 = Taug * Ter;
Tel2 = Taug * Tel;
Ter2_g = Taug * Ter_g;
Tel2_g = Taug * Tel_g;

%% VISUALIZATION SECTION
figure; hold on; view(45, 30); grid on;

x_lims = [-0.7,0.7]; y_lims = [-1.0,1.0]; z_lims = [-0.01,1.0];
frames_len = 0.07;

% World frame
quiver3(0, 0, 0, frames_len*2, 0, 0, 'r', 'LineWidth', 3, 'MaxHeadSize', 0.7);
quiver3(0, 0, 0, 0, frames_len*2, 0, 'g', 'LineWidth', 3, 'MaxHeadSize', 0.7);
quiver3(0, 0, 0, 0, 0, frames_len*2, 'b', 'LineWidth', 3, 'MaxHeadSize', 0.7);

% ROBOTS AND OBJECTS ------------------------------------------------------------------------------------------------
% Robot right base (cylinder)
[X, Y, Z] = cylinder(0.05, 20);
Z = Z * 0.02;
surf(X + Tbr(1,4), Y + Tbr(2,4), Z + Tbr(3,4), 'FaceColor', [0.7 0.9 1], 'FaceAlpha', 0.3, 'EdgeColor', [0.7 0.9 1]);

% Robot left base (cylinder)
surf(X + Tbl(1,4), Y + Tbl(2,4), Z + Tbl(3,4), 'FaceColor', [0.7 0.9 1], 'FaceAlpha', 0.3, 'EdgeColor', [0.7 0.9 1]);

% Original object pointcloud
pcshow(objectPC, 'MarkerSize', 20);
% Transformed object pointcloud
pcshow(objectPC2, 'MarkerSize', 20);

% Bounding box around object
draw_box(Tbox, [w h d], [0.6 0.4 0.2]);
% Transformed bounding box
draw_box(Tbox2, [w h d], [0.5 0.5 0.5]);

% ================================================================================================================================
% Transformation lines ------------------------------------------------------------------------------------------------------------
% - World -> boxes centroids
plot3([0 Tbox(1,4)], [0 Tbox(2,4)], [0 Tbox(3,4)], 'Color', [0.6 0.4 0.2], 'LineWidth', 2, 'LineStyle', '--');
plot3([0 Tbox2(1,4)], [0 Tbox2(2,4)], [0 Tbox2(3,4)], 'Color', [0.5 0.5 0.5], 'LineWidth', 2, 'LineStyle', '--');

% - Robots' bases -> end-effector poses
plot3([Tbr(1,4) Ter(1,4)], [Tbr(2,4) Ter(2,4)], [Tbr(3,4) Ter(3,4)], 'Color', [0.7 0.9 1], 'LineWidth', 2, 'LineStyle', '--');
plot3([Tbr(1,4) Ter2(1,4)], [Tbr(2,4) Ter2(2,4)], [Tbr(3,4) Ter2(3,4)], 'Color', [0.4 0.6 0.8], 'LineWidth', 2, 'LineStyle', '--');
plot3([Tbl(1,4) Tel(1,4)], [Tbl(2,4) Tel(2,4)], [Tbl(3,4) Tel(3,4)], 'Color', [0.7 0.9 1], 'LineWidth', 2, 'LineStyle', '--');
plot3([Tbl(1,4) Tel2(1,4)], [Tbl(2,4) Tel2(2,4)], [Tbl(3,4) Tel2(3,4)], 'Color', [0.4 0.6 0.8], 'LineWidth', 2, 'LineStyle', '--');

% Relative tfs:
% - Original box centroid -> transformed box centroid
plot3([Tbox(1,4) Tbox2(1,4)], [Tbox(2,4) Tbox2(2,4)], [Tbox(3,4) Tbox2(3,4)], 'Color', [1 0 1], 'LineWidth', 3, 'LineStyle', '--');
% - Original ee poses -> transformed ee poses
plot3([Ter(1,4) Ter2(1,4)], [Ter(2,4) Ter2(2,4)], [Ter(3,4) Ter2(3,4)], 'Color', [1 0 1], 'LineWidth', 2, 'LineStyle', '--');
plot3([Tel(1,4) Tel2(1,4)], [Tel(2,4) Tel2(2,4)], [Tel(3,4) Tel2(3,4)], 'Color', [1 0 1], 'LineWidth', 2, 'LineStyle', '--');

% End-effector to box relationships:
% - Original ees GRASP -> original box
plot3([Ter_g(1,4) Tbox(1,4)], [Ter_g(2,4) Tbox(2,4)], [Ter_g(3,4) Tbox(3,4)], 'Color', [1 1 0], 'LineWidth', 2, 'LineStyle', '--');
plot3([Tel_g(1,4) Tbox(1,4)], [Tel_g(2,4) Tbox(2,4)], [Tel_g(3,4) Tbox(3,4)], 'Color', [1 1 0], 'LineWidth', 2, 'LineStyle', '--');
% - Transformed ees GRASP -> transformed box
plot3([Ter2_g(1,4) Tbox2(1,4)], [Ter2_g(2,4) Tbox2(2,4)], [Ter2_g(3,4) Tbox2(3,4)], 'Color', [0.6 0.6 0], 'LineWidth', 2, 'LineStyle', '--');
plot3([Tel2_g(1,4) Tbox2(1,4)], [Tel2_g(2,4) Tbox2(2,4)], [Tel2_g(3,4) Tbox2(3,4)], 'Color', [0.6 0.6 0], 'LineWidth', 2, 'LineStyle', '--');

% - Original ees PREGRASP -> original ees GRASP
plot3([Ter(1,4) Ter_g(1,4)], [Ter(2,4) Ter_g(2,4)], [Ter(3,4) Ter_g(3,4)], 'Color', [1 1 1], 'LineWidth', 2);
plot3([Tel(1,4) Tel_g(1,4)], [Tel(2,4) Tel_g(2,4)], [Tel(3,4) Tel_g(3,4)], 'Color', [1 1 1], 'LineWidth', 2);
% - Transformed ees PREGRASP -> transformed ees GRASP
plot3([Ter2(1,4) Ter2_g(1,4)], [Ter2(2,4) Ter2_g(2,4)], [Ter2(3,4) Ter2_g(3,4)], 'Color', [0.7 0.7 0.7], 'LineWidth', 2);
plot3([Tel2(1,4) Tel2_g(1,4)], [Tel2(2,4) Tel2_g(2,4)], [Tel2(3,4) Tel2_g(3,4)], 'Color', [0.7 0.7 0.7], 'LineWidth', 2);

% =====================================================================================================================================
% FRAMES ------------------------------------------------------------------------------------------------------------------------------
% Box frames
draw_frame_3d(Tbox, frames_len);
draw_frame_3d(Tbox2, frames_len);

% Robot base frames
draw_frame_3d(Tbr, frames_len);
draw_frame_3d(Tbl, frames_len);

% End-effector frames
draw_frame_3d(Ter, frames_len);
draw_frame_3d(Tel, frames_len);
draw_frame_3d(Ter_g, frames_len);
draw_frame_3d(Tel_g, frames_len);
draw_frame_3d(Ter2, frames_len);
draw_frame_3d(Tel2, frames_len);
draw_frame_3d(Ter2_g, frames_len);
draw_frame_3d(Tel2_g, frames_len);

% =====================================================================================================================================
axis equal; xlim(x_lims); ylim(y_lims); zlim(z_lims);
xlabel('X'); ylabel('Y'); zlabel('Z');

%% USEFUL FUNCTIONS
% Extract object properties using PCA
function [center, dims, orientation] = extract_object_properties(points)
    % Compute centroid
    center = mean(points, 1)';
    
    % Center the points
    centered_points = points - center';
    
    % Perform PCA
    [coeff, ~, ~] = pca(centered_points);
    
    % Transform points to principal component space
    transformed_points = centered_points * coeff;
    
    % Compute bounding box dimensions in PC space
    min_vals = min(transformed_points, [], 1);
    max_vals = max(transformed_points, [], 1);
    pc_dims = (max_vals - min_vals)';
    
    % Find which PC axis is longer (should be Y)
    [~, longest_idx] = max(pc_dims(1:2));
    
    if longest_idx == 1  % First PC is longer, make it Y
        dims = [pc_dims(2); pc_dims(1); pc_dims(3)];
        % Y-axis direction from first PC
        y_vec = coeff(:, 1);
        orientation = atan2(y_vec(2), y_vec(1)) - pi/2;
    else  % Second PC is longer, make it Y
        dims = pc_dims;
        % Y-axis direction from second PC
        y_vec = coeff(:, 2);
        orientation = atan2(y_vec(2), y_vec(1));
    end
    
    % Ensure Y-axis points toward positive world Y
    if y_vec(2) < 0
        orientation = orientation + pi;
    end
end
% Creates a 3D homogeneous transformation matrix (rotation around Z-axis)
function res = rot_frame_3d(angle, position, israd)
    if nargin < 3, israd = true; end
    if nargin < 2, position = [0; 0; 0]; end
    if nargin < 1, angle = 0; end
    
    if ~israd, angle = deg2rad(angle); end
    
    res = [cos(angle), -sin(angle), 0, position(1); 
           sin(angle),  cos(angle), 0, position(2); 
           0,           0,          1, position(3); 
           0,           0,          0, 1];
end

% Draws a 3D box given transformation matrix and dimensions
function draw_box(T, dims, color)
    w = dims(1); h = dims(2); d = dims(3);
    
    % Define box vertices
    vertices = [-w/2 -h/2 -d/2; w/2 -h/2 -d/2; w/2 h/2 -d/2; -w/2 h/2 -d/2;
                -w/2 -h/2  d/2; w/2 -h/2  d/2; w/2 h/2  d/2; -w/2 h/2  d/2];
    
    % Transform vertices
    vertices_h = [vertices ones(8,1)]';
    vertices_t = T * vertices_h;
    vertices_t = vertices_t(1:3,:)';
    
    % Define faces
    faces = [1 2 3 4; 5 6 7 8; 1 2 6 5; 2 3 7 6; 3 4 8 7; 4 1 5 8];
    
    % Draw box
    patch('Vertices', vertices_t, 'Faces', faces, 'FaceColor', color, 'FaceAlpha', 0.3, 'EdgeColor', color, 'LineWidth', 2);
end

% Draws a 3D coordinate frame
function draw_frame_3d(T, len)
    pos = T(1:3, 4);
    x_axis = T(1:3, 1) * len;
    y_axis = T(1:3, 2) * len;
    z_axis = T(1:3, 3) * len;
    
    quiver3(pos(1), pos(2), pos(3), x_axis(1), x_axis(2), x_axis(3), 'r', 'LineWidth', 2, 'MaxHeadSize', 0.5);
    quiver3(pos(1), pos(2), pos(3), y_axis(1), y_axis(2), y_axis(3), 'g', 'LineWidth', 2, 'MaxHeadSize', 0.5);
    quiver3(pos(1), pos(2), pos(3), z_axis(1), z_axis(2), z_axis(3), 'b', 'LineWidth', 2, 'MaxHeadSize', 0.5);
end
