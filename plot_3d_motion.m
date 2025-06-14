% Script to read robot motion data from CSV and plot 3D poses
clear; clc; close all;

% Read the CSV file
data = readtable('/Users/danielecarraro/Documents/VSCODE/master-thesis/YOTO/data/1749729939_motion.csv');

% Create time index
time = (1:height(data))';

% Create figure for 3D visualization
figure('Name', 'Robot 3D Motion', 'Position', [100, 100, 1200, 800]);
hold on; grid on; box on;

% Define colors for robots 1 and 3
colors = {'r', 'b'};
robot_names = {'Robot 1', 'Robot 3'};

% Plot trajectories for robots 1 and 3
robot_indices = [1, 3];
for idx = 1:length(robot_indices)
    i = robot_indices(idx);
    % Extract position data for current robot
    x_col = sprintf('x%d', i);
    y_col = sprintf('y%d', i);
    z_col = sprintf('z%d', i);
    
    % Plot 3D trajectory
    plot3(data.(x_col), data.(y_col), data.(z_col), colors{idx}, 'LineWidth', 2);
    
    % Add markers for start and end positions
    scatter3(data.(x_col)(1), data.(y_col)(1), data.(z_col)(1), 100, colors{idx}, 'filled', 'MarkerEdgeColor', 'k');
    scatter3(data.(x_col)(end), data.(y_col)(end), data.(z_col)(end), 100, colors{idx}, 'filled', 'MarkerEdgeColor', 'k', 'Marker', 'diamond');
    
    % Add text labels for start and end positions
    text(data.(x_col)(1), data.(y_col)(1), data.(z_col)(1), [robot_names{idx} ' start'], 'Color', colors{idx}, 'FontWeight', 'bold');
    text(data.(x_col)(end), data.(y_col)(end), data.(z_col)(end), [robot_names{idx} ' end'], 'Color', colors{idx}, 'FontWeight', 'bold');
end

% Add visualization of orientation frames at specific intervals
interval = floor(height(data)/10); % Show 10 frames along the trajectory
robot_indices = [1, 3];
for idx = 1:length(robot_indices)
    i = robot_indices(idx);
    % Extract position and orientation data for current robot
    x_col = sprintf('x%d', i);
    y_col = sprintf('y%d', i);
    z_col = sprintf('z%d', i);
    rx_col = sprintf('rx%d', i);
    ry_col = sprintf('ry%d', i);
    rz_col = sprintf('rz%d', i);
    
    % Plot orientation frames at intervals
    for j = 1:interval:height(data)
        % Get position
        pos = [data.(x_col)(j), data.(y_col)(j), data.(z_col)(j)];
        
        % Convert Euler angles to rotation matrix (assuming XYZ Euler angles)
        rx = data.(rx_col)(j);
        ry = data.(ry_col)(j);
        rz = data.(rz_col)(j);
        
        % Create rotation matrix from Euler angles
        Rx = [1 0 0; 0 cos(rx) -sin(rx); 0 sin(rx) cos(rx)];
        Ry = [cos(ry) 0 sin(ry); 0 1 0; -sin(ry) 0 cos(ry)];
        Rz = [cos(rz) -sin(rz) 0; sin(rz) cos(rz) 0; 0 0 1];
        R = Rz * Ry * Rx;
        
        % Draw coordinate frame axes
        scale = 0.05; % Scale for the coordinate frame arrows
        quiver3(pos(1), pos(2), pos(3), R(1,1)*scale, R(2,1)*scale, R(3,1)*scale, colors{idx}, 'LineWidth', 2);
        quiver3(pos(1), pos(2), pos(3), R(1,2)*scale, R(2,2)*scale, R(3,2)*scale, colors{idx}, 'LineWidth', 2);
        quiver3(pos(1), pos(2), pos(3), R(1,3)*scale, R(2,3)*scale, R(3,3)*scale, colors{idx}, 'LineWidth', 2);
    end
end

% Set plot properties
xlabel('X (m)', 'FontSize', 12, 'FontWeight', 'bold');
ylabel('Y (m)', 'FontSize', 12, 'FontWeight', 'bold');
zlabel('Z (m)', 'FontSize', 12, 'FontWeight', 'bold');
title('Robot 1 & 3 Motion Trajectories', 'FontSize', 14, 'FontWeight', 'bold');
legend(robot_names, 'Location', 'best', 'FontSize', 12);

% Set view angle
view(45, 30);

% Add a colorbar to show time progression
colormap(jet);
c = colorbar;
c.Label.String = 'Time (samples)';
c.Label.FontSize = 12;
c.Label.FontWeight = 'bold';

% Make the plot look nice
axis equal;
grid on;