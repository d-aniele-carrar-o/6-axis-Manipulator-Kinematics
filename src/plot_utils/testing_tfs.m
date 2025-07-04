clc; close all;
%% LOGIC SECTION
% Rectangle parameters
w = 0.15; h = 0.35;

% First rectangle transformation
center = [-0.1 + 0.2*rand; -0.2 + 0.4*rand]; 
ang = -30 + 60*rand;
ori = deg2rad(ang);
fprintf('Box: center: [%.3f,%.3f], ori: %.1f°\n', center(1), center(2), ang);
Tbox = rot_frame(ori, center);

% Random transformation and second rectangle
trasl = [-0.25 + 0.5*rand; -0.4 + 0.8*rand];
dang = -30 + 60*rand;
rot = deg2rad(dang);
fprintf('Aug: transl: [%.3f,%.3f], rot: %.1f°\n', trasl(1), trasl(2), dang);
% Create relative transformation using rot_frame
Taug = rot_frame(rot, trasl);

% DEMONSTRATION: Two different ways to apply transformation
% Method 1: Pre-multiplication (transformation in world frame)
T2_world = Taug * Tbox;  % Taug expressed in world coordinates

% Method 2: Post-multiplication (transformation in local frame)
T2_local = Tbox * Taug;  % Taug expressed in T1's local coordinates

% Use world frame transformation for our example
Tbox2 = T2_world;
fprintf('Box: new center: [%.3f,%.3f], new ori: %.1f°\n', Tbox2(1,3), Tbox2(2,3), rad2deg(atan2(Tbox2(2,1), Tbox2(1,1))));

% fprintf('World frame result: [%.3f, %.3f]\n', T2_world(1,3), T2_world(2,3));
% fprintf('Local frame result: [%.3f, %.3f]\n', T2_local(1,3), T2_local(2,3));

% Robots' base pose wrt world
Tbr = rot_frame(0, [0,  0.8]);
Tbl = rot_frame(0, [0, -0.8]);

% End-effector poses wrt world (based on the box position)
Ter = Tbox * rot_frame(pi, [-0.05,  h/2+0.05]);
Tel = Tbox * rot_frame( 0, [ 0.05, -h/2-0.05]);

% End-effector poses wrt bases
Tber = Tbr \ Ter;
Tbel = Tbl \ Tel;

% Apply same transformation to end-effector poses
Ter2 = Taug * Ter;
Tel2 = Taug * Tel;

%% VISUALIZATION SECTION
figure; hold on; view(90, 90); grid on;

x_lims = [-0.5,0.5]; y_lims = [-1.0,1.0];
frames_len = 0.05;

% Axes and world frame
plot(x_lims, [0 0], 'Color', [0.7 0.7 0.7], 'LineWidth', 1);
plot([0 0], y_lims, 'Color', [0.7 0.7 0.7], 'LineWidth', 1);
quiver(0, 0, frames_len*2, 0, 'r', 'LineWidth', 3, 'MaxHeadSize', 0.7);
quiver(0, 0, 0, frames_len*2, 'g', 'LineWidth', 3, 'MaxHeadSize', 0.7);

% ROBOTS AND OBJECTS -----------------------------------------------------------------------------------
% Robot right base
theta = linspace(0, 2*pi, 100);
circle_x = Tbr(1,3) + 0.05*cos(theta);
circle_y = Tbr(2,3) + 0.05*sin(theta);
fill(circle_x, circle_y, [0.7 0.9 1], 'FaceAlpha', 0.2, 'EdgeColor', [0.7 0.9 1], 'LineWidth', 2);
% Robot left base
circle_x = Tbl(1,3) + 0.05*cos(theta);
circle_y = Tbl(2,3) + 0.05*sin(theta);
fill(circle_x, circle_y, [0.7 0.9 1], 'FaceAlpha', 0.2, 'EdgeColor', [0.7 0.9 1], 'LineWidth', 2);

% Brown box
pts_base = [-w/2 w/2 w/2 -w/2 -w/2; -h/2 -h/2 h/2 h/2 -h/2; ones(1,5)];
pts1 = Tbox * pts_base;
fill(pts1(1,:), pts1(2,:), [0.6 0.4 0.2], 'FaceAlpha', 0.2, 'EdgeColor', [0.6 0.4 0.2], 'LineWidth', 2);
% Transformed box
pts2 = Tbox2 * pts_base;
fill(pts2(1,:), pts2(2,:), [0.5 0.5 0.5], 'FaceAlpha', 0.2, 'EdgeColor', [0.5 0.5 0.5], 'LineWidth', 2);
% ============================================================================================================
% Transformation lines ---------------------------------------------------------------------------------------
% - World -> boxes centroids
plot([0 Tbox(1,3)], [0 Tbox(2,3)], 'Color', [0.6 0.4 0.2], 'LineWidth', 2, 'LineStyle', '--');
plot([0 Tbox2(1,3)], [0 Tbox2(2,3)], 'Color', [0.5 0.5 0.5], 'LineWidth', 2, 'LineStyle', '--');

% - Robots' bases -> end-effector poses
plot([Tbr(1,3) Ter(1,3)], [Tbr(2,3) Ter(2,3)], 'Color', [0.7 0.9 1], 'LineWidth', 2, 'LineStyle', '--');
plot([Tbr(1,3) Ter2(1,3)], [Tbr(2,3) Ter2(2,3)], 'Color', [0.4 0.6 0.8], 'LineWidth', 2, 'LineStyle', '--');
plot([Tbl(1,3) Tel(1,3)], [Tbl(2,3) Tel(2,3)], 'Color', [0.7 0.9 1], 'LineWidth', 2, 'LineStyle', '--');
plot([Tbl(1,3) Tel2(1,3)], [Tbl(2,3) Tel2(2,3)], 'Color', [0.4 0.6 0.8], 'LineWidth', 2, 'LineStyle', '--');

% Relative tfs:
% - Original box centroid -> transformed box centroid
plot([Tbox(1,3) Tbox2(1,3)], [Tbox(2,3) Tbox2(2,3)], 'Color', [1 0 1], 'LineWidth', 3, 'LineStyle', '--');
% - Original ee right pose -> transformed ee right pose
plot([Ter(1,3) Ter2(1,3)], [Ter(2,3) Ter2(2,3)], 'Color', [1 0 1], 'LineWidth', 2, 'LineStyle', '--');
% - Original ee left pose -> transformed ee left pose
plot([Tel(1,3) Tel2(1,3)], [Tel(2,3) Tel2(2,3)], 'Color', [1 0 1], 'LineWidth', 2, 'LineStyle', '--');

% End-effector to box relationships:
% - Original ees -> original box
plot([Ter(1,3) Tbox(1,3)], [Ter(2,3) Tbox(2,3)], 'Color', [1 1 0], 'LineWidth', 2, 'LineStyle', '--');
plot([Tel(1,3) Tbox(1,3)], [Tel(2,3) Tbox(2,3)], 'Color', [1 1 0], 'LineWidth', 2, 'LineStyle', '--');
% - Transformed ees -> transformed box
plot([Ter2(1,3) Tbox2(1,3)], [Ter2(2,3) Tbox2(2,3)], 'Color', [0.6 0.6 0], 'LineWidth', 2, 'LineStyle', '--');
plot([Tel2(1,3) Tbox2(1,3)], [Tel2(2,3) Tbox2(2,3)], 'Color', [0.6 0.6 0], 'LineWidth', 2, 'LineStyle', '--');
% ====================================================================================================================
% FRAMES -------------------------------------------------------------------------------------------------------------
% Box frame
quiver(Tbox(1,3), Tbox(2,3), frames_len*Tbox(1,1), frames_len*Tbox(2,1), 'r', 'LineWidth', 2, 'MaxHeadSize', 0.5);
quiver(Tbox(1,3), Tbox(2,3), frames_len*Tbox(1,2), frames_len*Tbox(2,2), 'g', 'LineWidth', 2, 'MaxHeadSize', 0.5);
% Transformed box frame
quiver(Tbox2(1,3), Tbox2(2,3), frames_len*Tbox2(1,1), frames_len*Tbox2(2,1), 'r', 'LineWidth', 2, 'MaxHeadSize', 0.5);
quiver(Tbox2(1,3), Tbox2(2,3), frames_len*Tbox2(1,2), frames_len*Tbox2(2,2), 'g', 'LineWidth', 2, 'MaxHeadSize', 0.5);

% - Robot right base frame
quiver(Tbr(1,3), Tbr(2,3), frames_len*Tbr(1,1), frames_len*Tbr(2,1), 'r', 'LineWidth', 2, 'MaxHeadSize', 0.5);
quiver(Tbr(1,3), Tbr(2,3), frames_len*Tbr(1,2), frames_len*Tbr(2,2), 'g', 'LineWidth', 2, 'MaxHeadSize', 0.5);
% - Robot left base frame
quiver(Tbl(1,3), Tbl(2,3), frames_len*Tbl(1,1), frames_len*Tbl(2,1), 'r', 'LineWidth', 2, 'MaxHeadSize', 0.5);
quiver(Tbl(1,3), Tbl(2,3), frames_len*Tbl(1,2), frames_len*Tbl(2,2), 'g', 'LineWidth', 2, 'MaxHeadSize', 0.5);

% - End-effector right frame
quiver(Ter(1,3), Ter(2,3), frames_len*Ter(1,1), frames_len*Ter(2,1), 'r', 'LineWidth', 2, 'MaxHeadSize', 0.5);
quiver(Ter(1,3), Ter(2,3), frames_len*Ter(1,2), frames_len*Ter(2,2), 'g', 'LineWidth', 2, 'MaxHeadSize', 0.5);
% - End-effector left frame
quiver(Tel(1,3), Tel(2,3), frames_len*Tel(1,1), frames_len*Tel(2,1), 'r', 'LineWidth', 2, 'MaxHeadSize', 0.5);
quiver(Tel(1,3), Tel(2,3), frames_len*Tel(1,2), frames_len*Tel(2,2), 'g', 'LineWidth', 2, 'MaxHeadSize', 0.5);

% Transformed end-effector right frame
quiver(Ter2(1,3), Ter2(2,3), frames_len*Ter2(1,1), frames_len*Ter2(2,1), 'r', 'LineWidth', 2, 'MaxHeadSize', 0.5);
quiver(Ter2(1,3), Ter2(2,3), frames_len*Ter2(1,2), frames_len*Ter2(2,2), 'g', 'LineWidth', 2, 'MaxHeadSize', 0.5);
% Transformed end-effector left frame
quiver(Tel2(1,3), Tel2(2,3), frames_len*Tel2(1,1), frames_len*Tel2(2,1), 'r', 'LineWidth', 2, 'MaxHeadSize', 0.5);
quiver(Tel2(1,3), Tel2(2,3), frames_len*Tel2(1,2), frames_len*Tel2(2,2), 'g', 'LineWidth', 2, 'MaxHeadSize', 0.5);
% ====================================================================================================================
axis equal; xlim(x_lims); ylim(y_lims);

%% USEFUL FUNCTIONS
% Creates a 2D homogeneous transformation matrix
% Usage: rot_frame(angle, position) where position can be:
%   - Absolute coordinates (for poses w.r.t. world)
%   - Relative displacement (for transformations)
function res = rot_frame(angle, position, israd)
    if nargin < 3, israd = true; end
    if nargin < 2, position = [0; 0]; end
    if nargin < 1, angle = 0; end
    
    if ~israd, angle = deg2rad(angle); end
    
    res = [cos(angle), -sin(angle), position(1); sin(angle), cos(angle), position(2); 0, 0, 1];
end
