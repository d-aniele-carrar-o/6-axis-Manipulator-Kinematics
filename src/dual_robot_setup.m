% Dual robot setup and simulation
clear; clc; close all;

% Setup first robot (UR3e left)
parameters(0, 1);
robot_left  = robot;
config_left = config;
Trf_0_l     = Trf_0;

% Setup second robot (UR3e right)
parameters(0, 2);
robot_right  = robot;
config_right = config;
Trf_0_r      = Trf_0;

% Create environment for multi-robot setup --------------------------------------------------------
figure('Name', 'Dual Robot Workspace');
hold on; grid on;

tableParams.height = tableHeight;
tableParams.width = tableWidth;
tableParams.length = tableLength;

axs = create_environment( tablePosition, tableParams );

% --- File Paths ---
scenePcdPath = '/Users/danielecarraro/Documents/GITHUB/6-axis-Manipulator-Kinematics/pointclouds/pointcloud_25-06-07-11-21-29.ply';
objectPcdPath = '/Users/danielecarraro/Documents/GITHUB/6-axis-Manipulator-Kinematics/pointclouds/segmented_objects/pointcloud_25-06-07-11-21-29/object_00.ply';
% scenePcdPath = '/Volumes/Shared_part/realsense_data/pointcloud/pointcloud_25-06-12-12-05-30.ply';
% objectPcdPath = '/Volumes/Shared_part/realsense_data/segmented_objects/25-06-12-12-05-30/object_02.ply';

% Calibrate camera and transform object point cloud
[tform_cam_to_world, ptCloudObject_world, ptCloudRemaining_world] = calibrate_camera( scenePcdPath, objectPcdPath, tableParams, false );

% Display camera transformation
disp('Camera-to-world transformation:');
disp(tform_cam_to_world.A);

% Add camera visualization
plotCamera('AbsolutePose', tform_cam_to_world, 'Size', 0.1, 'Color', 'b', 'Opacity', 0.5, 'Parent', axs);
text(tform_cam_to_world.Translation(1), tform_cam_to_world.Translation(2), tform_cam_to_world.Translation(3) + 0.1, ...
    'Camera', 'Color', 'b', 'Parent', axs);

% Add scene point cloud
pcshow(ptCloudRemaining_world.Location, [0.7 0.7 0.7], 'MarkerSize', 10);

% Add object point cloud
pcshow(ptCloudObject_world.Location, 'r', 'MarkerSize', 30, 'Parent', axs);
text(mean(ptCloudObject_world.Location(:,1)), mean(ptCloudObject_world.Location(:,2)), mean(ptCloudObject_world.Location(:,3)) + 0.05, ...
    'Object', 'Color', 'r', 'Parent', axs);

% Display object position
object_centroid = mean(ptCloudObject_world.Location);
disp('Object centroid position in world frame:');
disp(object_centroid);

% Find grasp points on the object
[grasp_points, grasp_orientations] = find_object_grasp_points(ptCloudObject_world);

% Display grasp points
disp('Grasp points:');
disp(grasp_points);

% Visualize grasp points
scatter3(grasp_points(1,1), grasp_points(1,2), grasp_points(1,3), 100, 'g*', 'Parent', axs);
scatter3(grasp_points(2,1), grasp_points(2,2), grasp_points(2,3), 100, 'g*', 'Parent', axs);
text(grasp_points(1,1), grasp_points(1,2), grasp_points(1,3) + 0.05, 'Grasp 1', 'Color', 'g', 'Parent', axs);
text(grasp_points(2,1), grasp_points(2,2), grasp_points(2,3) + 0.05, 'Grasp 2', 'Color', 'g', 'Parent', axs);

% Visualize grasp orientations
quiver3(grasp_points(1,1), grasp_points(1,2), grasp_points(1,3), ...
       squeeze(grasp_orientations(1,1,3)), squeeze(grasp_orientations(1,2,3)), squeeze(grasp_orientations(1,3,3)), ...
       0.05, 'g', 'LineWidth', 2, 'Parent', axs);
quiver3(grasp_points(2,1), grasp_points(2,2), grasp_points(2,3), ...
       squeeze(grasp_orientations(2,1,3)), squeeze(grasp_orientations(2,2,3)), squeeze(grasp_orientations(2,3,3)), ...
       0.05, 'g', 'LineWidth', 2, 'Parent', axs);

% Initial joint configurations --------------------------------------------------------------------
q0_left  = [ pi/2,   -pi/3,  2*pi/3,   -pi/3,  pi/2, 0]
q0_right = [-pi/2, -2*pi/3, -2*pi/3, -2*pi/3, -pi/2, 0]

% Set robot configurations
config_left  = set_robot_configuration( q0_left,  config_left );
config_right = set_robot_configuration( q0_right, config_right );

% Visualize robots
show( robot_left,  config_left,  "Visuals", "on", "Frames", "on", "FastUpdate", true, "PreservePlot", false, "Parent", axs );
show( robot_right, config_right, "Visuals", "on", "Frames", "on", "FastUpdate", true, "PreservePlot", false, "Parent", axs );

% Add text labels for each robot on the table surface
left_base_pos = Trf_0_l(1:3,4);
right_base_pos = Trf_0_r(1:3,4);
text(left_base_pos(1), left_base_pos(2), tableHeight + 0.01, 'Robot Left', ...
    'Color', 'b', 'FontSize', 12, 'FontWeight', 'bold', 'HorizontalAlignment', 'center', 'Parent', axs);
text(right_base_pos(1), right_base_pos(2), tableHeight + 0.01, 'Robot Right', ...
    'Color', 'r', 'FontSize', 12, 'FontWeight', 'bold', 'HorizontalAlignment', 'center', 'Parent', axs);

% Compute end-effector poses 
parameters(1, 1); % Load parameters for robot 1
[Te_w_e_left,  Te_l] = direct_kinematics( q0_left, 1 );

parameters(1, 2); % Load parameters for robot 2
[Te_w_e_right, Te_r] = direct_kinematics( q0_right, 2 );

% Display end-effector positions
disp('Robot left end-effector pose (wTee):');
disp(Te_w_e_left);
disp('Robot left end-effector pose (bTee):');
disp(Te_l)
disp('--------------------------------------')
disp('Robot right end-effector pose (wTee):');
disp(Te_w_e_right);
disp('Robot right end-effector pose (bTee):');
disp(Te_r)
disp('--------------------------------------')


view(90, 30); % Set frontal view of the table
pause()

% Create trajectories for both robots -------------------------------------------------------------
disp('Planning trajectories for both robots...');
%         /------------/
%        /            /
%       /      R     /
%    + /     |/     /
%   Y /      . -   / world reference frame
%  - /            /
%   /      L     /
%  /            /
% /------------/
%     - X +

% Create viapoints for both robots in world reference frame
ti   = 0

% First viapoint: set first desired configuration/pose (viapoint) using grasp points
% Create transformation matrices for grasp points
R_grasp_l = squeeze(grasp_orientations(1,:,:));
pf_l = grasp_points(1,:)';

% Add a small offset in the approach direction (z-axis of grasp orientation)
% Negative offset to approach from outside the object
approach_offset = 0.05; % 5cm offset for approach
pf_l = pf_l - approach_offset * R_grasp_l(:,3);
T_w_o_l = [R_grasp_l, pf_l; 0,0,0,1]
Tf_l = inv(Trf_0_l) * T_w_o_l

R_grasp_r = squeeze(grasp_orientations(2,:,:));
pf_r = grasp_points(2,:)';
% Add a small offset in the approach direction
pf_r = pf_r - approach_offset * R_grasp_r(:,3);
T_w_o_r = [R_grasp_r, pf_r; 0,0,0,1]
Tf_r = inv(Trf_0_r) * T_w_o_r

t1   = 1

% Second viapoint: move to actual grasp positions (no offset)
pf2_l = grasp_points(1,:)';
T2_w_o_l = [R_grasp_l, pf2_l; 0,0,0,1]
Tf2_l = inv(Trf_0_l) * T2_w_o_l

pf2_r = grasp_points(2,:)';
T2_w_o_r = [R_grasp_r, pf2_r; 0,0,0,1]
Tf2_r = inv(Trf_0_r) * T2_w_o_r

t2   = 2

% Third viapoint: lift object together
pf3_l = grasp_points(1,:)' + [0; 0; 0.2]; % Lift 20cm up
T3_w_o_l = [R_grasp_l, pf3_l; 0,0,0,1]
Tf3_l = inv(Trf_0_l) * T3_w_o_l

pf3_r = grasp_points(2,:)' + [0; 0; 0.2]; % Lift 20cm up
T3_w_o_r = [R_grasp_r, pf3_r; 0,0,0,1]
Tf3_r = inv(Trf_0_r) * T3_w_o_r

t3   = 3

viapoints_l = [Tf_l];
viapoints_r = [Tf_r];
times       = [ti, t1];

% Compute multi-viapoint trajectory for selected times and viapoints for both robots --------------
% First: compute the trajectory to the pre-grasp poses (they might take different amount of time 
%        and we need to make sure that the steps after are completely synchronous)
[t_l_appr, p_l_appr, v_l_appr] = multipoint_trajectory( q0_left,  viapoints_l, times );
[t_r_appr, p_r_appr, v_r_appr] = multipoint_trajectory( q0_right, viapoints_r, times );

viapoints_l = [Tf2_l; Tf3_l];
viapoints_r = [Tf2_r; Tf3_r];
times       = [t1, t2, t3];

% Second: compute trajectory for the following pieces of the task, as the two manipulators will now 
%         be now synchronous
[t_l, p_l, v_l] = multipoint_trajectory( p_l_appr(end,:),  viapoints_l, times );
[t_r, p_r, v_r] = multipoint_trajectory( p_r_appr(end,:), viapoints_r, times );

% Run the simulation ------------------------------------------------------------------------------
[qf, axs] = simulate_dual({robot_left, robot_right}, {config_left, config_right}, ...
                          {Trf_0_l, Trf_0_r}, {p_l_appr, p_r_appr}, axs);
[qf, axs] = simulate_dual({robot_left, robot_right}, {config_left, config_right}, ...
                          {Trf_0_l, Trf_0_r}, {p_l, p_r}, axs);
