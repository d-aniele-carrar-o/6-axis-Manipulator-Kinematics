%% Robot Base Calibration Example
% This script demonstrates how to collect calibration data and compute
% the transformation between the world reference frame and the robot base frames.
clear; clc; close all;

%% Step 1: Define known parameters
% Height of the robot stands
stand_height = 0.215;  % meters

% Distance between robot stands along Y axis
y_distance = 1.1;  % meters

%% Step 2: Collect calibration data
% In a real scenario, you would move each robot to known positions in the world frame
% and record the joint configurations and the corresponding world frame poses.
% Here we'll simulate this process.

% Initialize robots
parameters(0, 1); % Left robot
robot_left = robot;
config_left = config;
Trf_0_l_initial = Trf_0;

parameters(0, 2); % Right robot
robot_right = robot;
config_right = config;
Trf_0_r_initial = Trf_0;

% Create figure for visualization
figure('Name', 'Robot Calibration Data Collection', 'Position', [100, 100, 1200, 800]);
hold on; grid on;

tableParams.height = tableHeight;
tableParams.width = tableWidth;
tableParams.length = tableLength;

axs = create_environment(tablePosition, tableParams);

% Define calibration points in world frame
% These are points where we'll place the robot end-effectors
% For example, we might use markers on the table at known positions
world_calibration_points = [
    % Point 1: Center of the table
    0.0, 0.0, tableHeight + 0.3;
    % Point 2: Front left corner
    -tableWidth/4, -tableLength/4, tableHeight + 0.25;
    % Point 3: Front right corner
    -tableWidth/4, tableLength/4, tableHeight + 0.25;
];

% Plot calibration points
scatter3(world_calibration_points(:,1), world_calibration_points(:,2), world_calibration_points(:,3), 100, 'r', 'filled');
for i = 1:size(world_calibration_points, 1)
    text(world_calibration_points(i,1), world_calibration_points(i,2), world_calibration_points(i,3) + 0.05, ...
        ['Point ', num2str(i)], 'FontSize', 10, 'HorizontalAlignment', 'center');
end

% Plot world coordinate frame
triad('Parent', axs, 'Matrix', eye(4), 'Scale', 0.2, 'LineWidth', 2);
text(0, 0, 0.05, 'World Frame', 'FontSize', 10, 'FontWeight', 'bold');

% In a real scenario, you would move each robot to these points and record the joint configurations
% Here we'll simulate this by using inverse kinematics

% Define orientations for each calibration point (in Euler angles XYZ)
% For simplicity, we'll use fixed orientations for each robot
left_orientations = repmat([0, pi, 0], size(world_calibration_points, 1), 1);
right_orientations = repmat([0, 0, 0], size(world_calibration_points, 1), 1);

% Initialize arrays to store joint configurations
q_left = zeros(size(world_calibration_points, 1), 6);
q_right = zeros(size(world_calibration_points, 1), 6);

% Initial joint configurations
q0_left = [pi/2, -pi/3, 2*pi/3, -pi/3, pi/2, 0];
q0_right = [-pi/2, -2*pi/3, -2*pi/3, -2*pi/3, -pi/2, 0];

% For each calibration point, compute the joint configurations
fprintf('Computing joint configurations for calibration points...\n');
for i = 1:size(world_calibration_points, 1)
    % Create target poses in world frame
    target_pos_left = world_calibration_points(i,:);
    target_rot_left = left_orientations(i,:);
    T_world_ee_left = eul2tform(target_rot_left, 'XYZ') * trvec2tform(target_pos_left);
    
    target_pos_right = world_calibration_points(i,:);
    target_rot_right = right_orientations(i,:);
    T_world_ee_right = eul2tform(target_rot_right, 'XYZ') * trvec2tform(target_pos_right);
    
    % Convert to robot base frame using initial transformations
    T_base_ee_left = inv(Trf_0_l_initial) * T_world_ee_left;
    T_base_ee_right = inv(Trf_0_r_initial) * T_world_ee_right;
    
    % In a real scenario, you would use inverse kinematics to find joint configurations
    % Here we'll just use some example configurations
    % In practice, you would use your inverse kinematics function:
    % q_left(i,:) = inverse_kinematics(T_base_ee_left, q0_left);
    % q_right(i,:) = inverse_kinematics(T_base_ee_right, q0_right);
    
    % For this example, we'll just use slightly modified initial configurations
    q_left(i,:) = q0_left + [0, -0.1*i, 0.1*i, -0.1*i, 0, 0.1*i];
    q_right(i,:) = q0_right + [0, 0.1*i, -0.1*i, 0.1*i, 0, -0.1*i];
    
    % Visualize robot at this configuration
    config_left_i = set_robot_configuration(q_left(i,:), config_left);
    config_right_i = set_robot_configuration(q_right(i,:), config_right);
    
    show(robot_left, config_left_i, "Visuals", "on", "Frames", "on", "FastUpdate", true, "PreservePlot", false, "Parent", axs);
    show(robot_right, config_right_i, "Visuals", "on", "Frames", "on", "FastUpdate", true, "PreservePlot", false, "Parent", axs);
    
    % Add text to indicate which calibration point this is
    title(['Calibration Point ', num2str(i)]);
    drawnow;
    pause(1);
end

% Combine positions and orientations to create poses
poses_left = [world_calibration_points, left_orientations];
poses_right = [world_calibration_points, right_orientations];

%% Step 3: Compute the transformations
[T_world_base_left, T_world_base_right] = compute_robot_base_transforms(poses_left, poses_right, q_left, q_right, stand_height, y_distance);

%% Step 4: Visualize the results
% Create a new figure for the results
figure('Name', 'Robot Base Calibration Results', 'Position', [100, 100, 1200, 800]);
hold on; grid on;

axs_result = create_environment(tablePosition, tableParams);

% Update robot base transformations with calibrated values
Trf_0_l_calibrated = T_world_base_left;
Trf_0_r_calibrated = T_world_base_right;

% Set initial robot configurations
config_left = set_robot_configuration(q0_left, config_left);
config_right = set_robot_configuration(q0_right, config_right);

% Show robots with calibrated base transformations
show(robot_left, config_left, "Visuals", "on", "Frames", "on", "FastUpdate", true, "PreservePlot", false, "Parent", axs_result);
show(robot_right, config_right, "Visuals", "on", "Frames", "on", "FastUpdate", true, "PreservePlot", false, "Parent", axs_result);

% Add text labels for each robot
left_base_pos = Trf_0_l_calibrated(1:3,4);
right_base_pos = Trf_0_r_calibrated(1:3,4);
text(left_base_pos(1), left_base_pos(2), tableHeight + 0.01, 'Robot Left (Calibrated)', ...
    'Color', 'r', 'FontSize', 12, 'FontWeight', 'bold', 'HorizontalAlignment', 'center', 'Parent', axs_result);
text(right_base_pos(1), right_base_pos(2), tableHeight + 0.01, 'Robot Right (Calibrated)', ...
    'Color', 'b', 'FontSize', 12, 'FontWeight', 'bold', 'HorizontalAlignment', 'center', 'Parent', axs_result);

% Plot world coordinate frame
triad('Parent', axs_result, 'Matrix', eye(4), 'Scale', 0.2, 'LineWidth', 2);
text(0, 0, 0.05, 'World Frame', 'FontSize', 10, 'FontWeight', 'bold', 'Parent', axs_result);

% Plot robot base coordinate frames
triad('Parent', axs_result, 'Matrix', T_world_base_left, 'Scale', 0.15, 'LineWidth', 2);
triad('Parent', axs_result, 'Matrix', T_world_base_right, 'Scale', 0.15, 'LineWidth', 2);

% Set view angle
view(45, 30);
title('Robot Base Calibration Results');

%% Step 5: Save the calibrated transformations
save('calibrated_robot_bases.mat', 'T_world_base_left', 'T_world_base_right');
fprintf('Calibrated transformations saved to calibrated_robot_bases.mat\n');

% Display instructions for using the calibrated transformations
fprintf('\nTo use these calibrated transformations in your code:\n');
fprintf('1. Load the saved transformations: load(''calibrated_robot_bases.mat'')\n');
fprintf('2. Update the robot base transformations in parameters.m or in your code:\n');
fprintf('   robot_base_transforms = {\n');
fprintf('       T_world_base_left,  %% Left robot\n');
fprintf('       T_world_base_right  %% Right robot\n');
fprintf('   };\n');

%% Step 6: Verify calibration accuracy
% In a real scenario, you would move the robots to new positions and verify
% that they reach the expected positions in the world frame
fprintf('\nVerification step: Move robots to new positions and verify accuracy...\n');

% Define a new verification point in world frame
verify_point = [0.1, 0.1, tableHeight + 0.3];
fprintf('Verification point in world frame: [%.4f, %.4f, %.4f]\n', verify_point);

% Compute joint configurations for this point using the calibrated transformations
% (In practice, you would use your inverse kinematics function)

% Visualize the verification
figure('Name', 'Calibration Verification', 'Position', [100, 100, 1200, 800]);
hold on; grid on;

axs_verify = create_environment(tablePosition, tableParams);

% Plot verification point
scatter3(verify_point(1), verify_point(2), verify_point(3), 100, 'g', 'filled');
text(verify_point(1), verify_point(2), verify_point(3) + 0.05, 'Verification Point', ...
    'FontSize', 10, 'HorizontalAlignment', 'center');

% Plot world coordinate frame
triad('Parent', axs_verify, 'Matrix', eye(4), 'Scale', 0.2, 'LineWidth', 2);
text(0, 0, 0.05, 'World Frame', 'FontSize', 10, 'FontWeight', 'bold', 'Parent', axs_verify);

% Show robots with calibrated base transformations
% (In practice, you would move the robots to the joint configurations computed above)
show(robot_left, config_left, "Visuals", "on", "Frames", "on", "FastUpdate", true, "PreservePlot", false, "Parent", axs_verify);
show(robot_right, config_right, "Visuals", "on", "Frames", "on", "FastUpdate", true, "PreservePlot", false, "Parent", axs_verify);

% Set view angle
view(45, 30);
title('Calibration Verification');

fprintf('\nCalibration process complete!\n');