%% Intel RealSense D415 Camera Calibration Example
% This script demonstrates how to calibrate an Intel RealSense D415 camera
% mounted approximately 1 meter above a table surface using a checkerboard pattern.

clear; clc; close all;

%% Setup parameters optimized for RealSense D415 at 1m height

% World parameters
worldParams = struct();
worldParams.tableHeight = 0.72;  % Adjust to your table height in meters

% Checkerboard parameters optimized for RealSense at 1m height
checkerboardParams = struct();
checkerboardParams.squareSize = 0.035;  % 3.5cm squares
checkerboardParams.patternSize = [7, 9];  % 8×10 squares (7×9 inner corners)
checkerboardParams.worldLocation = [0, 0, worldParams.tableHeight];  % Center of table
checkerboardParams.worldOrientation = eye(3);  % Checkerboard aligned with world frame

% Paths to image files - replace with your actual file paths
rgbImagePath = 'path/to/rgb_image.png';
depthImagePath = 'path/to/depth_image.png';

%% Run calibration
fprintf('Running checkerboard calibration for RealSense D415...\n');
tform_cam_to_world = calibrate_camera_checkerboard(rgbImagePath, depthImagePath, checkerboardParams, worldParams, true);

% Save the calibration result
save('realsense_d415_calibration.mat', 'tform_cam_to_world');
fprintf('Calibration complete. Results saved to realsense_d415_calibration.mat\n');

%% Multiple position calibration (recommended for higher accuracy)
% Uncomment and modify this section to use multiple checkerboard positions

% % Define multiple checkerboard positions
% positions = {
%     [0, 0, worldParams.tableHeight],     % Center
%     [0.2, 0.2, worldParams.tableHeight], % Top-right
%     [-0.2, 0.2, worldParams.tableHeight], % Top-left
%     [0.2, -0.2, worldParams.tableHeight], % Bottom-right
%     [-0.2, -0.2, worldParams.tableHeight] % Bottom-left
% };
% 
% % Calibrate with each position
% numPositions = length(positions);
% allTransforms = cell(numPositions, 1);
% 
% for i = 1:numPositions
%     fprintf('Calibrating position %d/%d...\n', i, numPositions);
%     
%     % Update checkerboard position
%     checkerboardParams.worldLocation = positions{i};
%     
%     % Update image paths for this position
%     rgbImagePath = sprintf('path/to/rgb_image_pos%d.png', i);
%     depthImagePath = sprintf('path/to/depth_image_pos%d.png', i);
%     
%     % Run calibration
%     tform = calibrate_camera_checkerboard(rgbImagePath, depthImagePath, checkerboardParams, worldParams, false);
%     allTransforms{i} = tform;
%     
%     % Save individual calibration
%     save(sprintf('realsense_calibration_pos%d.mat', i), 'tform');
% end
% 
% % Average the transformations
% fprintf('Averaging transformations from %d positions...\n', numPositions);
% 
% % Extract rotations and translations
% allQuats = zeros(numPositions, 4);
% allTrans = zeros(numPositions, 3);
% 
% for i = 1:numPositions
%     tform = allTransforms{i};
%     allQuats(i,:) = rotm2quat(tform.R);
%     allTrans(i,:) = tform.T;
% end
% 
% % Ensure quaternions are in the same hemisphere
% for i = 2:numPositions
%     if dot(allQuats(1,:), allQuats(i,:)) < 0
%         allQuats(i,:) = -allQuats(i,:);
%     end
% end
% 
% % Average quaternions and normalize
% avgQuat = mean(allQuats, 1);
% avgQuat = avgQuat / norm(avgQuat);
% 
% % Average translations
% avgTrans = mean(allTrans, 1);
% 
% % Create final transformation
% tform_cam_to_world = rigidtform3d(quat2rotm(avgQuat), avgTrans);
% 
% % Save final calibration
% save('realsense_d415_calibration_final.mat', 'tform_cam_to_world');
% fprintf('Final calibration complete. Results saved to realsense_d415_calibration_final.mat\n');

%% Verification (uncomment to use)
% % Load a point cloud to verify calibration
% ptCloud = pcread('path/to/verification_pointcloud.ply');
% 
% % Transform to world coordinates
% ptCloud_world = pctransform(ptCloud, tform_cam_to_world);
% 
% % Visualize
% figure('Name', 'Calibration Verification', 'Position', [100, 100, 1200, 800]);
% 
% % Create a simulated table surface
% [X, Y] = meshgrid(-0.5:0.05:0.5, -0.5:0.05:0.5);
% Z = ones(size(X)) * worldParams.tableHeight;
% surf(X, Y, Z, 'FaceColor', [0.8 0.7 0.6], 'FaceAlpha', 0.3, 'EdgeColor', 'none');
% hold on;
% 
% % Plot the transformed point cloud
% pcshow(ptCloud_world, 'MarkerSize', 20);
% 
% % Plot world coordinate axes
% axisLength = 0.2;
% plot3([0, axisLength], [0, 0], [0, 0], 'r-', 'LineWidth', 3); % X-axis
% plot3([0, 0], [0, axisLength], [0, 0], 'g-', 'LineWidth', 3); % Y-axis
% plot3([0, 0], [0, 0], [axisLength], 'b-', 'LineWidth', 3); % Z-axis
% 
% % Plot the camera position
% plotCamera('AbsolutePose', tform_cam_to_world, 'Size', 0.1, 'Color', 'b', 'Opacity', 0.7);
% 
% title('Calibration Verification');
% xlabel('X (m)'); ylabel('Y (m)'); zlabel('Z (m)');
% axis equal; grid on;
% view(30, 30);
% 
% % Check if points near the table are at the correct height
% tablePoints = ptCloud_world.Location(:,3);
% tablePoints = tablePoints(abs(tablePoints - worldParams.tableHeight) < 0.02);
% 
% fprintf('Table height verification:\n');
% fprintf('  - Expected height: %.3f m\n', worldParams.tableHeight);
% fprintf('  - Measured height: %.3f m (mean of points near table)\n', mean(tablePoints));
% fprintf('  - Height error: %.1f mm\n', abs(mean(tablePoints) - worldParams.tableHeight) * 1000);