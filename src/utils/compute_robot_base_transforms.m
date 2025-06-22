function [T_world_base_left, T_world_base_right] = compute_robot_base_transforms(poses_left, poses_right, q_left, q_right, stand_height, y_distance)
% COMPUTE_ROBOT_BASE_TRANSFORMS Computes the transformation between world and robot base frames
%
% Inputs:
%   poses_left  - 3x6 matrix with known poses [x,y,z,rx,ry,rz] of left robot in world frame
%   poses_right - 3x6 matrix with known poses [x,y,z,rx,ry,rz] of right robot in world frame
%   q_left      - 3x6 matrix with joint configurations for left robot corresponding to poses
%   q_right     - 3x6 matrix with joint configurations for right robot corresponding to poses
%   stand_height- Height of the robot stands
%   y_distance  - Distance between robot stands along Y axis
%
% Outputs:
%   T_world_base_left  - 4x4 homogeneous transformation from world to left robot base
%   T_world_base_right - 4x4 homogeneous transformation from world to right robot base
%
% This function uses known poses in world frame and corresponding joint configurations
% to compute the transformation between world frame and robot base frames.

    % Number of calibration poses
    num_poses = size(poses_left, 1);
    
    % Initialize parameters for both robots
    parameters(0, 1); % Left robot
    AL_left = AL;
    A_left = A;
    D_left = D;
    TH_left = TH;
    
    parameters(0, 2); % Right robot
    AL_right = AL;
    A_right = A;
    D_right = D;
    TH_right = TH;
    
    % Initialize matrices to store transformations
    T_base_ee_left = cell(num_poses, 1);
    T_base_ee_right = cell(num_poses, 1);
    T_world_ee_left = cell(num_poses, 1);
    T_world_ee_right = cell(num_poses, 1);
    
    % Compute forward kinematics for each pose
    for i = 1:num_poses
        % Compute end-effector pose in base frame using forward kinematics
        T_base_ee_left{i}  = direct_kinematics_wrapper(q_left(i,:),  AL_left, A_left, D_left, TH_left, 1);
        T_base_ee_right{i} = direct_kinematics_wrapper(q_right(i,:), AL_right, A_right, D_right, TH_right, 2);
        
        % Convert world poses to homogeneous transformations
        % Assuming poses are in [x,y,z,rx,ry,rz] format where rx,ry,rz are Euler angles in XYZ convention
        pos_left = poses_left(i,1:3)
        rot_left = poses_left(i,4:6)
        tl =  eul2tform(rot_left, 'XYZ') * trvec2tform(pos_left)
        T_world_ee_left{i} = tl;
        
        pos_right = poses_right(i,1:3)
        rot_right = poses_right(i,4:6)
        tr =  eul2tform(rot_right, 'XYZ') * trvec2tform(pos_right)
        T_world_ee_right{i} = tr;
        disp("--------------------------")
    end
    
    % Solve AX=XB problem for each robot
    % For each robot, we have: T_world_base * T_base_ee = T_world_ee
    % So T_world_base = T_world_ee * inv(T_base_ee)
    
    % Initialize transformation matrices
    T_world_base_left_estimates  = cell(num_poses, 1);
    T_world_base_right_estimates = cell(num_poses, 1);
    
    for i = 1:num_poses
        a = T_world_ee_left{i}  / T_base_ee_left{i}
        T_world_base_left_estimates{i}  = a;
        b = T_world_ee_right{i} / T_base_ee_right{i}
        T_world_base_right_estimates{i} = b;
    end
    
    % Average the transformations to get a more robust estimate
    % First, extract translation and rotation components
    trans_left = zeros(num_poses, 3);
    trans_right = zeros(num_poses, 3);
    rot_left = zeros(num_poses, 3);
    rot_right = zeros(num_poses, 3);
    
    for i = 1:num_poses
        trans_left(i,:) = tform2trvec(T_world_base_left_estimates{i});
        trans_right(i,:) = tform2trvec(T_world_base_right_estimates{i});
        
        rot_left(i,:) = tform2eul(T_world_base_left_estimates{i}, 'XYZ');
        rot_right(i,:) = tform2eul(T_world_base_right_estimates{i}, 'XYZ');
    end
    
    % Average translations and rotations
    avg_trans_left = mean(trans_left, 1)
    avg_trans_right = mean(trans_right, 1)
    avg_rot_left = mean(rot_left, 1);
    avg_rot_right = mean(rot_right, 1);
    
    % Create final transformation matrices
    T_world_base_left = eul2tform(avg_rot_left, 'XYZ') * trvec2tform(avg_trans_left);
    T_world_base_right = eul2tform(avg_rot_right, 'XYZ') * trvec2tform(avg_trans_right);
    
    % Apply known constraints
    % 1. Z position should be table height + stand height
    % 2. Y distance between robots should be y_distance
    
    % Get current positions
    pos_left = tform2trvec(T_world_base_left);
    pos_right = tform2trvec(T_world_base_right);
    
    % Adjust Z position based on stand height
    tableHeight = 0.6; % From parameters.m
    pos_left(3) = tableHeight + stand_height;
    pos_right(3) = tableHeight + stand_height;
    
    % Adjust Y positions to maintain the known Y distance
    mid_y = (pos_left(2) + pos_right(2)) / 2;
    pos_left(2) = mid_y - y_distance/2;
    pos_right(2) = mid_y + y_distance/2;
    
    % Update transformations with adjusted positions
    T_world_base_left(1:3,4) = pos_left';
    T_world_base_right(1:3,4) = pos_right';
    
    % Print results
    fprintf('Left robot base transformation (world to base):\n');
    disp(T_world_base_left);
    fprintf('Position: [%.4f, %.4f, %.4f]\n', pos_left);
    fprintf('Orientation (XYZ Euler angles): [%.4f, %.4f, %.4f]\n', avg_rot_left);
    
    fprintf('\nRight robot base transformation (world to base):\n');
    disp(T_world_base_right);
    fprintf('Position: [%.4f, %.4f, %.4f]\n', pos_right);
    fprintf('Orientation (XYZ Euler angles): [%.4f, %.4f, %.4f]\n', avg_rot_right);
end