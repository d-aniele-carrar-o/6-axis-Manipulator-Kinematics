function simulate_inference()
% Script to simulate neural network inference actions with ground truth comparison
% Parameters:
%   timestamp - timestamp for ground truth data (e.g., '25-06-24-21-38-38')
%   inference_file - path to inference .mat file (e.g., 'pred_actions.mat')
%   debug_gt - (optional) flag to simulate ground truth actions for debug (default: false)

    debug_gt = true;
    
    clc; close all;

    % Setup robots
    parameters(0, 1);
    robot_left = robot; config_left = config; Trf_0_l = Trf_0;
    parameters(0, 2);
    robot_right = robot; config_right = config; Trf_0_r = Trf_0;

    % Create environment
    figure('Name', 'Neural Network Inference Simulation', 'Position', [100, 100, 1200, 800]);
    hold on; grid on; axis equal;
    axs = create_environment();

    % Load ground truth demonstration
    timestamp = '25-06-24-21-38-38';
    data_root = '/Users/danielecarraro/Documents/VSCODE/data/output/yoto/';
    task_name = 'liftbox';
    gt_file = sprintf('%s%s_%s.json', data_root, timestamp, task_name);
    if ~exist(gt_file, 'file')
        error('Ground truth file not found: %s', gt_file);
    end
    fprintf('Loading ground truth from: %s\n', gt_file);
    gt_data = jsondecode(fileread(gt_file));

    % Load inference data
    inference_file = sprintf('%s%s_pred_actions.mat', data_root, timestamp);
    if ~exist(inference_file, 'file')
        error('Inference file not found: %s', inference_file);
    end
    fprintf('Loading inference from: %s\n', inference_file);
    data = load(inference_file);
    poses     = data.poses;             % (N, 4, 4) transformation matrices
    grippers  = data.grippers;          % (N, 1) gripper states
    arm_order = data.arm_order;         % Cell array of 'L'/'R'
    scene_pc  = data.scene_pointcloud;  % (N, 3) point cloud

    % Display scene pointcloud from inference
    if ~isempty(scene_pc)
        fprintf('Displaying inference scene pointcloud\n');
        pcshow(scene_pc, [0.5 0.5 0.5], 'MarkerSize', 15, 'Parent', axs);
    end

    % Display original scene pointcloud from ground truth
    if ~isempty(gt_data) && isfield(gt_data(1), 'obs') && isfield(gt_data(1).obs, 'pcl')
        fprintf('Displaying ground truth scene pointcloud\n');
        gt_pc = gt_data(1).obs.pcl;
        pcshow(gt_pc, [0.8 0.8 0.8], 'MarkerSize', 10, 'Parent', axs);
    end

    % Initial configurations
    q0_left  = [pi/2, -pi/3, 2*pi/3, -pi/3, pi/2, 0];
    q0_right = [-pi/2, -2*pi/3, -2*pi/3, -2*pi/3, -pi/2, 0];

    % Plot ground truth keyframes in grey
    if ~isempty(gt_data) && isfield(gt_data(1), 'actions')
        fprintf('Plotting ground truth keyframes\n');
        plot_gt_keyframes(gt_data(1).actions, axs);
    end

    % Plot inference keyframes in colors
    fprintf('Plotting inference keyframes\n');
    plot_inference_keyframes(poses, arm_order, axs);

    % Show initial robot configurations
    config_left  = set_robot_configuration(q0_left, config_left);
    config_right = set_robot_configuration(q0_right, config_right);
    show(robot_left, config_left, "Visuals", "on", "Frames", "off", "FastUpdate", true, "PreservePlot", false, "Parent", axs);
    show(robot_right, config_right, "Visuals", "on", "Frames", "off", "FastUpdate", true, "PreservePlot", false, "Parent", axs);

    % Add labels
    title('Neural Network Inference vs Ground Truth', 'FontSize', 14);
    xlabel('X (m)'); ylabel('Y (m)'); zlabel('Z (m)');
    view(45, 30);

    % Generate trajectories from inference keyframes
    fprintf('Generating trajectories from inference keyframes\n');
    [q_left_traj, q_right_traj] = generate_inference_trajectories(poses, arm_order, grippers, q0_left, q0_right);

    % Simulate trajectories
    if debug_gt && ~isempty(gt_data) && isfield(gt_data(1), 'actions')
        fprintf('Starting ground truth simulation for debug...\n');
        pause();
        [q_gt_left, q_gt_right] = generate_gt_trajectories(gt_data(1).actions, q0_left, q0_right);
        simulate_trajectories(robot_left, robot_right, config_left, config_right, q_gt_left, q_gt_right, axs);
        
        fprintf('Ground truth simulation completed. Press key for inference simulation...\n');
        pause();
    end
    
    fprintf('Starting inference simulation, press a key to continue...\n');
    pause();
    simulate_trajectories(robot_left, robot_right, config_left, config_right, ...
                          q_left_traj, q_right_traj, axs);

    fprintf('Inference simulation completed\n');
end

function plot_gt_keyframes(actions, axs)
% Plot ground truth keyframes in grey with connecting lines
    left_poses = []; right_poses = [];

    % Plot left arm keyframes
    if isfield(actions, 'left')
        for i = 1:length(actions.left)
            pos = actions.left(i).trans;
            scatter3(pos(1), pos(2), pos(3), 80, [0.7 0.7 0.7], 'filled', 'Parent', axs);
            left_poses = [left_poses; pos'];
        end
    end
    
    % Plot right arm keyframes
    if isfield(actions, 'right')
        for i = 1:length(actions.right)
            pos = actions.right(i).trans;
            scatter3(pos(1), pos(2), pos(3), 80, [0.7 0.7 0.7], 'filled', 'Parent', axs);
            right_poses = [right_poses; pos'];
        end
    end

    % Draw connecting lines for each arm
    if size(left_poses, 1) > 1
        plot3(left_poses(:,1), left_poses(:,2), left_poses(:,3), '--', 'Color', [0.7 0.7 0.7], 'LineWidth', 2, 'Parent', axs);
    end
    if size(right_poses, 1) > 1
        plot3(right_poses(:,1), right_poses(:,2), right_poses(:,3), '--', 'Color', [0.7 0.7 0.7], 'LineWidth', 2, 'Parent', axs);
    end
end

function plot_inference_keyframes(poses, arm_order, axs)
% Plot inference keyframes in colors with connecting lines
    left_poses = []; right_poses = [];

    for i = 1:size(poses, 1)
        pose = squeeze(poses(i, :, :));
        pos = pose(1:3, 4);
        
        % Handle different arm_order formats
        if iscell(arm_order)
            arm_char = arm_order{i};
        elseif ischar(arm_order) || isstring(arm_order)
            arm_char = arm_order(i);
        else
            arm_char = char(arm_order(i));
        end
        
        % Color based on arm
        if strcmp(arm_char, 'L')
            color = 'r';
            left_poses = [left_poses; pos'];
        else
            color = 'b';
            right_poses = [right_poses; pos'];
        end
        
        % Plot keyframe position
        scatter3(pos(1), pos(2), pos(3), 100, color, 'filled', 'Parent', axs);
    end

    % Draw connecting lines for each arm
    if size(left_poses, 1) > 1
        plot3(left_poses(:,1), left_poses(:,2), left_poses(:,3), 'r-', 'LineWidth', 3, 'Parent', axs);
    end
    if size(right_poses, 1) > 1
        plot3(right_poses(:,1), right_poses(:,2), right_poses(:,3), 'b-', 'LineWidth', 3, 'Parent', axs);
    end
end

function [qs_l, qs_r] = generate_inference_trajectories(poses, arm_order, grippers, q0_left, q0_right)
% Generate joint trajectories from inference keyframes
    
    N = length(poses);

    left_poses  = zeros(N/2, 4, 4); right_poses = zeros(N/2, 4, 4);
    left_times  = zeros(N/2, 1);    right_times = zeros(N/2, 1);

    % Get world-base tf for both robots
    parameters(1, 1); Trf_0_l = Trf_0;
    parameters(1, 2); Trf_0_r = Trf_0;

    % Separate poses by arm
    lidx = 1; ridx = 1;
    for i = 1:size(poses, 1)
        % Extract arm value
        arm_char = arm_order(i);
        
        % Extract current pose (wrt world frame)
        Tw_ee = squeeze(poses(i,:,:));
        if strcmp(arm_char, 'L')
            % Transform poses from world to robot base rf
            Tl = Trf_0_l \ Tw_ee;
            left_poses(lidx,:,:) = Tl;
            left_times(lidx) = 2; % 2 seconds per segment
            lidx = lidx + 1;
        else
            % Transform poses from world to robot base rf
            Tr = Trf_0_r \ Tw_ee;
            right_poses(ridx,:,:) = Tr;
            right_times(ridx) = 2; % 2 seconds per segment
            ridx = ridx + 1;
        end

    end

    % Generate trajectories using multipoint_trajectory
    [~, traj_l, vs_l] = multipoint_trajectory(q0_left,  left_poses,  left_times,  1);
    [~, traj_r, vs_r] = multipoint_trajectory(q0_right, right_poses, right_times, 2);

    % Ensure trajectories have same length
    if length(traj_l) ~= length(traj_r)
        error('Trajectories left/right appear to have different lengths: left: %d vs right: %d', ...
                length(traj_l), length(traj_r))
    end

    % Transform the end-effector poses using IK of selected manipulator to joint space configurations
    [qs_l, ~] = task2joint_space( q0_left,  traj_l, vs_l );
    [qs_r, ~] = task2joint_space( q0_right, traj_r, vs_r );

end

function [q_gt_left, q_gt_right] = generate_gt_trajectories(actions, q0_left, q0_right)
% Generate joint trajectories from ground truth keyframes
    
    Nl = length(actions.left);
    Nr = length(actions.right);
    left_poses = zeros(Nl, 4, 4); right_poses = zeros(Nr, 4, 4);
    left_times = zeros(Nl, 1);    right_times = zeros(Nr, 1);
    
    % Get world-base tf for both robots
    parameters(1, 1); Trf_0_l = Trf_0;
    parameters(1, 2); Trf_0_r = Trf_0;
    
    % Process left arm actions
    for i = 1:Nl
        trans = actions.left(i).trans;
        rot = actions.left(i).rot;
        angle = norm(rot);
        if angle > 0
            axis = rot / angle;
            R = axang2rotm([axis', angle]);
        else
            R = eye(3);
        end
        pose = [R, trans; 0, 0, 0, 1];
        Tl = Trf_0_l \ pose;
        left_poses(i,:,:) = Tl;
        left_times(i) = 2;
    end
    
    % Process right arm actions
    for i = 1:Nr
        trans = actions.right(i).trans;
        rot = actions.right(i).rot;
        angle = norm(rot);
        if angle > 0
            axis = rot / angle;
            R = axang2rotm([axis', angle]);
        else
            R = eye(3);
        end
        pose = [R, trans; 0, 0, 0, 1];
        Tr = Trf_0_r \ pose;
        right_poses(i,:,:) = Tr;
        right_times(i) = 2;
    end
    
    % Generate trajectories
    [~, traj_l, vs_l] = multipoint_trajectory(q0_left,  left_poses,  left_times,  1);
    [~, traj_r, vs_r] = multipoint_trajectory(q0_right, right_poses, right_times, 2);
    
    % Ensure trajectories have same length
    if length(traj_l) ~= length(traj_r)
        error('Trajectories left/right appear to have different lengths: left: %d vs right: %d', ...
                length(traj_l), length(traj_r))
    end

    [q_gt_left,  ~] = task2joint_space(q0_left,  traj_l, vs_l);
    [q_gt_right, ~] = task2joint_space(q0_right, traj_r, vs_r);
end

function simulate_trajectories(robot_left, robot_right, config_left, config_right, q_left_traj, q_right_traj, axs)
% Simulate trajectories
    for i = 1:size(q_left_traj, 1)
        config_left  = set_robot_configuration(q_left_traj(i,:), config_left);
        config_right = set_robot_configuration(q_right_traj(i,:), config_right);
        
        show(robot_left, config_left, "Visuals", "on", "Frames", "off", "FastUpdate", true, "PreservePlot", false, "Parent", axs);
        show(robot_right, config_right, "Visuals", "on", "Frames", "off", "FastUpdate", true, "PreservePlot", false, "Parent", axs);
        
        title(sprintf('Inference Simulation - Frame %d/%d', i, size(q_left_traj, 1)), 'FontSize', 14);
        pause(0.05);
        
        if ~ishandle(axs.Parent)
            break;
        end
    end
end
