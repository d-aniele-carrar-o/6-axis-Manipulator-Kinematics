function process_multiple_scenes(dirPath, tableParams, axs)
% PROCESS_MULTIPLE_SCENES Process all scenes in a directory and generate trajectories
%   This function processes all point cloud files in a directory, clusters objects
%   in each scene, computes grasp points, and generates robot trajectories
%
%   Input:
%       dirPath - Path to directory containing scene point cloud files
%       tableParams - Table parameters for calibration

    fprintf('\n===== STARTING FULL PIPELINE PROCESSING =====\n');
    fprintf('Directory: %s\n', dirPath);

    if nargin < 3
        % Setup dual robot environment (similar to dual_robot_setup.m)
        fprintf('\nSetting up dual robot environment...\n');
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

        % Create environment
        figure('Name', 'Dual Robot Workspace - Multiple Scenes');
        hold on; grid on;
        axs = create_environment(tablePosition, tableParams);
    
        % Initial joint configurations
        q0_left  = [pi/2, -pi/3, 2*pi/3, -pi/3, pi/2, 0];
        q0_right = [-pi/2, -2*pi/3, -2*pi/3, -2*pi/3, -pi/2, 0];
        
        % Set robot configurations
        config_left  = set_robot_configuration(q0_left, config_left);
        config_right = set_robot_configuration(q0_right, config_right);
        
        % Visualize robots
        show(robot_left, config_left, "Visuals", "on", "Frames", "on", "FastUpdate", true, "PreservePlot", false, "Parent", axs);
        show(robot_right, config_right, "Visuals", "on", "Frames", "on", "FastUpdate", true, "PreservePlot", false, "Parent", axs);
        
        % Compute end-effector poses
        parameters(1, 1); % Load parameters for robot 1
        [Te_w_e_left, ~] = direct_kinematics(q0_left, 1);
        
        parameters(1, 2); % Load parameters for robot 2
        [Te_w_e_right, ~] = direct_kinematics(q0_right, 2);
    
    end

    % Get all point cloud files in the directory
    files = dir(fullfile(dirPath, '*.ply'));
    numScenes = length(files);
    
    if numScenes == 0
        fprintf('\n===== ERROR: NO FILES FOUND =====\n');
        fprintf('No .ply files found in directory: %s\n', dirPath);
        return;
    end
    
    fprintf('\n===== FOUND %d SCENE FILES =====\n', numScenes);
    
    % Use default scene for calibration
    project_root = fileparts(fileparts(mfilename('fullpath')));
    calibScenePath = fullfile(project_root, 'output', 'segmented_objects', '25-06-07-11-21-29', 'table_surface.ply');
    
    if ~exist(calibScenePath, 'file')
        fprintf('\n===== ERROR: CALIBRATION FILE NOT FOUND =====\n');
        fprintf('Calibration file not found: %s\n', calibScenePath);
        return;
    end
    
    % Process each scene
    for i = 1:numScenes
        % Get full path to the scene file
        scenePath = fullfile(dirPath, files(i).name);
        fprintf('\n===== PROCESSING SCENE %d/%d =====\n', i, numScenes);
        fprintf('File: %s\n', files(i).name);
        
        try
            % Calibrate camera and transform scene point cloud
            fprintf('Calibrating scene...\n');
            [~, scenePC_world, ~] = calibrate_camera(calibScenePath, scenePath, tableParams, false);
            
            % Cluster objects in the scene
            fprintf('Clustering objects in scene...\n');
            objectPCs = cluster_scene_objects(scenePC_world);
            
            % Skip if no objects found
            if isempty(objectPCs)
                fprintf('No objects found in this scene. Skipping to next scene.\n');
                continue;
            end
            
            fprintf('Found %d objects in scene\n', length(objectPCs));
            
            % Process each object in the scene
            for j = 1:length(objectPCs)
                fprintf('\n----- Processing Object %d/%d -----\n', j, length(objectPCs));
                
                % Find grasp points for this object
                fprintf('Finding grasp points...\n');
                [grasp_points, grasp_orientations] = find_object_grasp_points(objectPCs{j}, Te_w_e_left, Te_w_e_right);
                
                % Skip if no valid grasp points found
                if isempty(grasp_points)
                    fprintf('No valid grasp points found. Skipping to next object.\n');
                    continue;
                end
                
                % Visualize the current object and grasp points
                fprintf('Visualizing object and grasp points...\n');
                figure('Name', sprintf('Scene %d - Object %d', i, j));
                hold on; grid on;
                axs_obj = gca;
                
                % Show object point cloud
                pcshow(objectPCs{j}.Location, 'r', 'MarkerSize', 30, 'Parent', axs_obj);
                
                % Show grasp points
                scatter3(grasp_points(1,1), grasp_points(2,1), grasp_points(3,1), 100, 'g*', 'Parent', axs_obj);
                scatter3(grasp_points(1,2), grasp_points(2,2), grasp_points(3,2), 100, 'g*', 'Parent', axs_obj);
                
                % Generate trajectories for this object
                fprintf('Generating robot trajectories...\n');
                
                % Create transformation matrices for grasp points
                R_grasp_l = squeeze(grasp_orientations(:,:,1));
                pf_l = grasp_points(:,1);
                approach_offset = 0.05;
                pf_l = pf_l - approach_offset * R_grasp_l(:,3);
                T_w_o_l = [R_grasp_l, pf_l; 0,0,0,1];
                Tf_l = Trf_0_l \ T_w_o_l;
                
                R_grasp_r = squeeze(grasp_orientations(:,:,2));
                pf_r = grasp_points(:,2);
                pf_r = pf_r - approach_offset * R_grasp_r(:,3);
                T_w_o_r = [R_grasp_r, pf_r; 0,0,0,1];
                Tf_r = Trf_0_r \ T_w_o_r;
                
                % Second viapoint: move to actual grasp positions
                pf2_l = grasp_points(:,1);
                T2_w_o_l = [R_grasp_l, pf2_l; 0,0,0,1];
                Tf2_l = Trf_0_l \ T2_w_o_l;
                
                pf2_r = grasp_points(:,2);
                T2_w_o_r = [R_grasp_r, pf2_r; 0,0,0,1];
                Tf2_r = Trf_0_r \ T2_w_o_r;
                
                % Third viapoint: lift object
                pf3_l = grasp_points(:,1) + [0; 0; 0.2];
                T3_w_o_l = [R_grasp_l, pf3_l; 0,0,0,1];
                Tf3_l = Trf_0_l \ T3_w_o_l;
                
                pf3_r = grasp_points(:,2) + [0; 0; 0.2];
                T3_w_o_r = [R_grasp_r, pf3_r; 0,0,0,1];
                Tf3_r = Trf_0_r \ T3_w_o_r;
                
                % Compute trajectories
                viapoints_l = [Tf_l; Tf2_l; Tf3_l];
                viapoints_r = [Tf_r; Tf2_r; Tf3_r];
                times = [2, 2, 2];
                
                % Generate trajectories
                [t_l, p_l, v_l] = multipoint_trajectory(q0_left, viapoints_l, times, 1);
                [t_r, p_r, v_r] = multipoint_trajectory(q0_right, viapoints_r, times, 2);
                
                % Simulate dual robot motion
                fprintf('Simulating robot trajectories...\n');
                [qf, ~] = simulate_dual({robot_left, robot_right}, {config_left, config_right}, ...
                                      {Trf_0_l, Trf_0_r}, {t_l, t_r}, {p_l, p_r}, {v_l, v_r}, axs);
                
                % Update initial configurations for next object
                q0_left = qf{1}(end,:);
                q0_right = qf{2}(end,:);
                
                % Wait for user input before proceeding to next object
                fprintf('\n===== OBJECT PROCESSING COMPLETE =====\n');
                fprintf('Press Enter to continue to next object...\n');
                pause()
            end
            
        catch e
            fprintf('\n===== ERROR PROCESSING SCENE %d =====\n', i);
            fprintf('Error: %s\n', e.message);
        end
    end
    
    fprintf('\n===== FULL PIPELINE PROCESSING COMPLETE =====\n');
end
