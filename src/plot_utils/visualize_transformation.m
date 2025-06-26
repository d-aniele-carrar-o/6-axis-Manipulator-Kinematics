function visualize_transformation(aug_id, ql_orig, qr_orig, ql_new, qr_new, ...
                                  object_data, interaction_points, keyframes_data)
% Advanced visualization with multiple objects and interaction points
    
    % Original trajectory window
    figure('Name', sprintf('Original Trajectory - Aug %d', aug_id), 'Position', [50, 50, 800, 600]);
    hold on; grid on; axis equal;
    title('Original Trajectory & Objects');
    
    axs = create_environment();
    colors = lines(size(object_data.original_centroids, 1));
    for i = 1:size(object_data.original_centroids, 1)
        scatter3(object_data.original_centroids(i,1), object_data.original_centroids(i,2), object_data.original_centroids(i,3), ...
            150, colors(i,:), 'o', 'filled');
    end
    
    plot_robot_trajectory(ql_orig, keyframes_data, 1, axs, 'r-', true, true);
    plot_robot_trajectory(qr_orig, keyframes_data, 2, axs, 'b-', true, true);
    
    [ee_posl, ~] = get_end_effector_trajectory(ql_orig, 1, true);
    [ee_posr, ~] = get_end_effector_trajectory(qr_orig, 2, true);
    
    for i = 1:length(interaction_points.left)
        idx = interaction_points.left(i);
        scatter3(ee_posl(idx,1), ee_posl(idx,2), ee_posl(idx,3), ...
            100, 'b', 's', 'filled', 'LineWidth', 2);
    end
    
    for i = 1:length(interaction_points.right)
        idx = interaction_points.right(i);
        scatter3(ee_posr(idx,1), ee_posr(idx,2), ee_posr(idx,3), ...
            100, 'r', 's', 'filled', 'LineWidth', 2);
    end
    
    xlim([-0.8, 0.8]); ylim([-0.8, 0.8]); zlim([0, 1.5]); view(45, 30);
    
    % Transformed trajectory window
    figure('Name', sprintf('Transformed Trajectory - Aug %d', aug_id), 'Position', [900, 50, 800, 600]);
    hold on; grid on; axis equal;
    title(sprintf('Transformed Trajectory (Aug %d)', aug_id));
    
    axs = create_environment();
    for i = 1:size(object_data.transformed_centroids, 1)
        scatter3(object_data.transformed_centroids(i,1), object_data.transformed_centroids(i,2), object_data.transformed_centroids(i,3), ...
            150, colors(i,:), 'o', 'filled');
        
        quiver3(object_data.original_centroids(i,1), object_data.original_centroids(i,2), object_data.original_centroids(i,3), ...
                object_data.transformed_centroids(i,1) - object_data.original_centroids(i,1), ...
                object_data.transformed_centroids(i,2) - object_data.original_centroids(i,2), ...
                object_data.transformed_centroids(i,3) - object_data.original_centroids(i,3), ...
                0, 'k-', 'LineWidth', 2, 'MaxHeadSize', 0.5);
    end
    
    plot_robot_trajectory(ql_new, keyframes_data, 1, axs, 'r-', true, true, true);
    plot_robot_trajectory(qr_new, keyframes_data, 2, axs, 'b-', true, true, true);
    
    xlim([-0.8, 0.8]); ylim([-0.8, 0.8]); zlim([0, 1.5]); view(45, 30);
    
    % 3D Keyframes window
    figure('Name', sprintf('3D Keyframes & Objects - Aug %d', aug_id), 'Position', [475, 350, 800, 600]);
    hold on; grid on; axis equal;
    title('3D Keyframes & Objects');
    
    create_environment();
    
    if isfield(object_data, 'original_pcs') && ~isempty(object_data.original_pcs)
        for i = 1:length(object_data.original_pcs)
            pcshow(object_data.original_pcs{i}.Location, [0.7 0.7 0.7], 'MarkerSize', 10);
        end
    end
    if isfield(object_data, 'transformed_pcs') && ~isempty(object_data.transformed_pcs)
        colors = lines(length(object_data.transformed_pcs));
        for i = 1:length(object_data.transformed_pcs)
            pcshow(object_data.transformed_pcs{i}.Location, colors(i,:), 'MarkerSize', 15);
        end
    end
    
    if isfield(keyframes_data, 'q_l') && isfield(keyframes_data, 'q_r')
        for i = 1:size(keyframes_data.q_l, 1)
            parameters(1, 1); [Twel_orig, ~] = direct_kinematics(keyframes_data.q_l(i,:), 1);
            parameters(1, 2); [Twer_orig, ~] = direct_kinematics(keyframes_data.q_r(i,:), 2);
            
            plotTransforms(Twel_orig(1:3,4)', rotm2quat(Twel_orig(1:3,1:3)), ...
                'FrameSize', 0.05, 'FrameColor', [0.5 0.5 0.5]);
            plotTransforms(Twer_orig(1:3,4)', rotm2quat(Twer_orig(1:3,1:3)), ...
                'FrameSize', 0.05, 'FrameColor', [0.5 0.5 0.5]);
        end
    end
    
    % Show original trajectories (grey)
    [ee_posl_orig, ~] = get_end_effector_trajectory(ql_orig, 1, true);
    [ee_posr_orig, ~] = get_end_effector_trajectory(qr_orig, 2, true);
    plot3(ee_posl_orig(:,1), ee_posl_orig(:,2), ee_posl_orig(:,3), 'Color', [0.5 0.5 0.5], 'LineWidth', 1);
    plot3(ee_posr_orig(:,1), ee_posr_orig(:,2), ee_posr_orig(:,3), 'Color', [0.5 0.5 0.5], 'LineWidth', 1);
    
    % Show new trajectories (colored)
    [ee_posl_new, ~] = get_end_effector_trajectory(ql_new, 1, true);
    [ee_posr_new, ~] = get_end_effector_trajectory(qr_new, 2, true);
    plot3(ee_posl_new(:,1), ee_posl_new(:,2), ee_posl_new(:,3), 'b-', 'LineWidth', 2);
    plot3(ee_posr_new(:,1), ee_posr_new(:,2), ee_posr_new(:,3), 'r-', 'LineWidth', 2);
    
    if isfield(keyframes_data, 'transformed_keyframes_l') && isfield(keyframes_data, 'transformed_keyframes_r')
        for i = 1:size(keyframes_data.transformed_keyframes_l, 1)
            if any(any(keyframes_data.transformed_keyframes_l(i,:,:)))
                T_l = squeeze(keyframes_data.transformed_keyframes_l(i,:,:));
                plotTransforms(T_l(1:3,4)', rotm2quat(T_l(1:3,1:3)), ...
                    'FrameSize', 0.08, 'FrameColor', [0 0 1]);
            end
        end
        for i = 1:size(keyframes_data.transformed_keyframes_r, 1)
            if any(any(keyframes_data.transformed_keyframes_r(i,:,:)))
                T_r = squeeze(keyframes_data.transformed_keyframes_r(i,:,:));
                plotTransforms(T_r(1:3,4)', rotm2quat(T_r(1:3,1:3)), ...
                    'FrameSize', 0.08, 'FrameColor', [1 0 0]);
            end
        end
    end
    
    xlim([-0.8, 0.8]); ylim([-0.8, 0.8]); zlim([0, 1.5]); view(45, 30);
end
