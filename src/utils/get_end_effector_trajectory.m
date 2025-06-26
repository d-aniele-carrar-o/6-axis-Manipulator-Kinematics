function [ee_pos, ee_rot] = get_end_effector_trajectory(q_trajectory, robot_id, world_rf)
% Get end-effector positions for trajectory
    
    if nargin < 3
        disp("[get_end_effector_trajectory] WARNING: no reference frame selected, setting it to world frame");
        world_rf = true;
    end
    ee_pos = zeros(size(q_trajectory, 1), 3);
    ee_rot = zeros(size(q_trajectory, 1), 3, 3);

    for i = 1:size(q_trajectory, 1)
        if world_rf
            [T_w_e, ~] = direct_kinematics(q_trajectory(i,:), robot_id);
            ee_pos(i,:) = T_w_e(1:3,4)';
            ee_rot(i,:,:) = T_w_e(1:3,1:3);
        else
            [~, T_e] = direct_kinematics(q_trajectory(i,:), robot_id);
            ee_pos(i,:) = T_e(1:3,4)';
            ee_rot(i,:,:) = T_e(1:3,1:3);
        end
    end
end
