function config = set_robot_configuration( config, robot_q, gripper_q )
    q = [robot_q, gripper_q];

    for i=1:8
        config(i).JointPosition = q(i);
    end
end
