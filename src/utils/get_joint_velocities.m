function [q_dots] = get_joint_velocities( positions, velocities )
    params

    N = max(size( positions ));
    
    q_dots = zeros( N, 6 );
    for i=1:N
        Te          = direct_kinematics_cpp( positions(i,:), AL, A, D, TH );
        J           = Jacobian_cpp( Te, positions(i,:), AL, A, D, TH );
        q_dot       = J \ velocities(i,:)';
        q_dots(i,:) = q_dot';
    end

end
