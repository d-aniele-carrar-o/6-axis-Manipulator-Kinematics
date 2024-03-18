function [positions, velocities] = joint2task_space( qs, q_dots )
    parameters(1)

    N          = size( qs, 1 );

    % Preallocate final vectors
    positions  = zeros( 4*N, 4 );
    velocities = zeros( N, 6 );

    
    for i=1:N
        % Compute end-effector pose via DK
        Te                         = direct_kinematics_cpp( qs(i,:), AL, A, D, TH );
        positions(4*(i-1)+1:4*i,:) = Trf_0*Te;
        % Compute end-effector velocity via Jacobian
        J                          = Jacobian_cpp( Te, qs(i,:), AL, A, D, TH );
        velocities(i,:)            = (J * q_dots(i,:)')';

    end
    
end

