function [q] = generate_line_path( q_i, T_f, N )
    T_i = UR5_direct_kinematics( q_i );
    
    p_i = T_i(1:3,4);
    p_f = T_f(1:3,4);

    R_i = T_i(1:3,1:3);
    R_f = T_f(1:3,1:3);

    path = lerp_path( p_i, p_f, N );
    
    orient = slerp( R_i, R_f, N );
    q      = zeros( N, 6 );
    curr_q = q_i;

    for i=1:N
        H_d = UR5_inverse_kinematics( path(:,i), orient(:,:,i) );
        q_d = get_closer( H_d, curr_q );
        curr_q = q_d;
        q(i,:) = q_d;
    end

end
