% Inverse kinematics for IRB 7600-400/2.55, 404 mm for LeanID
function [H] = ABB_inverse_kinematics( T06 )
    % load manipulator parameters
    params
    
    H = zeros( 8, 6 );

    % extract position and orientation from homogeneous matrix
    pe = T06(1:3,4);
    Re = T06(1:3,1:3);

    % theta 1 =========================================================
    z_6 = T06(1:3,3);
    T05 = T06 - [eye(3), D(7)*z_6; [0,0,0,1]];
    p05 = T05(1:3,4);
    
    % theta 1 = atan2( p05y, p05x )
    theta1_1 = round( atan2( p05(2), p05(1) ), 4 );
    theta1_2 = round( theta1_1 - pi, 4 );


    % theta 3 =========================================================
    % slightly diffenent from direct kinematic approach, in this case, T01 is frame 1 (always wrt frame 0)
    % but with different origin position: in this case, it coincides with position of origin of frame 2
    % (frame 1 from direct kin but also moved by a1) - at the end, we are computing the position of origin of Frame 2
    phi  = atan2( D(5), A(4) );
    d4_p = sqrt( A(4)^2 + D(5)^2 );
    
    % theta3 1&2 with theta1_1
    T01_1 = rot_traslZ( theta1_1, D(2) ) * rot_tralsX( AL(1), A(2) );
    p15_1 = inv(T01_1)*[p05; 1];
    
    F_2_1       = p15_1(1)^2 + p15_1(3)^2;
    cos_gamma_1 = (F_2_1 - A(3)^2 - d4_p^2) / (-2*A(3)*d4_p);
    % check if the position is reachable: |cos(gamma)| <= 1
    % if not, the configuration with this theta1 is not feasible
    impossible_conf_1 = abs(cos_gamma_1) > 1;
    if impossible_conf_1
        gamma_1   = nan;
        theta3_11 = nan;
        theta3_12 = nan;
    else
        gamma_1     = atan2( sqrt(1-cos_gamma_1^2), cos_gamma_1 );
        theta3_11 = round(  gamma_1 + phi - pi, 4 );
        theta3_12 = round( -gamma_1 + phi - pi, 4 );
    end

    % theta3 3&4 with theta1_2
    T01_2 = rot_traslZ( theta1_2, D(2) ) * rot_tralsX( AL(1), A(2) );
    p15_2 = inv(T01_2)*[p05; 1];

    F_2_2       = p15_2(1)^2 + p15_2(3)^2;
    cos_gamma_2 = (F_2_2 - A(3)^2 - d4_p^2) / (-2*A(3)*d4_p);
    % check if the position is reachable: |cos(gamma)| <= 1
    % if not, the configuration with this theta1 is not feasible
    impossible_conf_2 = abs(cos_gamma_2) > 1;
    if impossible_conf_2
        gamma_2   = nan;
        theta3_21 = nan;
        theta3_22 = nan;
    else
        gamma_2     = atan2( sqrt(1-cos_gamma_2^2), cos_gamma_2 );
        theta3_21 = round(  gamma_2 + phi - pi, 4 );
        theta3_22 = round( -gamma_2 + phi - pi, 4 );
    end

    % theta 2 =========================================================
    
    % theta2 1&2 with theta1_1
    if ~impossible_conf_1
        beta_1      = pi/2 - atan2( p15_1(1), p15_1(3) );
        sin_alpha_1 = d4_p*sin( gamma_1 )/sqrt(F_2_1);
        alpha_1     = atan2( sin_alpha_1, sqrt( 1-sin_alpha_1^2 ) );
        
        theta2_11   = round(  alpha_1 + beta_1, 4 );
        theta2_12   = round( -alpha_1 + beta_1, 4 );
    else
        theta2_11 = nan;
        theta2_12 = nan;
    end

    % theta2 3&4 with theta1_2
    if ~impossible_conf_2
        beta_2      = pi/2 - atan2( p15_2(1), p15_2(3) );
        sin_alpha_2 = d4_p*sin( gamma_2 )/sqrt(F_2_2);
        alpha_2     = atan2( sin_alpha_2, sqrt( 1-sin_alpha_2^2 ) );
        
        theta2_21   = round(  alpha_2 + beta_2, 4 );
        theta2_22   = round( -alpha_2 + beta_2, 4 );
    else
        theta2_21 = nan;
        theta2_22 = nan;
    end
    

    % theta 5 =========================================================
    % cos(theta5) = -y3 (dot) z6
    
    if ~impossible_conf_1
        T03_111 = transf_i_1_i( 1, theta1_1 ) * transf_i_1_i( 2, theta2_11 ) * transf_i_1_i( 3, theta3_11 );
        T03_112 = transf_i_1_i( 1, theta1_1 ) * transf_i_1_i( 2, theta2_11 ) * transf_i_1_i( 3, theta3_11 );
        T03_121 = transf_i_1_i( 1, theta1_1 ) * transf_i_1_i( 2, theta2_12 ) * transf_i_1_i( 3, theta3_12 );
        T03_122 = transf_i_1_i( 1, theta1_1 ) * transf_i_1_i( 2, theta2_12 ) * transf_i_1_i( 3, theta3_12 );
        
        y_3m_111 = -T03_111(1:3,2);
        y_3m_112 = -T03_112(1:3,2);
        y_3m_121 = -T03_121(1:3,2);
        y_3m_122 = -T03_122(1:3,2);
    
        cos_th5_111 = dot( y_3m_111, z_6 );
        cos_th5_112 = dot( y_3m_112, z_6 );
        cos_th5_121 = dot( y_3m_121, z_6 );
        cos_th5_122 = dot( y_3m_122, z_6 );
    
        theta5_111 = round( atan2(  sqrt(1-cos_th5_111^2), cos_th5_111 ), 4 );
        theta5_112 = round( atan2( -sqrt(1-cos_th5_112^2), cos_th5_112 ), 4 );
        theta5_121 = round( atan2(  sqrt(1-cos_th5_121^2), cos_th5_121 ), 4 );
        theta5_122 = round( atan2( -sqrt(1-cos_th5_122^2), cos_th5_122 ), 4 );
    else
        theta5_111 = nan;
        theta5_112 = nan;
        theta5_121 = nan;
        theta5_122 = nan;
    end
    
    if ~impossible_conf_2
        T03_211 = transf_i_1_i( 1, theta1_2 ) * transf_i_1_i( 2, theta2_21 ) * transf_i_1_i( 3, theta3_21 );
        T03_212 = transf_i_1_i( 1, theta1_2 ) * transf_i_1_i( 2, theta2_21 ) * transf_i_1_i( 3, theta3_21 );
        T03_221 = transf_i_1_i( 1, theta1_2 ) * transf_i_1_i( 2, theta2_22 ) * transf_i_1_i( 3, theta3_22 );
        T03_222 = transf_i_1_i( 1, theta1_2 ) * transf_i_1_i( 2, theta2_22 ) * transf_i_1_i( 3, theta3_22 );
        
        y_3m_211 = -T03_211(1:3,2);
        y_3m_212 = -T03_212(1:3,2);
        y_3m_221 = -T03_221(1:3,2);
        y_3m_222 = -T03_222(1:3,2);
    
        cos_th5_211 = dot( y_3m_211, z_6 );
        cos_th5_212 = dot( y_3m_212, z_6 );
        cos_th5_221 = dot( y_3m_221, z_6 );
        cos_th5_222 = dot( y_3m_222, z_6 );
        
        theta5_211 = round( atan2(  sqrt(1-cos_th5_211^2), cos_th5_211 ), 4 );
        theta5_212 = round( atan2( -sqrt(1-cos_th5_212^2), cos_th5_212 ), 4 );
        theta5_221 = round( atan2(  sqrt(1-cos_th5_221^2), cos_th5_221 ), 4 );
        theta5_222 = round( atan2( -sqrt(1-cos_th5_222^2), cos_th5_222 ), 4 );
    else
        theta5_211 = nan;
        theta5_212 = nan;
        theta5_221 = nan;
        theta5_222 = nan;
    end

    % theta 6 =========================================================
    if ~impossible_conf_1
        T36_111 = inv( T03_111 ) * T06;
        T36_112 = inv( T03_112 ) * T06;
        T36_121 = inv( T03_121 ) * T06;
        T36_122 = inv( T03_122 ) * T06;
    
        if theta5_111 ~= 0
            theta6_111 = round( atan2( T36_111(2,2)/sin(theta5_111), -T36_111(2,1)/sin(theta5_111) ), 4 ) - pi;
            theta4_111 = round( atan2( T36_111(3,3)/sin(theta5_111),  T36_111(1,3)/sin(theta5_111) ), 4 );
        else
            % [theta6_111, theta4_111] = compute_th4_th6( T36_111(3,1), T36_111(3,2), theta5_111 );
            theta6_111 = nan;
            theta4_111 = nan;
        end
        if theta5_112 ~= 0
            theta6_112 = round( atan2( T36_112(2,2)/sin(theta5_112), -T36_112(2,1)/sin(theta5_112) ), 4 ) - pi;
            theta4_112 = round( atan2( T36_112(3,3)/sin(theta5_112),  T36_112(1,3)/sin(theta5_112) ), 4 );
        else
            % [theta6_112, theta4_112] = compute_th4_th6( T36_112(3,1), T36_112(3,2), theta5_112 );
            theta6_112 = nan;
            theta4_112 = nan;
        end
        if theta5_121 ~= 0
            theta6_121 = round( atan2( T36_121(2,2)/sin(theta5_121), -T36_121(2,1)/sin(theta5_121) ), 4 ) - pi;
            theta4_121 = round( atan2( T36_121(3,3)/sin(theta5_121),  T36_121(1,3)/sin(theta5_121) ), 4 );
        else
            % [theta6_121, theta4_121] = compute_th4_th6( T36_121(3,1), T36_121(3,2), theta5_121 );
            theta6_121 = nan;
            theta4_121 = nan;
        end
        if theta5_122 ~= 0
            theta6_122 = round( atan2( T36_122(2,2)/sin(theta5_122), -T36_122(2,1)/sin(theta5_122) ), 4 ) - pi;
            theta4_122 = round( atan2( T36_122(3,3)/sin(theta5_122),  T36_122(1,3)/sin(theta5_122) ), 4 );
        else
            % [theta6_122, theta4_122] = compute_th4_th6( T36_122(3,1), T36_122(3,2), theta5_122 );
            theta6_122 = nan;
            theta4_122 = nan;
        end
    else
        theta4_111 = nan;
        theta4_112 = nan;
        theta4_121 = nan;
        theta4_122 = nan;
        theta6_111 = nan;
        theta6_112 = nan;
        theta6_121 = nan;
        theta6_122 = nan;
    end

    if ~impossible_conf_2
        T36_211 = inv( T03_211 ) * T06;
        T36_212 = inv( T03_212 ) * T06;
        T36_221 = inv( T03_221 ) * T06;
        T36_222 = inv( T03_222 ) * T06;
    
        if theta5_211 ~= 0
            theta6_211 = round( atan2( T36_211(2,2)/sin(theta5_211), -T36_211(2,1)/sin(theta5_211) ), 4 ) - pi;
            theta4_211 = round( atan2( T36_211(3,3)/sin(theta5_211),  T36_211(1,3)/sin(theta5_211) ), 4 );
        else
            % [theta6_211, theta4_211] = compute_th4_th6( T36_211(3,1), T36_211(3,2), theta5_211 );
            theta6_211 = nan;
            theta4_211 = nan;
        end
        if theta5_212 ~= 0
            theta6_212 = round( atan2( T36_212(2,2)/sin(theta5_212), -T36_212(2,1)/sin(theta5_212) ), 4 ) - pi;
            theta4_212 = round( atan2( T36_212(3,3)/sin(theta5_212),  T36_212(1,3)/sin(theta5_212) ), 4 );
        else
            % [theta6_212, theta4_212] = compute_th4_th6( T36_212(3,1), T36_212(3,2), theta5_212 );
            theta6_212 = nan;
            theta4_212 = nan;
        end
        if theta5_221 ~= 0
            theta6_221 = round( atan2( T36_221(2,2)/sin(theta5_221), -T36_221(2,1)/sin(theta5_221) ), 4 ) - pi;
            theta4_221 = round( atan2( T36_221(3,3)/sin(theta5_221),  T36_221(1,3)/sin(theta5_221) ), 4 );
        else
            % [theta6_221, theta4_221] = compute_th4_th6( T36_221(3,1), T36_221(3,2), theta5_221 );
            theta6_221 = nan;
            theta4_221 = nan;
        end
        if theta5_222 ~= 0
            theta6_222 = round( atan2( T36_222(2,2)/sin(theta5_222), -T36_222(2,1)/sin(theta5_222) ), 4 ) - pi;
            theta4_222 = round( atan2( T36_222(3,3)/sin(theta5_222),  T36_222(1,3)/sin(theta5_222) ), 4 );
        else
            % [theta6_222, theta4_222] = compute_th4_th6( T36_222(3,1), T36_222(3,2), theta5_222 );
            theta6_222 = nan;
            theta4_222 = nan;
        end
    else
        theta4_211 = nan;
        theta4_212 = nan;
        theta4_221 = nan;
        theta4_222 = nan;
        theta6_211 = nan;
        theta6_212 = nan;
        theta6_221 = nan;
        theta6_222 = nan;
    end

    H(1, :) = [theta1_1, theta2_11, theta3_11, theta4_111, theta5_111, theta6_111];
    H(2, :) = [theta1_1, theta2_11, theta3_11, theta4_112, theta5_112, theta6_112];
    H(3, :) = [theta1_1, theta2_12, theta3_12, theta4_121, theta5_121, theta6_121];
    H(4, :) = [theta1_1, theta2_12, theta3_12, theta4_122, theta5_122, theta6_122];
    H(5, :) = [theta1_2, theta2_21, theta3_21, theta4_211, theta5_211, theta6_211];
    H(6, :) = [theta1_2, theta2_21, theta3_21, theta4_212, theta5_212, theta6_212];
    H(7, :) = [theta1_2, theta2_22, theta3_22, theta4_221, theta5_221, theta6_221];
    H(8, :) = [theta1_2, theta2_22, theta3_22, theta4_222, theta5_222, theta6_222];

end
