%Inverse Kineamtics of UR5
function [Th] = UR5_inverse_kinematics( p06, R06 )
    fprintf("UR5_inverse_kinematics - start\n")
    params
    
    fprintf("UR5_inverse_kinematics - 1\n")
    T06 = [R06, p06; zeros(1,3), 1];
    
    % Finding th1 ==============================================================
    p05      = T06*[0, 0, -D(7), 1]';
    phi1     = atan2(p05(2), p05(1));
    cos_phi2 = D(5) / hypot(p05(2), p05(1));
    phi2     = atan2(sqrt(1-cos_phi2^2), cos_phi2);

    th1_1 = round( phi1 - phi2 + pi/2, 6 );
    th1_2 = round( phi1 + phi2 + pi/2, 6 );
    
    fprintf("UR5_inverse_kinematics - 2\n")
    % Finding th5 ==============================================================
    cos_th5_1 = (p06(1)*sin(th1_1) - p06(2)*cos(th1_1) - D(5)) / D(7);
    cos_th5_2 = (p06(1)*sin(th1_2) - p06(2)*cos(th1_2) - D(5)) / D(7);
    
    if round( cos_th5_1, 6 ) == 1
        th5_1 = 0;
        th5_2 = 0;
    else
        th5_1 = atan2( sqrt(1 - cos_th5_1^2), cos_th5_1 );
        th5_2 = -th5_1;
    end
    if round( cos_th5_2, 6 ) == 1
        th5_3 = 0;
        th5_4 = 0;
    else
        th5_3 = atan2( sqrt(1 - cos_th5_2^2), cos_th5_2 );
        th5_4 = -th5_3;
    end

    fprintf("UR5_inverse_kinematics - 3\n")
    % finding th6 ==============================================================
    % related to th11 and th51
    T60  = inv(T06);
    Xhat = T60(1:3,1);
    Yhat = T60(1:3,2);
    
    sin_th6_1 = (-Xhat(2)*sin(th1_1) + Yhat(2)*cos(th1_1)) / sin(th5_1);
    cos_th6_1 = ( Xhat(1)*sin(th1_1) - Yhat(1)*cos(th1_1)) / sin(th5_1);
    sin_th6_2 = (-Xhat(2)*sin(th1_1) + Yhat(2)*cos(th1_1)) / sin(th5_2);
    cos_th6_2 = ( Xhat(1)*sin(th1_1) - Yhat(1)*cos(th1_1)) / sin(th5_2);
    
    sin_th6_3 = (-Xhat(2)*sin(th1_2) + Yhat(2)*cos(th1_2)) / sin(th5_3);
    cos_th6_3 = ( Xhat(1)*sin(th1_2) - Yhat(1)*cos(th1_2)) / sin(th5_3);
    sin_th6_4 = (-Xhat(2)*sin(th1_2) + Yhat(2)*cos(th1_2)) / sin(th5_4);
    cos_th6_4 = ( Xhat(1)*sin(th1_2) - Yhat(1)*cos(th1_2)) / sin(th5_4);
    
    th6_1 = atan2( sin_th6_1, cos_th6_1 );
    th6_2 = atan2( sin_th6_2, cos_th6_2 );
    th6_3 = atan2( sin_th6_3, cos_th6_3 );
    th6_4 = atan2( sin_th6_4, cos_th6_4 );
    
    fprintf("UR5_inverse_kinematics - 4\n")
    % finding th3 ================================================
    T56_1 = transf_i_1_i(5,th5_1) * transf_i_1_i(6,th6_1);
    T56_2 = transf_i_1_i(5,th5_2) * transf_i_1_i(6,th6_2);
    T56_3 = transf_i_1_i(5,th5_3) * transf_i_1_i(6,th6_3);
    T56_4 = transf_i_1_i(5,th5_4) * transf_i_1_i(6,th6_4);

    T01_1 = transf_i_1_i(1,th1_1);
    T01_2 = transf_i_1_i(1,th1_2);
    
    T14     = T01_1 \ T06 / T56_1;
    p41_1   = T14(1:3,4);
    p41xz_1 = hypot( p41_1(1), p41_1(3) );
    
    T14     = T01_1 \ T06 / T56_2;
    p41_2   = T14(1:3,4);
    p41xz_2 = hypot( p41_2(1), p41_2(3) );
    
    T14     = T01_2 \ T06 / T56_3;
    p41_3   = T14(1:3,4);
    p41xz_3 = hypot( p41_3(1), p41_3(3) );
    
    T14     = T01_2 \ T06 / T56_4;
    p41_4   = T14(1:3,4);
    p41xz_4 = hypot( p41_4(1), p41_4(3) );
    
    cos_th3_1 = -(A(3)^2 + A(4)^2 - p41xz_1^2) / (2*A(3)*A(4));
    cos_th3_2 = -(A(3)^2 + A(4)^2 - p41xz_2^2) / (2*A(3)*A(4));
    cos_th3_3 = -(A(3)^2 + A(4)^2 - p41xz_3^2) / (2*A(3)*A(4));
    cos_th3_4 = -(A(3)^2 + A(4)^2 - p41xz_4^2) / (2*A(3)*A(4));

    % computation of the 8 possible values for th3    
    th3_1 = -atan2( sqrt(1-cos_th3_1^2), cos_th3_1 );
    th3_2 = -atan2( sqrt(1-cos_th3_2^2), cos_th3_2 );
    th3_3 = -atan2( sqrt(1-cos_th3_3^2), cos_th3_3 );
    th3_4 = -atan2( sqrt(1-cos_th3_4^2), cos_th3_4 );
    
    th3_5 = -th3_1;
    th3_6 = -th3_2;
    th3_7 = -th3_3;
    th3_8 = -th3_4;
    
    fprintf("UR5_inverse_kinematics - 6\n")
    % finding th2 ==============================================================
    % computation of 8 possible values for th2
    phi_11    = atan2( -p41_1(3), -p41_1(1) );
    sin_phi21 = (-A(4)*sin(th3_1)) / p41xz_1;
    phi_21    = atan2( sin_phi21, sqrt(1 - sin_phi21^2) );

    phi_12    = atan2( -p41_2(3), -p41_2(1) );
    sin_phi22 = (-A(4)*sin(th3_2)) / p41xz_2;
    phi_22    = atan2( sin_phi22, sqrt(1 - sin_phi22^2) );

    phi_13    = atan2( -p41_3(3), -p41_3(1) );
    sin_phi23 = (-A(4)*sin(th3_3)) / p41xz_3;
    phi_23    = atan2( sin_phi23, sqrt(1 - sin_phi23^2) );

    phi_14    = atan2( -p41_4(3), -p41_4(1) );
    sin_phi24 = (-A(4)*sin(th3_4)) / p41xz_4;
    phi_24    = atan2( sin_phi24, sqrt(1 - sin_phi24^2) );
    
    th2_1 = phi_11 - phi_21;
    th2_2 = phi_12 - phi_22;
    th2_3 = phi_13 - phi_23;
    th2_4 = phi_14 - phi_24;

    phi_15    = atan2( -p41_1(3), -p41_1(1) );
    sin_phi25 = (-A(4)*sin(th3_5)) / p41xz_1;
    phi_25    = atan2( sin_phi25, sqrt(1 - sin_phi25^2) );

    phi_16    = atan2( -p41_2(3), -p41_2(1) );
    sin_phi26 = (-A(4)*sin(th3_6)) / p41xz_2;
    phi_26    = atan2( sin_phi26, sqrt(1 - sin_phi26^2) );

    phi_17    = atan2( -p41_3(3), -p41_3(1) );
    sin_phi27 = (-A(4)*sin(th3_7)) / p41xz_3;
    phi_27    = atan2( sin_phi27, sqrt(1 - sin_phi27^2) );

    phi_18    = atan2( -p41_4(3), -p41_4(1) );
    sin_phi28 = (-A(4)*sin(th3_8)) / p41xz_4;
    phi_28    = atan2( sin_phi28, sqrt(1 - sin_phi28^2) );
    
    th2_5 = phi_15 - phi_25;
    th2_6 = phi_16 - phi_26;
    th2_7 = phi_17 - phi_27;
    th2_8 = phi_18 - phi_28;

    fprintf("UR5_inverse_kinematics - 7\n")
    % finding th4 ==================================================================================
    T34   = (T01_1 * transf_i_1_i(2,th2_1) * transf_i_1_i(3,th3_1)) \ T06 / T56_1;
    th4_1 = atan2( T34(2,1), T34(1,1) );
    
    T34   = (T01_1 * transf_i_1_i(2,th2_2) * transf_i_1_i(3,th3_2)) \ T06 / T56_2;
    th4_2 = atan2( T34(2,1), T34(1,1) );
    
    T34   = (T01_2 * transf_i_1_i(2,th2_3) * transf_i_1_i(3,th3_3)) \ T06 / T56_3;
    th4_3 = atan2( T34(2,1), T34(1,1) );
    
    T34   = (T01_2 * transf_i_1_i(2,th2_4) * transf_i_1_i(3,th3_4)) \ T06 / T56_4;
    th4_4 = atan2( T34(2,1), T34(1,1) );

    
    T34   = (T01_1 * transf_i_1_i(2,th2_5) * transf_i_1_i(3,th3_5)) \ T06 / T56_1;
    th4_5 = atan2( T34(2,1), T34(1,1) );
    
    T34   = (T01_1 * transf_i_1_i(2,th2_6) * transf_i_1_i(3,th3_6)) \ T06 / T56_2;
    th4_6 = atan2( T34(2,1), T34(1,1) );
    
    T34   = (T01_2 * transf_i_1_i(2,th2_7) * transf_i_1_i(3,th3_7)) \ T06 / T56_3;
    th4_7 = atan2( T34(2,1), T34(1,1) );
    
    T34   = (T01_2 * transf_i_1_i(2,th2_8) * transf_i_1_i(3,th3_8)) \ T06 / T56_4;
    th4_8 = atan2( T34(2,1), T34(1,1) );

   
    % ======================================================================================
    Th = [th1_1 th2_1 th3_1 th4_1 th5_1 th6_1;
          th1_1 th2_2 th3_2 th4_2 th5_2 th6_2;
          th1_2 th2_3 th3_3 th4_3 th5_3 th6_3;
          th1_2 th2_4 th3_4 th4_4 th5_4 th6_4;
          th1_1 th2_5 th3_5 th4_5 th5_1 th6_1;
          th1_1 th2_6 th3_6 th4_6 th5_2 th6_2;
          th1_2 th2_7 th3_7 th4_7 th5_3 th6_3;
          th1_2 th2_8 th3_8 th4_8 th5_4 th6_4;
         ];

    fprintf("UR5_inverse_kinematics - end\n")
