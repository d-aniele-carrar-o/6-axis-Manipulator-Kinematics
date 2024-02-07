%Direct Kinematics of the UR5
%Th: six joint angles
%pe: cartesian position of the end effector
%Re: Rotation matrix of the end effecto
function [Te] = UR5_direct_kinematics( th )

    T01 = transf_i_1_i( 1, th(1) );
    T12 = transf_i_1_i( 2, th(2) );
    T23 = transf_i_1_i( 3, th(3) );
    T34 = transf_i_1_i( 4, th(4) );
    T45 = transf_i_1_i( 5, th(5) );
    T56 = transf_i_1_i( 6, th(6) );
    
    T06 = T01 * T12 * T23 * T34 * T45 * T56;
    
    pe = T06(1:3,4);
    Re = T06(1:3, 1:3);
    Te = [Re, pe; 0, 0, 0, 1];
end
