function [quat] = rotm2quat( rotm )
    tr = rotm(1,1) + rotm(2,2) + rotm(3,3);

    if tr > 0                                                       % S=4*qw 
        S  = sqrt(tr+1.0) * 2;  
        qw = 0.25 * S;
        qx = (rotm(3,2) - rotm(2,3)) / S;
        qy = (rotm(1,3) - rotm(3,1)) / S;
        qz = (rotm(2,1) - rotm(1,2)) / S;
    elseif rotm(1,1) > rotm(2,2) && rotm(1,1) > rotm(3,3)           % S=4*qx 
        S  = sqrt(1.0 + rotm(1,1) - rotm(2,2) - rotm(3,3)) * 2;  
        qw = (rotm(3,2) - rotm(2,3)) / S;
        qx = 0.25 * S;
        qy = (rotm(1,2) + rotm(2,1)) / S;
        qz = (rotm(1,3) + rotm(3,1)) / S;
    elseif rotm(2,2) > rotm(3,3)                                    % S=4*qy
        S  = sqrt(1.0 + rotm(2,2) - rotm(1,1) - rotm(3,3)) * 2;  
        qw = (rotm(1,3) - rotm(3,1)) / S;
        qx = (rotm(1,2) + rotm(2,1)) / S;
        qy = 0.25 * S;
        qz = (rotm(2,3) + rotm(3,2)) / S;
    else                                                            % S=4*qz
        S  = sqrt(1.0 + rotm(3,3) - rotm(1,1) - rotm(2,2)) * 2;  
        qw = (rotm(2,1) - rotm(1,2)) / S;
        qx = (rotm(1,3) + rotm(3,1)) / S;
        qy = (rotm(2,3) + rotm(3,2)) / S;
        qz = 0.25 * S;
    end

    quat = [qw, qx, qy, qz];
end