function [eulXYZ] = rotm2eulFDR(R)
    % the output of rotm2eul is a vector with ZYX coordinates so I need to
    % convert it
    eulZYX =rotm2eul(R,'ZYX');
    eulXYZ(3) = eulZYX(1);
    eulXYZ(2) = eulZYX(2);
    eulXYZ(1) = eulZYX(3);   
end