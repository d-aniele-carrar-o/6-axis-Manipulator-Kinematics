function [R] = eul2rotmFDR(eulXYZ)
    eulZYX(3) = eulXYZ(1);
    eulZYX(2) = eulXYZ(2);
    eulZYX(1) = eulXYZ(3);
    R = eul2rotm(eulZYX);
end