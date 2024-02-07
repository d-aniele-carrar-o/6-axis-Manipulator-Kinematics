function [rotm] = eul2rotm( rpy )
    % rpy angles defined as [roll, pitch, yaw]
    % rot X -> rot Y -> rot Z

    rotx = [1,           0,            0;
            0, cos(rpy(1)), -sin(rpy(1));
            0, sin(rpy(1)),  cos(rpy(1))];

    roty = [ cos(rpy(2)), 0, sin(rpy(2));
                       0, 1,           0;
            -sin(rpy(2)), 0, cos(rpy(2))];

    rotz = [cos(rpy(3)), -sin(rpy(3)), 0;
            sin(rpy(3)),  cos(rpy(3)), 0;
                      0,            0, 1];

    rotm = rotx * roty * rotz;
end

