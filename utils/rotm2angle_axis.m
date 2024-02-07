% source: https://www.euclideanspace.com/maths/geometry/rotations/conversions/index.htm
function [angle, axis] = rotm2angle_axis( m )
	epsilon  = 0.01; % margin to allow for rounding errors
	epsilon2 = 0.1;  % margin to distinguish between 0 and 180 degrees
	% optional check that input is pure rotation, 'isRotationMatrix' is defined at:
	% https://www.euclideanspace.com/maths/algebra/matrix/orthogonal/rotation/
	% assert isRotationMatrix(m) : "not valid rotation matrix" ;// for debugging
    
    if abs(m(1,2)-m(2,1)) < epsilon && abs(m(1,3)-m(3,1)) < epsilon && abs(m(2,3)-m(3,2)) < epsilon
        % singularity found
        % first check for identity matrix which must have +1 for all terms in leading diagonal and zero in other terms
        if abs(m(1,2)+m(2,1)) < epsilon2 && abs(m(1,3)+m(3,1)) < epsilon2 && abs(m(2,3)+m(3,2)) < epsilon2 && abs(trace(m)-3) < epsilon2
            % this singularity is identity matrix so angle = 0 -> zero angle, arbitrary axis
            angle = 0;
            axis  = [1;0;0];
            return
        end
    
        % otherwise this singularity is angle = 180
        angle = pi;
        xx = (m(1,1)+1)/2;
        yy = (m(2,2)+1)/2;
        zz = (m(3,3)+1)/2;
        xy = (m(1,2)+m(2,1))/4;
        xz = (m(1,3)+m(3,1))/4;
        yz = (m(2,3)+m(3,2))/4;
        if xx > yy && xx > zz   % m[0][0] is the largest diagonal term
            if xx < epsilon
                x = 0;
                y = sqrt(2)/2;
                z = sqrt(2)/2;
            else
                x = sqrt( xx );
                y = xy / x;
                z = xz / x;
            end
        elseif yy > zz          % m[1][1] is the largest diagonal term
            if yy < epsilon
                x = sqrt(2)/2;
                y = 0;
                z = sqrt(2)/2;
            else
                y = sqrt( yy );
                x = xy / y;
                z = yz / y;
            end
        else                    % m[2][2] is the largest diagonal term so base result on this
            if zz < epsilon
                x = sqrt(2)/2;
                y = sqrt(2)/2;
                z = 0;
            else
                z = sqrt( zz );
                x = xz / z;
                y = yz / z;
            end
        end
        % return 180 deg rotation
        axis = [x;y;z];
        return
    end
	% as we have reached here there are no singularities so we can handle normally
	s = sqrt( (m(3,2) - m(2,3))*(m(3,2) - m(2,3)) + (m(1,3) - m(3,1))*(m(1,3) - m(3,1)) + (m(2,1) - m(1,2))*(m(2,1) - m(1,2)));
	% used to normalise
    
    if abs(s) < 0.001 
        s = 1;
    end
    % prevent divide by zero, should not happen if matrix is orthogonal and should be
	% caught by singularity test above, but I've left it in just in case
	cos_angle = ( trace(m) - 1) / 2;
	angle     = atan2( sqrt(1-cos_angle^2), cos_angle );
    x = (m(3,2) - m(2,3))/s;
	y = (m(1,3) - m(3,1))/s;
	z = (m(2,1) - m(1,2))/s;
    axis = [x;y;z];
end

