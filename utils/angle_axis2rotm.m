% source: https://www.euclideanspace.com/maths/geometry/rotations/conversions/index.htm
function [rotm] = angle_axis2rotm( angle, axis )
    c = cos( angle );
    s = sin( angle );
    t = 1.0 - c;
	%  if axis is not already normalised then uncomment this
	% double magnitude = Math.sqrt(a1.x*a1.x + a1.y*a1.y + a1.z*a1.z);
	% if (magnitude==0) throw error;
	% a1.x /= magnitude;
	% a1.y /= magnitude;
	% a1.z /= magnitude;
    
    rotm = zeros( 3 );

    rotm(1,1) = c + t*axis(1)^2;
    rotm(2,2) = c + t*axis(2)^2;
    rotm(3,3) = c + t*axis(3)^2;

    tmp1 = axis(1) * axis(2) * t;
    tmp2 = axis(3) * s;
    rotm(2,1) = tmp1 + tmp2;
    rotm(1,2) = tmp1 - tmp2;
    
    tmp1 = axis(1) * axis(3) * t;
    tmp2 = axis(2) * s;
    rotm(3,1) = tmp1 - tmp2;
    rotm(1,3) = tmp1 + tmp2;
    
    tmp1 = axis(2) * axis(3) * t;
    tmp2 = axis(1) * s;
    rotm(3,2) = tmp1 + tmp2;
    rotm(2,3) = tmp1 - tmp2;
end

