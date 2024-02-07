function [slerp] = rotm_slerp( rotmi, rotmf, N )
    T     = linspace( 0, 1, N );
    slerp = zeros( [3,3,N] );
    qi    = rotm2quat( rotmi );
    qf    = rotm2quat( rotmf );
    
    % compute angle between initial and final quaternion
    cos_half_theta = dot( qi, qf ) / (norm(qi)*norm(qf));

    if cos_half_theta < 0 
        qf = -qf;
        cos_half_theta = -cos_half_theta;
    end
    
    angle = atan2( sqrt(1-cos_half_theta^2), cos_half_theta );
    denom = sin( angle );
    
    % t € [0,1] -> 
    % - @t=0: rotm_m = rotmi
    % - @t=1: rotm_m = rotmf
    for i=1:N
        if round( denom, 5 ) ~= 0
            qm = ( qi*sin((1.-T(i))*angle) + qf*sin(T(i)*angle) ) / denom;
            rotm_m = quat2rotm( qm );
        else
            rotm_m = rotmf;
        end
        
        slerp(:,:,i) = rotm_m;
    end
end
