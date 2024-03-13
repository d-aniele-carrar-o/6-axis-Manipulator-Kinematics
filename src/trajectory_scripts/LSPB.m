function [q_d, q_dot_d] = LSPB( t, ti, tf, tb, qi, qf, v )
    % Linear Segments with Parabolic Blends - trapezodial velocity profile
    
    a = v / tb;
    
    if t < ti
        q_d     = qi';
        q_dot_d = zeros( size(qi) )';
    elseif t < ti + tb
        q_d     = qi' + a'/2 * (t-ti)^2;
        q_dot_d = a'*(t-ti);
    elseif t <= tf - tb
        q_d     = qi' + v'*(t-ti-tb/2);
        q_dot_d = v';
    elseif t < tf
        q_d     = qf' - a'/2 * (tf-t)^2;
        q_dot_d = a'*(tf-t);
    else
        q_d     = qf';
        q_dot_d = zeros( size(qf) )';
    end

end
