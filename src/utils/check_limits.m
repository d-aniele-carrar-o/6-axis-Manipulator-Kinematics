% check joint limits - if outside -> set to limit value
function [q] = check_limits( q, limits )
    for i=1:max(size(q))
        q(i) = min( limits(i,1), max( q(i), limits(i,2) ) ); 
    end
end
