% Linear intERPolation between initial and final configurations
% Parameters:
% t  : time at which to compute desired configuration p_d
% - p_d=p_i @ t=0
% - p_d=p_f @ t=T
% p_i: initial configuration 
% p_f: final configuration   
% T  : total period of lerp
function [p_d] = lerp( t, p_i, p_f, T )
    if t > T
        p_d = p_f;
    else
        p_d = t/T*p_f + (1-t/T)*p_i;
    end
end
