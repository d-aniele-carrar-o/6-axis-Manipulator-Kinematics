function [best_q] = get_closer( H, q )
    best_q    = H(1,:);
    best_norm = norm( H(1,:) - q );
    
    for i=2:8
        curr_norm = norm( H(i,:) - q );
        if curr_norm < best_norm
            best_norm = curr_norm;
            best_q    = H(i,:);

        end

    end
    
end
