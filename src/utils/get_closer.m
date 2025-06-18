function [best_q] = get_closer(H, q)
    % Find the solution in H that is closest to q, ignoring solutions with NaN values
    
    % Initialize variables
    best_q = [];
    best_dist = Inf;
    
    % Check all solutions
    for i=1:size(H,1)
        % Skip solutions with NaN values
        if any(isnan(H(i,:)))
            continue;
        end
        
        % Calculate angular distance considering joint angle wrapping
        curr_dist = angular_distance(H(i,:), q);
        
        % Update best solution if current is better
        if curr_dist < best_dist
            best_dist = curr_dist;
            best_q = H(i,:);
        end
    end
    
    % If no valid solution was found (all rows contain NaN), return the input q
    if isempty(best_q)
        warning('All solutions contain NaN values. Returning input q as fallback.');
        best_q = q;
    end
    
    % Optional warning for large angular distances
    % if best_dist > 0.1
    %     disp("!!!!   Warning: Large angular distance between solutions !!!!")
    %     q
    %     H
    %     best_q
    % end
end

function dist = angular_distance(q1, q2)
    % Calculate angular distance considering joint angle wrapping
    % For each joint, find the minimum distance considering the 2π periodicity
    diff = q1 - q2;
    
    % Wrap differences to [-π, π] range
    wrapped_diff = mod(diff + pi, 2*pi) - pi;
    
    % Calculate Euclidean norm of the wrapped differences
    dist = norm(wrapped_diff);
end