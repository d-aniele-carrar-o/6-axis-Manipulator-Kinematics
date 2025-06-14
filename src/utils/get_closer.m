function [best_q] = get_closer( H, q )
    % Initialize with first solution
    best_q = H(1,:);
    
    % Calculate angular distance considering joint angle wrapping
    best_dist = angular_distance(H(1,:), q);
    
    % Check all other solutions
    for i=2:8
        curr_dist = angular_distance(H(i,:), q);
        if curr_dist < best_dist
            best_dist = curr_dist;
            best_q = H(i,:);
        end
    end
    
    if best_dist > 0.1
        disp("Warning: Large angular distance between solutions")
        q
        H
        best_q
    end
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
