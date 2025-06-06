function [qf, handles] = simple_simulate(t, p, ~, v, q0, ~, plot_flag)
% SIMULATE Simple implementation of trajectory simulation
% This function is a fallback for when the original function has issues
%
% Inputs:
%   t - Time vector
%   p - Position trajectory
%   ~ - Unused parameter
%   v - Velocity trajectory
%   q0 - Initial joint configuration
%   ~ - Unused parameter
%   plot_flag - Flag to enable plotting
%
% Outputs:
%   qf - Final joint configuration
%   handles - Plot handles (if plotting is enabled)

    % Default plot flag
    if nargin < 7
        plot_flag = true;
    end
    
    % Initialize handles
    handles = [];
    
    % Final joint configuration
    qf = p(end,:);
    
    % Plot if requested
    if plot_flag
        figure;
        
        % Plot joint positions
        subplot(2,1,1);
        hold on;
        for i = 1:size(p,2)
            plot(t, p(:,i), 'LineWidth', 1.5);
        end
        grid on;
        xlabel('Time [s]');
        ylabel('Joint Position [rad]');
        title('Joint Positions');
        legend('q1', 'q2', 'q3', 'q4', 'q5', 'q6');
        
        % Plot joint velocities
        subplot(2,1,2);
        hold on;
        for i = 1:size(v,2)
            plot(t, v(:,i), 'LineWidth', 1.5);
        end
        grid on;
        xlabel('Time [s]');
        ylabel('Joint Velocity [rad/s]');
        title('Joint Velocities');
        legend('v1', 'v2', 'v3', 'v4', 'v5', 'v6');
    end
end