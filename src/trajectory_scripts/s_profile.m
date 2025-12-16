clc; close all;

% --- 1. Multi-Axis Inputs ---
% Distances for each axis (e.g., X, Y, Z, Rx, Ry, Rz)
dist_axes = [10, 5, 0.5];   % meters or radians

% Constraints PER AXIS
% Note: You can have different cruise limits for rotation vs translation
v_cruise_limits = [5, 5, 2];     % m/s or rad/s
a_max_limits    = [4, 4, 4];     % m/s^2 (or vector if distinct)
j_max_limits    = [10, 10, 10];  % m/s^3 (or vector if distinct)

% --- 2. Project Constraints to 1D Path ---
% Total Path Length
dist_path = norm(dist_axes);

if dist_path < 1e-9
    error('Total distance is zero.');
end

% Unit Direction Vector (how much each axis contributes to the path)
u_vec = dist_axes / dist_path; 

% Avoid division by zero for axes that don't move
active_mask = abs(u_vec) > 1e-6;

% Calculate Effective Path Limits
% Logic: If Axis 1 moves 100% of the path, PathLimit = AxisLimit.
%        If Axis 1 moves 50% of the path (u=0.5), PathLimit = 2 * AxisLimit.
%        We take the MINIMUM across all active axes to stay safe.

% Expand scalar limits to vectors for easy math
if size(a_max_limits) ~= size(dist_axes) 
    a_limits = repmat(a_max_common, size(dist_axes));
end
if size(j_limits) ~= size(dist_axes) 
    j_limits = repmat(j_max_common, size(dist_axes));
end

path_v_max = min(v_cruise_limits(active_mask) ./ abs(u_vec(active_mask)));
path_a_max = min(a_limits(active_mask)        ./ abs(u_vec(active_mask)));
path_j_max = min(j_limits(active_mask)        ./ abs(u_vec(active_mask)));


% --- 3. Run the Robust 1D Solver using Path Inputs ---
% Inputs: dist_path, path_j_max, path_a_max, path_v_max

% A. Theoretical Limits
v_accel_limit = path_a_max^2 / path_j_max;

% B. Ideal Times (Infinite Distance)
if path_v_max <= v_accel_limit
    Ta_ideal = sqrt(path_v_max / path_j_max);
    Tc_ideal = 0;
else
    Ta_ideal = path_a_max / path_j_max;
    Tc_ideal = (path_v_max - path_a_max * Ta_ideal) / path_a_max;
end

% C. Check Distance Requirement
d_req_for_cruise = path_v_max * (2 * Ta_ideal + Tc_ideal);

if dist_path >= d_req_for_cruise
    % Scenario: Long Move
    Ta = Ta_ideal;
    Tc = Tc_ideal;
    d_cruise = dist_path - d_req_for_cruise;
    Tcruise = d_cruise / path_v_max;
    
else
    % Scenario: Short Move (No Cruise)
    Tcruise = 0;
    d_half = dist_path / 2;
    d_amax_triangular = path_j_max * (path_a_max / path_j_max)^3;
    
    if d_half < d_amax_triangular
        % Sub-Case: Double Triangle (Jerk Limited)
        Ta = (d_half / path_j_max)^(1/3);
        Tc = 0;
    else
        % Sub-Case: Trapezoidal Accel (Accel Limited)
        Ta = path_a_max / path_j_max;
        roots_tc = roots([1, 3*Ta, (2*Ta^2 - 2*d_half/path_a_max)]);
        Tc = max(roots_tc);
    end
end

% --- 4. Generate Global Path Profile ---
% We calculate the scalar 's' (distance along path)
% Parameters for the piecewise function
j_use = path_j_max;
a_use = path_j_max * Ta; % The actual peak accel reached
v0    = 0;

t1 = Ta;
t2 = Ta + Tc;
t3 = 2*Ta + Tc;
t4 = 2*Ta + Tc + Tcruise;
t5 = 3*Ta + Tc + Tcruise;
t6 = 3*Ta + 2*Tc + Tcruise;
T  = 4*Ta + 2*Tc + Tcruise;

% Pre-calc boundary states for the PATH
v1 = v0 + 0.5 * j_use * Ta^2;
s1 = v0 * Ta + (1/6) * j_use * Ta^3;
v2 = v1 + a_use * Tc;
s2 = s1 + v1 * Tc + 0.5 * a_use * Tc^2;
v_mid = v2 + 0.5 * j_use * Ta^2;
s3 = s2 + v2 * Ta + 0.5 * a_use * Ta^2 - (1/6) * j_use * Ta^3;
s4 = s3 + v_mid * Tcruise;
v5 = v_mid - 0.5 * j_use * Ta^2;
s5 = s4 + v_mid * Ta - (1/6) * j_use * Ta^3;
v6 = v5 - a_use * Tc;
s6 = s5 + v5 * Tc - 0.5 * a_use * Tc^2;

% Define the SCALAR path position s(t)
% (Using the clean recursive definition)
syms t
v_path = piecewise(t < t1, v0 + 0.5 * j_use * t^2, ...
                   t < t2, v1     + a_use * (t - t1), ...
                   t < t3, v2     + a_use * (t - t2) - 0.5 * j_use * (t - t2)^2, ...
                   t < t4, v_mid, ...
                   t < t5, v_mid  - 0.5 * j_use * (t - t4)^2, ...
                   t < t6, v5     - a_use * (t - t5), ...
                   t <= T, v6     - a_use * (t - t6) + 0.5 * j_use * (t - t6)^2, ...
                   0);

s_path = piecewise(t < t1, v0*t + (1/6)*j_use*t^3, ...
                   t < t2, s1 + v1*(t-t1) + 0.5*a_use*(t-t1)^2, ...
                   t < t3, s2 + v2*(t-t2) + 0.5*a_use*(t-t2)^2 - (1/6)*j_use*(t-t2)^3, ...
                   t < t4, s3 + v_mid*(t-t3), ...
                   t < t5, s4 + v_mid*(t-t4) - (1/6)*j_use*(t-t4)^3, ...
                   t < t6, s5 + v5*(t-t5) - 0.5*a_use*(t-t5)^2, ...
                   t <= T, s6 + v6*(t-t6) - 0.5*a_use*(t-t6)^2 + (1/6)*j_use*(t-t6)^3, ...
                   dist_path);

% --- 5. Distribute to Axes (The Final Output) ---
% Simply scale the path function by the unit vector
% s_axes will be a vector function [x(t), y(t), z(t)]

s_axes = s_path * u_vec; 
v_axes = v_path * u_vec;
a_axes = diff(v_axes, t); % Conceptually: a_path * u_vec

% Time vector
t_vec = linspace(0, T, 500);

% Evaluate all axes using arrayfun
n_axes = length(dist_axes);
a_axes_vec = zeros(length(t_vec), n_axes);
v_axes_vec = zeros(length(t_vec), n_axes);
s_axes_vec = zeros(length(t_vec), n_axes);

for i = 1:n_axes
    a_axes_vec(:,i) = arrayfun(@(ti) double(subs(a_axes(i), t, ti)), t_vec);
    v_axes_vec(:,i) = arrayfun(@(ti) double(subs(v_axes(i), t, ti)), t_vec);
    s_axes_vec(:,i) = arrayfun(@(ti) double(subs(s_axes(i), t, ti)), t_vec);
end

% Plot
figure('Position', [100 100 1200 800]);
axis_labels = {'X', 'Y', 'Z', 'Rx', 'Ry', 'Rz'};

subplot(3,1,1)
plot(t_vec, a_axes_vec, 'LineWidth', 2)
grid on
ylabel('Acceleration [m/s^2]')
title('Multi-Axis S-Curve Motion Profile')
xline([t1, t2, t3, t4, t5, t6], '--k', 'Alpha', 0.3)
legend(axis_labels(1:n_axes), 'Location', 'best')
xlim([-0.05*T, 1.05*T])

subplot(3,1,2)
plot(t_vec, v_axes_vec, 'LineWidth', 2)
grid on
ylabel('Velocity [m/s]')
xline([t1, t2, t3, t4, t5, t6], '--k', 'Alpha', 0.3)
legend(axis_labels(1:n_axes), 'Location', 'best')
xlim([-0.05*T, 1.05*T])

subplot(3,1,3)
plot(t_vec, s_axes_vec, 'LineWidth', 2)
grid on
xlabel('Time [s]')
ylabel('Position [m]')
xline([t1, t2, t3, t4, t5, t6], '--k', 'Alpha', 0.3)
legend(axis_labels(1:n_axes), 'Location', 'best')
xlim([-0.05*T, 1.05*T])
