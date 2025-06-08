% Dual robot setup and simulation
clear; clc; close all;

% Setup first robot (UR3e left)
parameters(0, 1);
robot_left  = robot;
config_left = config;
Trf_0_l     = Trf_0;

% Setup second robot (UR3e right)
parameters(0, 2);
robot_right  = robot;
config_right = config;
Trf_0_r      = Trf_0;

% Create environment for multi-robot setup --------------------------------------------------------
figure('Name', 'Dual Robot Workspace');
hold on; grid on;
create_environment( tablePosition, tableLength, tableWidth, tableHeight );

% Initial joint configurations --------------------------------------------------------------------
q0_left  = [0,  pi/6,  5*pi/6, 0,  pi/2,  pi]
q0_right = [0, -pi/6, -5*pi/6, 0, -pi/2, 0.0]

% Set robot configurations
config_left  = set_robot_configuration( q0_left, config_left );
config_right = set_robot_configuration( q0_right, config_right );

% Visualize robots
axs = show( robot_left, config_left, "Visuals", "on", "Frames", "on", "FastUpdate", true, "PreservePlot", false );
show( robot_right, config_right, "Visuals", "on", "Frames", "on", "FastUpdate", true, "PreservePlot", false, "Parent", axs );

pause()

% Compute end-effector poses
parameters(1, 1); % Load parameters for robot 1
[Te_w_e_left,  Te_l] = direct_kinematics( q0_left, 1 );

parameters(1, 2); % Load parameters for robot 2
[Te_w_e_right, Te_r] = direct_kinematics( q0_right, 2 );

% Display end-effector positions
disp('Robot left end-effector pose (wrt world frame):');
disp(Te_w_e_left);
disp('Robot right end-effector pose (wrt world frame):');
disp(Te_w_e_right);

% Create trajectories for both robots -------------------------------------------------------------
disp('Planning trajectories for both robots...');
%         /------------/
%        /            /
%       /      R     /
%    + /     |/     /
%   Y /      o-    / world reference frame
%  - /            /
%   /      L     /
%  /            /
% /------------/
%     - X +

% Create viapoints for both robots in world reference frame
ti   = 0

% First viapoint: set first desired configuration/pose (viapoint)
pf_l = [-0.3; -0.05; tableHeight + 0.1];
T_w_o_l = [Te_l(1:3,1:3), pf_l; 0,0,0,1]
Tf_l = inv(Trf_0_l) * T_w_o_l

pf_r = [-0.3;  0.05; tableHeight + 0.1];
T_w_o_r = [Te_r(1:3,1:3), pf_r; 0,0,0,1]
Tf_r = inv(Trf_0_r) * T_w_o_r

t1   = 1

% Second viapoint: set second desired configuration/pose (viapoint)
pf2_l = T_w_o_l(1:3,4) + [0; 0; 0.2];
T2_w_o_l = [Tf_l(1:3,1:3), pf2_l; 0,0,0,1]
Tf2_l = inv(Trf_0_l) * T2_w_o_l

pf2_r = T_w_o_r(1:3,4) + [0; 0; 0.2];
T2_w_o_r = [Tf_r(1:3,1:3), pf2_r; 0,0,0,1]
Tf2_r = inv(Trf_0_r) * T2_w_o_r

t2   = 2

viapoints_l = [Tf_l; Tf2_l];
viapoints_r = [Tf_r; Tf2_r];
times       = [ti, t1, t2];

% Compute multi-viapoint trajectory for selected times and viapoints for both robots
[t_l, p_l, v_l] = multipoint_trajectory( q0_left,  viapoints_l, times );
[t_r, p_r, v_r] = multipoint_trajectory( q0_right, viapoints_r, times );

% Plot manipulator and scatter the positions of the end-effector to highlight trajectory 3D in space
p_ee_l = Trf_0_l(1:3,4);
p_ee_r = Trf_0_r(1:3,4);
scatter3( p_ee_l(1), p_ee_l(2), p_ee_l(3), 10, 'r.', 'Parent', axs ); hold on;
scatter3( p_ee_r(1), p_ee_r(2), p_ee_r(3), 10, 'b.', 'Parent', axs ); hold on;

disp("Press button to start simulation..")
k = waitforbuttonpress;
while k ~= 1
    k = waitforbuttonpress;
end

disp("[simulate] Simulation started.")
for i=2:max(size( p_l, 1 ), size( p_r, 1 ))
    if i <= size( p_l, 1 )
        config_left = set_robot_configuration( p_l(i,:), config_left );
        show( robot_left, config_left, "Visuals", "on", "Frames", "off", "FastUpdate", true, "PreservePlot", false, "Parent", axs ); hold on;
        [Te_w_e_left,  Te_l] = direct_kinematics( p_l(i,1:6), 1 );
        p_ee_l = Te_w_e_left(1:3,4);
        if (mod(i, 5) == 0)
            scatter3( p_ee_l(1), p_ee_l(2), p_ee_l(3), 10, 'r.', 'Parent', axs ); hold on;
        end
    end

    if i <= size( p_r, 1 )
        config_right = set_robot_configuration( p_r(i,:), config_right );
        show( robot_right, config_right, "Visuals", "on", "Frames", "off", "FastUpdate", true, "PreservePlot", false, "Parent", axs ); hold on;
        [Te_w_e_right,  Te_r] = direct_kinematics( p_r(i,1:6), 2 );
        p_ee_r = Te_w_e_right(1:3,4);
        if (mod(i, 5) == 0)
            scatter3( p_ee_r(1), p_ee_r(2), p_ee_r(3), 10, 'b.', 'Parent', axs ); hold on;
        end
    end
    waitfor( rate );
end
