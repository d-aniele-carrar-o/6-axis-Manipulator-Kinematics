parameters(0)

basePosition_l    = [tablePosition(1), tablePosition(2)-0.4, tableHeight];
baseOrientation_l = [0, 0, 0];
Trf_0_l = trvec2tform(basePosition_l) * eul2tform(baseOrientation_l);
basePosition_r    = [tablePosition(1), tablePosition(2)+0.4, tableHeight];
baseOrientation_r = [0, 0, 0];
Trf_0_r = trvec2tform(basePosition_r) * eul2tform(baseOrientation_r);

robot_left  = get_robot(dhparams, Trf_0_l, true);
robot_right = get_robot(dhparams, Trf_0_r, true);

figure;
create_environment(tablePosition, tableLength, tableWidth, tableHeight);

% Set initial configurations
T_home_w_l = Trf_0 * T_home;
T_home_w_r = Trf_0 * T_home2;
[config_left,  ~] = ik("tool0", T_home_w_l, ones(6,1), initial_guess);
[config_right, ~] = ik("tool0", T_home_w_r, ones(6,1), initial_guess2);
q_home_l = [config_left.JointPosition]
q_home_r = [config_right.JointPosition]
config_left  = set_robot_configuration(q_home_l, config_left);
config_right = set_robot_configuration(q_home_r, config_right);

% Show both robots in the environment
ax = show(robot_left, config_left, "Visuals", "on", "Frames", "on", "FastUpdate", true, "PreservePlot", false); hold on;
show(robot_right, config_right, "Visuals", "on", "Frames", "on", "FastUpdate", true, "PreservePlot", false, "Parent", ax);

% Set up the plot
title('Dual Robot Working Environment');
xlabel('X'); ylabel('Y'); zlabel('Z');
view(3); grid on;
xlim([-1, 1]);
ylim([-1, 1]);
zlim([0, 1]);

% Enable lighting for better visualization
light('Position', [1 1 5], 'Style', 'infinite');
lighting gouraud;
material dull;

% Simple animation example (uncomment to use)
disp('Press any key to start animation...');
pause()

% Trajectory initial time and time periods for each piece of trajectory
ti    = 0;

% First viapoint - set first desired configuration/pose (viapoint)
phi_f = [-pi/2, 0, 0];
Rf = eul2rotm_custom( phi_f );
pf = [0; 0.5; 0.5];
Tf = [Rf, pf; 0,0,0,1]

% Second viapoint: set second desired configuration/pose (viapoint)
Tf2 = Tf + [zeros(3), [-0.4; 0; 0]; 0,0,0,0]

% Third viapoint: set second desired configuration/pose (viapoint)
Tf3 = Tf2 + [zeros(3), [0; -0.4; 0]; 0,0,0,0]

% Fourth viapoint: set second desired configuration/pose (viapoint)
Tf4 = Tf3 + [zeros(3), [0.4; 0; 0]; 0,0,0,0]

viapoints = [Tf; Tf2; Tf3; Tf4];
times     = [ti, 1, 2, 3, 4];

% Compute multi-viapoint trajectory for selected times and viapoints
[t, p, v] = multipoint_trajectory( q0_left, viapoints, times );

[qf, handlesR] = simulate( robot_left, t, p, [], v, q0_left, NaN, false );
