% Create a simple environment
figure;
create_environment(tablePosition, tableLength, tableWidth, tableHeight);

% Set a configuration
q0 = [0, pi/4, -pi/4, 0, pi/4, 0];
config = set_robot_configuration(q0, config);

% Show the robot in the environment
ax = show( robot, config, "Visuals", "on", "Frames", "off", "FastUpdate", true, "PreservePlot", false); hold on;
title('Robot in Working Environment');
xlabel('X'); ylabel('Y'); zlabel('Z');
view(3);grid on;
xlim([-1, 1]);
ylim([-1, 1]);
zlim([0, 1]);

% Enable lighting for better visualization
light('Position', [1 1 5], 'Style', 'infinite');
lighting gouraud;
material dull;

% Simple animation
disp('Press any key to start animation...');
pause()

for t = 0:0.05:2*pi
    q = q0 + 0.2*sin(5*t);
    config = set_robot_configuration(q, config);
    show( robot, config, "Visuals", "on", "Frames", "off", "FastUpdate", true, "PreservePlot", false, "Parent", ax ); hold on;
    drawnow;
    pause(0.01);
end
