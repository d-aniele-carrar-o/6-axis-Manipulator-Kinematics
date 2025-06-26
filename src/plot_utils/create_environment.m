function [axs] = create_environment()
    parameters(1);

    axs = gca;
    axs.AmbientLightColor = [0.6 0.6 0.6];

    xlimits = [-tableParams.length, tableParams.length];
    ylimits = [-tableParams.width,  tableParams.width];
    zlimits = [-0.1,1.3];
    xlim(xlimits); ylim(ylimits); zlim(zlimits);
    xlabel('X [m]'); ylabel('Y [m]'); zlabel('Z [m]');
    view(0, 30); % Frontal view (looking along Y-axis toward the origin)
    
    % Add coordinate frame axes for clarity
    plot3([0 0.2], [0 0], [0 0], 'r-', 'LineWidth', 3, 'Parent', axs); text(0.2, 0.01, 0, 'World X', 'Color', 'r', 'Parent', axs);
    plot3([0 0], [0 0.2], [0 0], 'g-', 'LineWidth', 3, 'Parent', axs); text(0, 0.21, 0, 'World Y', 'Color', 'g', 'Parent', axs);
    plot3([0 0], [0 0], [0 0.2], 'b-', 'LineWidth', 3, 'Parent', axs); text(0, 0.01, 0.2, 'World Z', 'Color', 'b', 'Parent', axs);

    % Add proper lighting for textures
    light('Position',[ 1  1 1],'Style','infinite');
    light('Position',[-1 -1 1],'Style','infinite');
    lighting gouraud;

    % Enhance visual appearance with better lighting and materials
    material('dull');
    camlight('headlight');
    camlight('right');
    lighting gouraud;

    % Add a floor
    [X, Y] = meshgrid(linspace(xlimits(1), xlimits(2), 15), linspace(ylimits(1), ylimits(2), 15));
    Z = zeros(size(X));
    surf(X, Y, Z, 'FaceColor', [0.8, 0.8, 0.8], 'EdgeColor', [0.7, 0.7, 0.7]);
    hold on;

    % Add the table
    tableColor = [0.6, 0.4, 0.2];

    % Draw table top
    [X, Y] = meshgrid(-tableParams.length/2:0.05:tableParams.length/2, -tableParams.width/2:0.05:tableParams.width/2);
    Z = zeros(size(X)) + tableParams.height;  % Table is at Z=tableHeight in world frame
    tableSurf = surf(X, Y, Z, 'FaceColor', [0.8 0.7 0.6], 'EdgeColor', 'none');
    set(tableSurf, 'AmbientStrength', 0.8, 'DiffuseStrength', 0.6, 'SpecularStrength', 0.1);
    hold on;

    % Draw table legs
    legRadius = 0.05;
    legPositions = [
        tablePosition(1)-tableParams.length/2+legRadius, tablePosition(2)-tableParams.width/2+legRadius, 0;
        tablePosition(1)+tableParams.length/2-legRadius, tablePosition(2)-tableParams.width/2+legRadius, 0;
        tablePosition(1)+tableParams.length/2-legRadius, tablePosition(2)+tableParams.width/2-legRadius, 0;
        tablePosition(1)-tableParams.length/2+legRadius, tablePosition(2)+tableParams.width/2-legRadius, 0
    ];

    for i = 1:4
        [X, Y, Z] = cylinder(legRadius, 10);
        Z = Z * tableParams.height;
        Z = Z + legPositions(i, 3);
        X = X + legPositions(i, 1);
        Y = Y + legPositions(i, 2);
        surf(X, Y, Z, 'FaceColor', tableColor, 'EdgeColor', 'none');
    end

end
