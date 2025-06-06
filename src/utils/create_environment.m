function create_environment(tablePosition, tableLength, tableWidth, tableHeight)
    xlim([-0.7,0.7]); ylim([-1,1]); zlim([-0.2,1.2]);
    xlabel('X [m]'); ylabel('Y [m]'); zlabel('Z [m]');
    view(3);

    % Add proper lighting for textures
    light('Position',[1 1 1],'Style','infinite');
    light('Position',[-1 -1 1],'Style','infinite');
    lighting gouraud;

    % Enhance visual appearance with better lighting and materials
    material('dull');
    camlight('headlight');
    camlight('right');
    lighting gouraud;
    ax = gca;
    ax.AmbientLightColor = [0.6 0.6 0.6];

    % Add a floor
    [X, Y] = meshgrid(-1:0.5:3, -1:0.5:3);
    Z = zeros(size(X));
    surf(X, Y, Z, 'FaceColor', [0.8, 0.8, 0.8], 'EdgeColor', [0.7, 0.7, 0.7]);
    hold on;

    % Add the table
    tableColor = [0.6, 0.4, 0.2];

    % Draw table top
    fill3([tablePosition(1)-tableLength/2, tablePosition(1)+tableLength/2, tablePosition(1)+tableLength/2, tablePosition(1)-tableLength/2], ...
        [tablePosition(2)-tableWidth/2, tablePosition(2)-tableWidth/2, tablePosition(2)+tableWidth/2, tablePosition(2)+tableWidth/2], ...
        [tablePosition(3)+tableHeight, tablePosition(3)+tableHeight, tablePosition(3)+tableHeight, tablePosition(3)+tableHeight], ...
        tableColor);

    % Draw table legs
    legRadius = 0.05;
    legPositions = [
        tablePosition(1)-tableLength/2+legRadius, tablePosition(2)-tableWidth/2+legRadius, 0;
        tablePosition(1)+tableLength/2-legRadius, tablePosition(2)-tableWidth/2+legRadius, 0;
        tablePosition(1)+tableLength/2-legRadius, tablePosition(2)+tableWidth/2-legRadius, 0;
        tablePosition(1)-tableLength/2+legRadius, tablePosition(2)+tableWidth/2-legRadius, 0
    ];

    for i = 1:4
        [X, Y, Z] = cylinder(legRadius, 10);
        Z = Z * tableHeight;
        Z = Z + legPositions(i, 3);
        X = X + legPositions(i, 1);
        Y = Y + legPositions(i, 2);
        surf(X, Y, Z, 'FaceColor', tableColor, 'EdgeColor', 'none');
    end

end
