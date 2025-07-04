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
    surf(X, Y, Z, 'FaceColor', [0.5, 0.6, 0.7], 'EdgeColor', 'none', 'FaceAlpha', 1.0);
    hold on;

    % Add the table with thickness
    tableColor = [0.6, 0.4, 0.2];
    tableThickness = 0.05; % 5cm thick table
    
    % Draw table top surface (at tableHeight)
    [X, Y] = meshgrid(-tableParams.length/2:0.05:tableParams.length/2, -tableParams.width/2:0.05:tableParams.width/2);
    Z = zeros(size(X)) + tableParams.height;
    tableSurf = surf(X, Y, Z, 'FaceColor', [0.8 0.7 0.6], 'EdgeColor', 'none');
    set(tableSurf, 'AmbientStrength', 0.8, 'DiffuseStrength', 0.6, 'SpecularStrength', 0.1);
    
    % Draw table bottom surface
    Z_bottom = zeros(size(X)) + tableParams.height - tableThickness;
    surf(X, Y, Z_bottom, 'FaceColor', tableColor, 'EdgeColor', 'none');
    
    % Draw table edges
    % Front edge
    [X_edge, Z_edge] = meshgrid(-tableParams.length/2:0.05:tableParams.length/2, tableParams.height-tableThickness:0.01:tableParams.height);
    Y_edge = ones(size(X_edge)) * (-tableParams.width/2);
    surf(X_edge, Y_edge, Z_edge, 'FaceColor', tableColor, 'EdgeColor', 'none');
    
    % Back edge
    Y_edge = ones(size(X_edge)) * (tableParams.width/2);
    surf(X_edge, Y_edge, Z_edge, 'FaceColor', tableColor, 'EdgeColor', 'none');
    
    % Left edge
    [Y_edge, Z_edge] = meshgrid(-tableParams.width/2:0.05:tableParams.width/2, tableParams.height-tableThickness:0.01:tableParams.height);
    X_edge = ones(size(Y_edge)) * (-tableParams.length/2);
    surf(X_edge, Y_edge, Z_edge, 'FaceColor', tableColor, 'EdgeColor', 'none');
    
    % Right edge
    X_edge = ones(size(Y_edge)) * (tableParams.length/2);
    surf(X_edge, Y_edge, Z_edge, 'FaceColor', tableColor, 'EdgeColor', 'none');
    
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
        Z = Z * tableParams.height - 0.02;
        Z = Z + legPositions(i, 3);
        X = X + legPositions(i, 1);
        Y = Y + legPositions(i, 2);
        surf(X, Y, Z, 'FaceColor', tableColor, 'EdgeColor', 'none');
    end

    % Add aluminum base supports (parallelepipeds under each robot base)
    baseSupportColor = [0.7, 0.7, 0.7]; % Grey metallic color
    baseSupportDims = [tableParams.length, 0.24, 0.04]; % tableLength x 24cm x 4cm
    
    % Robot positions (left and right)
    robotPositions = [
        0, -robotsDistance/2; % Left robot
        0,  robotsDistance/2  % Right robot
    ];
    
    % Draw aluminum base supports for both robots
    for i = 1:2
        baseSupportPos = [robotPositions(i, 1), robotPositions(i, 2), tableParams.height];
        
        % Create parallelepiped vertices
        vertices = [
            -baseSupportDims(1)/2, -baseSupportDims(2)/2, 0;
             baseSupportDims(1)/2, -baseSupportDims(2)/2, 0;
             baseSupportDims(1)/2,  baseSupportDims(2)/2, 0;
            -baseSupportDims(1)/2,  baseSupportDims(2)/2, 0;
            -baseSupportDims(1)/2, -baseSupportDims(2)/2, baseSupportDims(3);
             baseSupportDims(1)/2, -baseSupportDims(2)/2, baseSupportDims(3);
             baseSupportDims(1)/2,  baseSupportDims(2)/2, baseSupportDims(3);
            -baseSupportDims(1)/2,  baseSupportDims(2)/2, baseSupportDims(3)
        ];
        
        % Translate vertices to position
        vertices = vertices + baseSupportPos;
        
        % Define faces for the parallelepiped
        faces = [
            1 2 3 4; % bottom
            5 6 7 8; % top
            1 2 6 5; % front
            3 4 8 7; % back
            2 3 7 6; % right
            1 4 8 5  % left
        ];
        
        % Draw the aluminum base support
        patch('Vertices', vertices, 'Faces', faces, 'FaceColor', baseSupportColor, ...
              'EdgeColor', 'none', 'AmbientStrength', 0.6, 'DiffuseStrength', 0.8, 'SpecularStrength', 0.4);
    end
    
    % Add cylindrical robot base supports
    cylinderRadius = 0.10; % 20cm diameter = 10cm radius
    cylinderHeight = standHeight - baseSupportDims(3); % Height from parameters
    cylinderColor = [0.2, 0.2, 0.2]; % Dark grey
    
    % Draw cylindrical supports for both robots
    for i = 1:2
        % Cylinder body
        [X, Y, Z] = cylinder(cylinderRadius, 20);
        Z = Z * cylinderHeight;
        Z = Z + tableParams.height + baseSupportDims(3); % Start from top of base support
        X = X + robotPositions(i, 1);
        Y = Y + robotPositions(i, 2);
        surf(X, Y, Z, 'FaceColor', cylinderColor, 'EdgeColor', 'none', ...
             'FaceAlpha', 1.0, 'AmbientStrength', 0.8);
        
        % Cylinder top cover
        [Xtop, Ytop] = meshgrid(linspace(-cylinderRadius, cylinderRadius, 30));
        mask = (Xtop.^2 + Ytop.^2) <= cylinderRadius^2;
        Xtop(~mask) = NaN;
        Ytop(~mask) = NaN;
        Ztop = ones(size(Xtop)) * (tableParams.height + baseSupportDims(3) + cylinderHeight);
        surf(Xtop + robotPositions(i, 1), Ytop + robotPositions(i, 2), Ztop, ...
             'FaceColor', cylinderColor, 'EdgeColor', 'none', 'FaceAlpha', 1.0);
    end

end
