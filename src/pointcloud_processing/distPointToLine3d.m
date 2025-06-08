function distances = distPointToLine3d(points, lineModel)
% DISTPOINTTOLINE3D Calculate the distance from points to a 3D line
%   distances = distPointToLine3d(points, lineModel) calculates the shortest
%   distance from each point in points to the line defined by lineModel.
%
%   Inputs:
%   - points: Nx3 matrix of points [x, y, z]
%   - lineModel: 1x6 vector [x0, y0, z0, dx, dy, dz] where:
%       - [x0, y0, z0] is a point on the line
%       - [dx, dy, dz] is the direction vector of the line (should be normalized)
%
%   Output:
%   - distances: Nx1 vector of distances from each point to the line

% Extract line parameters
linePoint = lineModel(1:3);
lineDirection = lineModel(4:6);

% Ensure the direction vector is normalized
lineDirection = lineDirection / norm(lineDirection);

% Calculate the vector from the line point to each point
vectors = points - repmat(linePoint, size(points, 1), 1);

% Project these vectors onto the line direction
projections = vectors * lineDirection';

% Calculate the projected points on the line
projectedPoints = repmat(linePoint, size(points, 1), 1) + projections * lineDirection;

% Calculate the distances
distances = sqrt(sum((points - projectedPoints).^2, 2));
end