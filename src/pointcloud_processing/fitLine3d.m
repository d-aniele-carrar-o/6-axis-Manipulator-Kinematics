function lineModel = fitLine3d(x, y, z)
% FITLINE3D Fit a 3D line to a set of points using least squares
%   lineModel = fitLine3d(x, y, z) fits a 3D line to the points (x,y,z)
%   and returns the line model as [point, direction] where:
%   - point is a point on the line [x0, y0, z0]
%   - direction is the unit direction vector [dx, dy, dz]
%
%   The returned lineModel is a 1x6 vector [x0, y0, z0, dx, dy, dz]

% Check inputs
if length(x) < 2 || length(y) < 2 || length(z) < 2
    error('At least 2 points are required to fit a line');
end

% Compute the centroid of the points
centroid = [mean(x), mean(y), mean(z)];

% Create the data matrix for PCA
data = [x - centroid(1), y - centroid(2), z - centroid(3)];

% Perform SVD to find the principal direction
[~, ~, V] = svd(data, 0);
direction = V(:, 1)';

% Ensure the direction vector is a unit vector
direction = direction / norm(direction);

% Return the line model as [point, direction]
lineModel = [centroid, direction];
end
