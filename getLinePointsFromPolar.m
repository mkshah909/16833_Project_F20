function linePoints = getLinePointsFromPolar(features)
% features is nx2 [rho, alpha]
% rho is length of normal to line passing through origin
% alpha is angle of normal to positive x-axis, in rad
% linePoints is nx2 points [x, y]

rho = features(:, 1);
alpha = features(:, 2);

linePoints = [rho.*cos(alpha), rho.*sin(alpha)];

end