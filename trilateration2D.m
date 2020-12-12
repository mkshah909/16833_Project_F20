function position = trilateration2D(points, ranges)

A = [2.*points(2:end, 1) - 2.*points(1:(end-1), 1),...
    2.*points(2:end, 2) - 2.*points(1:(end-1), 2)];

ranges = (ranges.^2)';

b = ranges(1:(end-1)) - ranges(2:end) - (points(1:(end-1), 1)).^2 +...
    (points(2:end, 1)).^2 - (points(1:(end-1), 2)).^2 + (points(2:end, 2)).^2;

idx = true(size(A,1), 1);
idx(isnan(A(:,1)) | isnan(A(:,2))) = false;
A(isnan(A(:,1)) | isnan(A(:,2)), :) = [];
b = b(idx);

position = pinv(A)*b;

end