function [poseDiffFinal, valid_pair_num, error] = pointToPlaneICP(newScan, refScan)
% newScan and refScan are lidarScan objects
numIter = 10;
errThres = 4e-1;
poseDiffFinal = [1;1;0];
newScan = transformScan(newScan, poseDiffFinal);

% First, compute normals to prep for point-to-plane ICP
refScanPointsWNormals = [];
refNormalsUnfilt = [];

%%% matchScansLine used to obtain line features
SmoothnessThreshold = 20;
initRelPose = [0;0;0];
MinPointsPerLine = 5;
[~, ~, debugInfo] = matchScansLine(newScan, refScan, initRelPose,...
    'SmoothnessThreshold', SmoothnessThreshold,...
    'MinPointsPerLine', MinPointsPerLine);

refScanPoints = refScan.Cartesian;

for m = 1:size(debugInfo.ReferenceFeatures, 1)
    % get reference normals
    rho = debugInfo.ReferenceFeatures(m, 1);
    alpha = debugInfo.ReferenceFeatures(m, 2);
    featurePoints = refScanPoints(debugInfo.ReferenceScanMask(m,:), :);
    refScanPointsWNormals = [refScanPointsWNormals; featurePoints];
    normal = [rho.*cos(alpha), rho.*sin(alpha)];
    normal = -normal./norm(normal); % normalize unit vector and correct direction
    refNormalsUnfilt = [refNormalsUnfilt; repmat(normal, [size(featurePoints, 1), 1])];
end

i = 0;
error = 1;

while i < numIter && error > errThres
    i = i + 1;
    refScanPoints = refScanPointsWNormals;
    refNormals = refNormalsUnfilt;
    newScanPoints = newScan.Cartesian;

    % Next, find nearest points
    [idx, dist] = dsearchn(newScanPoints, refScanPoints);

    % For repeated indices, select the one with shorter distance
    [~, I] = sort(dist);
    sortIdx = idx(I);
    refScanPoints = refScanPoints(I, :);
    refNormals = refNormals(I, :);

    [uniqueIdx, ia] = unique(sortIdx);
    refScanPoints = refScanPoints(ia, :);
    refNormals = refNormals(ia, :);
    newScanPoints = newScanPoints(uniqueIdx, :);

    valid_pair_num = length(uniqueIdx);

    % Create A and b matrices
    A = zeros(valid_pair_num, 3);
    A(:, 1:2) = refNormals;
    A(:, end) = newScanPoints(:,1).*refNormals(:,2) - newScanPoints(:,2).*refNormals(:,1);

    b = refNormals(:,1).*(refScanPoints(:,1) - newScanPoints(:,1)) + ...
        refNormals(:,2).*(refScanPoints(:,2) - newScanPoints(:,2));

    poseDiff = pinv(A)*b;
    newScan = transformScan(newScan, poseDiff);
    error = sqrt(sum((A*poseDiff - b).^2)/valid_pair_num);
    
    subplot(2,2,2);
    plot(newScan)
    hold on
    plot(refScan)
    title('Aligned Scans')
    hold off
    poseDiffFinal = poseDiffFinal + poseDiff;
end

if error > errThres
    disp('error threshold not met')
end

end