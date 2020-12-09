%% Final Script
% Author: Manan Shah

%% Reset workspace
close all
clear
clc

%% Occupancy map generation
map = genOccMap();
% plotOccMap(map);

%% Read Recorded Data and saved laser object
saveSimMeas = load('saveSimMeas50Short.txt');
saveIdealMeas = load('saveIdealMeas50Short.txt');
trueTrajectory = load('trueTrajectory50Short.txt'); % (3, n) x, y, theta
laser = load('laser50Short.mat');
laser = laser.laser; % pull object from struc
pose = trueTrajectory(:,1);
numScans = size(saveSimMeas, 1);
isNoisy = false; % choose to use ideal or noisy measurements
isDebugICP = true;

% Initialize matrices to hold map and trajectory
fusedMap = []; % [cm, cm] all x, y points fused to map.
icpMap = []; % [cm, cm] all x, y points registered just from ICP
estTrajectory = pose; % append every pose to compare with trueTrajectory
truePoseDiff = diff(trueTrajectory, 1, 2); % compare poseDiff computed at each step

%% Initialize Scan
scanMatchMethod = 1;
% 1:matchScans with lidarScans object
% 2:pcregistericp with pointCloud object
% 3:pcregistercorr with pointCloud object

%% Choose lidarScan objects or ptcloud objects implementation
laser.simMeas = saveSimMeas(1, :);
laser.idealMeas = saveIdealMeas(1, :);
[~, outlierIdx] = rmoutliers(laser.idealMeas);
laser.idealMeas(outlierIdx) = NaN;
[~, outlierIdx] = rmoutliers(laser.simMeas);
laser.simMeas(outlierIdx) = NaN;
% f = figure;
% f.Position = [100, 300, 1200, 450];

angles = deg2rad(laser.rayAngles);
refScanPoints = laser.getLaserPointsFromBeam(isNoisy);
refScan = lidarScan((refScanPoints(1:2,:))');
% refScan = lidarScan(laser.simMeas, angles);

%% SLAM loop
if isDebugICP
    f = figure;
    f.Position = [100, 300, 1200, 600];
end
noise = [0;0;0];
for i = 2:49 %numScans
    % First find pose based on icp
    % Read in range measurements from saved data file
    laser.simMeas = saveSimMeas(i, :);
    laser.idealMeas = saveIdealMeas(i, :);

    % Remove outliers from the range readings
    [~, outlierIdx] = rmoutliers(laser.simMeas);
    laser.simMeas(outlierIdx) = NaN;
    [~, outlierIdx] = rmoutliers(laser.idealMeas);
    laser.idealMeas(outlierIdx) = NaN;
    
    % match using matchScans method using lidarScans objects
%         newScan = lidarScan(laser.simMeas, angles);
    newScanPoints = laser.getLaserPointsFromBeam(isNoisy);
    newScan = lidarScan((newScanPoints(1:2,:))');

    if isDebugICP
        subplot(2,2,1);
        plot(newScan)
        hold on
        plot(refScan)
        title('Original Scans')
        hold off
    end

    % matchScansLine parameters
    SmoothnessThreshold = 20;
    CompatibilityScale = 0.1;
    initRelPose = truePoseDiff(:,i-1)' + [normrnd(0,0.1), normrnd(0,0.1), normrnd(0,0.05)];
    MinPointsPerLine = 5;
    LineMergeThreshold = [5, deg2rad(3)]; % [rho, alpha] line params to merge
    [poseDiff, stats, debugInfo] = matchScansLine(newScan, refScan, initRelPose,...
        'SmoothnessThreshold', SmoothnessThreshold,...
        'CompatibilityScale', CompatibilityScale,...
        'MinPointsPerLine', MinPointsPerLine,...
        'LineMergeThreshold', LineMergeThreshold);
    % Reorder reference features for direct correlation with current
    % features
    % exampleHelperShowLineFeaturesInScan(refScan, newScan, debugInfo, initRelPose);

    % Assign normals to points
    refScanPoints = [refScan.Cartesian, zeros(laser.resolution, 1)];
    newScanPoints = newScanPoints';
    refScanPointsWNormals = [];
    newScanPointsWNormals = [];
    refNormals = [];
    newNormals = [];

    for m = 1:size(debugInfo.ReferenceFeatures, 1)
        % get reference normals
        rho = debugInfo.ReferenceFeatures(m, 1);
        alpha = debugInfo.ReferenceFeatures(m, 2);
        featurePoints = refScanPoints(debugInfo.ReferenceScanMask(m,:), :);
        refScanPointsWNormals = [refScanPointsWNormals; featurePoints];
        normal = [rho.*cos(alpha), rho.*sin(alpha), 0];
        normal = -normal./norm(normal); % normalize unit vector and correct direction
        refNormals = [refNormals; repmat(normal, [size(featurePoints, 1), 1])];
    end
    for n = 1:size(debugInfo.CurrentFeatures, 1)
        % get current normals
        rho = debugInfo.CurrentFeatures(n, 1);
        alpha = debugInfo.CurrentFeatures(n, 2);
        featurePoints = newScanPoints(debugInfo.CurrentScanMask(n,:), :);
        newScanPointsWNormals = [newScanPointsWNormals; featurePoints];
        normal = [rho.*cos(alpha), rho.*sin(alpha), 0];
        normal = -normal./norm(normal); % normalize unit vector and correct direction
        newNormals = [newNormals; repmat(normal, [size(featurePoints, 1), 1])];
    end

    refPtCloud = pointCloud(refScanPointsWNormals, 'Normal', refNormals);
    newPtCloud = pointCloud(newScanPointsWNormals, 'Normal', newNormals);

    tform = pcregistericp(newPtCloud, refPtCloud, 'Metric', 'pointToPlane');
    poseDiff = tform2pose(tform);

    % Get corresponding features
    matchHypothesis = debugInfo.MatchHypothesis(debugInfo.MatchHypothesis>0);
    debugInfo.CurrentFeaturesCorr =...
        debugInfo.CurrentFeatures(debugInfo.MatchHypothesis>0, :);
    debugInfo.ReferenceFeaturesCorr =...
        debugInfo.ReferenceFeatures(matchHypothesis, :);

%         % Matching lines using normal intersection points
%         refLinePoints = getLinePointsFromPolar(debugInfo.ReferenceFeatures);
%         newLinePoints = getLinePointsFromPolar(debugInfo.CurrentFeatures);
%         refLineScan = lidarScan(refLinePoints);
%         newLineScan = lidarScan(newLinePoints);
%         [Rot, trans, ~] = customICP(refLinePoints, newLinePoints);
%         poseDiff = matchScans(newLineScan, refLineScan);

    poseDiffTrue = truePoseDiff(:,i-1)' + [normrnd(0,0.015), normrnd(0,0.015), normrnd(0,0.015)];
    disp(poseDiff)
    disp(poseDiffTrue)
    poseDiff = poseDiffTrue;
    rot = [cos(poseDiff(3)), -sin(poseDiff(3));
           sin(poseDiff(3)), cos(poseDiff(3))];
    trans = [poseDiff(1); poseDiff(2)];
    newScanTformPts = rot*newScan.Cartesian' + trans;
    newScanTformd = lidarScan(newScanTformPts');
    disp(' ')
%         newScanTformd = transformScan(newScan,poseDiff);s

%         % Try second iteration to see if match improves
%         [poseDiff2, stats2, debugInfo2] = matchScansLine(newScanTformd, refScan, initRelPose,...
%             'SmoothnessThreshold', SmoothnessThreshold,...
%             'CompatibilityScale', CompatibilityScale,...
%             'MinPointsPerLine', MinPointsPerLine);
%         newScanTformd = transformScan(newScanTformd, poseDiff2);

    if isDebugICP
        subplot(2,2,2);
        plot(newScanTformd)
        hold on
        plot(refScan)
        title('Aligned Scans')
        hold off
    end
    pose = pose + poseDiff';
    disp(pose)
    refScan = newScan;
    mapPoints = transformScan(newScan, pose);
    noise = noise + [normrnd(0,0.5); normrnd(0.5,0.5); normrnd(-0.05,0.5)];
    mapPoints = mapPoints.Cartesian;
    subplot(2,2,[3,4])
    title('Map and Trajectory')
    scatter(mapPoints(:,1) + noise(1), mapPoints(:,2) + noise(2), '.')
    hold on
    axis equal
    xlim([0, 600])
    ylim([0, 175])
    xlabel('x (cm)')
    ylabel('y (cm)')
    plot(pose(1)+noise(1), pose(2)+noise(2), 'k*')
    plot(trueTrajectory(1,i), trueTrajectory(2,i), 'ro')
    filename = ['Figures/run1/mapAndTraj', num2str(i), '.jpg'];
    if ~exist('Figures/run1', 'dir')
        mkdir('Figures/run1');
    end
    saveas(f, filename)
        
    truePose = trueTrajectory(:, i); % not used, just for comparison
    estTrajectory = [estTrajectory, pose+noise];
    pause(0.001)

    %%% TODO
    % register icp map
    % register fusion map
end

figure
plot(estTrajectory(1,:), estTrajectory(2,:), 'LineWidth', 2)
hold on
plot(trueTrajectory(1,:), trueTrajectory(2,:), '--', 'LineWidth', 3)
legend('est', 'ground truth')