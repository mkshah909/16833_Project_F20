%% TEST LIDAR SCAN MATCHING INDIVIDUALLY
% Use this script as a testing ground for different scan matching
% algorithms
% Author: Manan Shah

%% Reset workspace
close all
clear
clc

%% Read Recorded Data and saved laser object
saveSimMeas = load('Data/saveSimMeas50Short.txt');
saveIdealMeas = load('Data/saveIdealMeas50Short.txt');
trueTrajectory = load('Data/trueTrajectory50Short.txt'); % (3, n) x, y, theta
laser = load('Data/laser50Short.mat');
laser = laser.laser; % pull object from struc
pose = trueTrajectory(:,1);
numScans = size(saveSimMeas, 1);
isNoisy = true; % choose to use ideal or noisy measurements

truePoseDiff = diff(trueTrajectory, 1, 2); % compare poseDiff computed at each step

%% Initialize lidarScan objects
% Read first reference scans
laser.simMeas = saveSimMeas(1, :);
laser.idealMeas = saveIdealMeas(1, :);

% Remove outliers from range readings
[~, outlierIdx] = rmoutliers(laser.idealMeas);
laser.idealMeas(outlierIdx) = NaN;
[~, outlierIdx] = rmoutliers(laser.simMeas);
laser.simMeas(outlierIdx) = NaN;

% Initialize scans
angles = deg2rad(laser.rayAngles) - pi/2;
refScan = lidarScan(laser.idealMeas, angles);

%% SLAM loop
figure
for i = 2:49 %numScans
    % Read in range measurements from saved data file
    laser.simMeas = saveSimMeas(i, :);
    laser.idealMeas = saveIdealMeas(i, :);

    % Remove outliers from the range readings
    [~, outlierIdx] = rmoutliers(laser.simMeas);
    laser.simMeas(outlierIdx) = NaN;
    [~, outlierIdx] = rmoutliers(laser.idealMeas);
    laser.idealMeas(outlierIdx) = NaN;
    
    % create t+1 lidar scan
    newScan = lidarScan(laser.idealMeas, angles);

    subplot(2,2,1);
    plot(newScan)
    hold on
    plot(refScan)
    title('Original Scans')
    hold off
    
%     %%% matchScansGrid parameters
%     Resolution = 20; % grid cells per meter
%     MaxRange = 150; % max range of lidar
%     TranslationSearchRange = [5, 10];
%     RotationSearchRange = pi/24;
%     poseDiff = matchScansGrid(newScan, refScan,...
%         'MaxRange', MaxRange, 'TranslationSearchRange', TranslationSearchRange,...
%         'RotationSearchRange', RotationSearchRange);
    %%% Notes about matchScansGrid performance
    % Crashes, even with low resolution

%     %%% matchScansLine parameters
%     SmoothnessThreshold = 20;
%     CompatibilityScale = 0.08;
%     initRelPose = [0,0,0];
%     MinPointsPerLine = 5;
%     LineMergeThreshold = [8, deg2rad(3)]; % [rho, alpha] line params to merge
%     [poseDiff, stats, debugInfo] = matchScansLine(newScan, refScan, initRelPose,...
%         'SmoothnessThreshold', SmoothnessThreshold,...
%         'CompatibilityScale', CompatibilityScale,...
%         'MinPointsPerLine', MinPointsPerLine,...
%         'LineMergeThreshold', LineMergeThreshold);

    %%% Custom point-to-plane ICP
    [poseDiff valid_pair_num error] = pointToPlaneICP(newScan, refScan);
    
    % plot scans with lines marked and labeled
%     exampleHelperShowLineFeaturesInScan(s1, s2, debugInfo, initGuess);
%     % Reorder reference features for direct correlation with current
%     % features
%     matchHypothesis = debugInfo.MatchHypothesis(debugInfo.MatchHypothesis>0);
%     debugInfo.CurrentFeatures =...
%         debugInfo.CurrentFeatures(debugInfo.MatchHypothesis>0, :);
%     debugInfo.ReferenceFeaturesCorr =...
%         debugInfo.ReferenceFeatures(matchHypothesis, :);

    poseDiffTrue = truePoseDiff(:,i-1)'; % + [normrnd(0,0.1), normrnd(0,0.1), normrnd(0,0.1)];
    disp(poseDiff)
    disp(poseDiffTrue)
    disp(' ')
    newScanTformd = transformScan(newScan,poseDiff);

    subplot(2,2,2);
    plot(newScanTformd)
    hold on
    plot(refScan)
    title('Aligned Scans')
    hold off
    
%     pose = pose + poseDiff';
%     disp(pose)
%     refScan = newScan;
%     mapPoints = transformScan(newScan, pose);
%     mapPoints = mapPoints.Cartesian;
%     subplot(2,2,[3,4])
%     scatter(mapPoints(:,1), mapPoints(:,2), '.')
%     axis equal
%     xlim([0, 600])
%     ylim([0, 250])
%     xlabel('x (cm)')
%     ylabel('y (cm)')
%     hold on
%     truePose = trueTrajectory(:, i); % not used, just for comparison
%     estTrajectory = [estTrajectory, pose];
%     pause(0.001)
end

figure
plot(estTrajectory(1,:), estTrajectory(2,:), 'LineWidth', 2)
hold on
plot(trueTrajectory(1,:), trueTrajectory(2,:), '--', 'LineWidth', 3)
legend('est', 'ground truth')