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
saveSimMeas = load('Data/saveSimMeas50Short.txt');
saveIdealMeas = load('Data/saveIdealMeas50Short.txt');
trueTrajectory = load('Data/trueTrajectory50Short.txt'); % (3, n) x, y, theta
laser = load('Data/laser50Short.mat');
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

if scanMatchMethod == 1
    angles = deg2rad(laser.rayAngles);
    refScanPoints = laser.getLaserPointsFromBeam(isNoisy);
    refScan = lidarScan((refScanPoints(1:2,:))');
    % refScan = lidarScan(laser.simMeas, angles);
else
    initLaserPoints = laser.getLaserPointsFromBeam(isNoisy);
    icpMap = [icpMap;
              initLaserPoints'];
    refPtCloud = pointCloud(initLaserPoints');
    currentTform = pose2tform(pose);
    refPtCloud = pctransform(refPtCloud, currentTform);
end

%% SLAM loop
if isDebugICP
    figure
end
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
    
    if scanMatchMethod==1
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
        exampleHelperShowLineFeaturesInScan(refScan, newScan, debugInfo, initGuess);
        % Reorder reference features for direct correlation with current
        % features
        matchHypothesis = debugInfo.MatchHypothesis(debugInfo.MatchHypothesis>0);
        debugInfo.CurrentFeatures =...
            debugInfo.CurrentFeatures(debugInfo.MatchHypothesis>0, :);
        debugInfo.ReferenceFeaturesCorr =...
            debugInfo.ReferenceFeatures(matchHypothesis, :);

        poseDiffTrue = truePoseDiff(:,i-1)'; % + [normrnd(0,0.1), normrnd(0,0.1), normrnd(0,0.1)];
        disp(poseDiff)
        disp(poseDiffTrue)
        disp(' ')
        newScanTformd = transformScan(newScan,poseDiff);
        
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
        mapPoints = mapPoints.Cartesian;
        subplot(2,2,[3,4])
        scatter(mapPoints(:,1), mapPoints(:,2), '.')
        axis equal
        xlim([0, 600])
        ylim([0, 250])
        xlabel('x (cm)')
        ylabel('y (cm)')
        hold on
    else

        % Convert range readings to laser x, y coordinates
        laserPointsEst = laser.getLaserPointsFromBeam(isNoisy);
        inputPtCloud = pointCloud(laserPointsEst');

        % Convert laser x, y coordinates to estimated world x,y using last
        % transform
        newPtCloud2 = pctransform(inputPtCloud, currentTform);

    %     % Downsample to increase registration accuracy
    %     refPtCloud = pcdownsample(refPtCloud, 'gridAverage', 2);
    %     newPtCloud2 = pcdownsample(newPtCloud2, 'gridAverage', 2);

        % Match scans using ICP
    %     [tform, ~, rmse] = pcregistericp(newPtCloud2, refPtCloud, 'MaxIterations', 100);
    %     disp(rmse)
    %     tform = pcregistericp(newPtCloud2, refPtCloud);
    %     [R, T, ~] = customICP(refPtCloud.Location, newPtCloud2.Location,...
    %         200,5,0,1e-5); %maxIter,minIter,critFun,thres
    %     RTmat = [R, T;
    %             zeros(1,3), 1];
    %     tform = affine3d(RTmat');

        [tform, rmse] = pcregistercorr(newPtCloud2, refPtCloud, 500, 0.5);

        currentTform = affine3d(tform.T*currentTform.T);

        % Transform ray laser x, y points with updated transform
        updatedPtCloud = pctransform(inputPtCloud, currentTform);

        % Save pose information for plotting only
        pose = tform2pose(currentTform);
    
        % TROUBLESHOOTING FIGURES
        if isDebugICP
    %         scatter(inputPtCloud.Location(:,1), inputPtCloud.Location(:,2), 'b.')
            scatter(newPtCloud2.Location(:,1), newPtCloud2.Location(:,2), 'g.')
            hold on
            scatter(refPtCloud.Location(:,1), refPtCloud.Location(:,2), 'r.')
            scatter(updatedPtCloud.Location(:,1), updatedPtCloud.Location(:,2), 'k.')
            plot(pose(1), pose(2), '*')
            plot(truePose(1), truePose(2), '^')
            legend('estNewPoints', 'refPoints', 'correctedNewPoints',...
                'SensorPoseEst', 'SensorPoseTrue')
            hold off
        end
        refPtCloud = updatedPtCloud;
    end
    truePose = trueTrajectory(:, i); % not used, just for comparison
    estTrajectory = [estTrajectory, pose];
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