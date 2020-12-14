%% Run 2D LiDAR SLAM Implementation from saved data
% Use this script as a testing ground for different scan matching
% algorithms
% Author: Manan Shah

%% Reset workspace
close all
clear
clc

%% Read Recorded Data and saved laser object
filenameSuffix = '12111945';
saveSimMeas = load(['Data/saveSimMeas', filenameSuffix, '.txt']);
saveIdealMeas = load(['Data/saveIdealMeas', filenameSuffix, '.txt']);
trueTrajectory = load(['Data/trueTrajectory', filenameSuffix, '.txt']); % (3, n) x, y, theta
laser = load(['Data/laser', filenameSuffix, '.mat']);
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
truePose = trueTrajectory(:, 1); % not used, just for comparison
estTrajectory = pose;

f = figure;
f.Position = [100, 100, 1200, 400];

mapPoints = transformScan(refScan, pose);
mapPoints = mapPoints.Cartesian;
subplot(2,2,[3,4])
scatter(mapPoints(:,1), mapPoints(:,2), '.')
axis equal
xlim([0, 600])
ylim([0, 250])
xlabel('x (cm)')
ylabel('y (cm)')
hold on
scatter(pose(1), pose(2), '*')
scatter(truePose(1), truePose(2), 'o')

for i = 2:numScans
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
    legend('New Scan', 'Reference Scan')
    hold off

    %%% Custom point-to-plane ICP
    [poseDiff valid_pair_num error] = pointToPlaneICP(newScan, refScan);

    poseDiffTrue = truePoseDiff(:,i-1)';
    newScanTformd = transformScan(newScan,poseDiff);

    subplot(2,2,2);
    plot(newScanTformd)
    hold on
    plot(refScan)
    title('Aligned Scans')
    hold off
    
    mapPoints = transformScan(newScanTformd, pose);
    mapPoints = mapPoints.Cartesian;
    
    position = trilateration2D(mapPoints, laser.idealMeas);
    pose(1:2) = position;
    pose(3) = pose(3) + poseDiff(3);
    truePose = trueTrajectory(:, i); % not used, just for comparison
    estTrajectory = [estTrajectory, pose];
    
    subplot(2,2,[3,4])
    scatter(mapPoints(:,1), mapPoints(:,2), 'g.')
    axis equal
    xlim([0, 900])
    ylim([0, 250])
    xlabel('x (cm)')
    ylabel('y (cm)')
    hold on
    scatter(pose(1), pose(2), 'b*')
    scatter(truePose(1), truePose(2), 'ro')
    legend('Plotted map', 'est pose', 'true pose')
    filename = ['Figures/run1/mapAndTraj', num2str(i), '.jpg'];
    if ~exist('Figures/run1', 'dir')
        mkdir('Figures/run1');
    end
    saveas(f, filename)
    
    % Update refScan for next step
    refScan = newScan;
    pause(0.001)
end

figure
plot(estTrajectory(1,:), estTrajectory(2,:), 'LineWidth', 2)
hold on
plot(trueTrajectory(1,:), trueTrajectory(2,:), '--', 'LineWidth', 3)
legend('est', 'ground truth')