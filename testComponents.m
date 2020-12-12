%% SCRIPT FOR TESTING INDIVIDUAL COMPONENTS
% Author: Manan Shah

%% Reset workspace
close all
clear
clc

%% Occupancy map generation
map = genOccMap();
plotOccMap(map);

%% Test generation of IMU poses
filenameSuffix = '12111515';
saveNewData = true;
numPoints = 200;
yPoints = [100; 110; 125; 140]; % 150; 170; 155; 200];
xPoints = [25; 100; 200; 300]; % 400; 500; 600; 700];
trajPoly = polyfit(xPoints, yPoints, 3);
xTraj = linspace(25,300,numPoints);
yTraj = polyval(trajPoly, xTraj)...
    + 5*sin(0.05*linspace(1, 300, numPoints));
zTraj = zeros(1, numPoints);
plot(xTraj, yTraj)
title('Visualize occupancy map with trajectory');
theta = deg2rad(-45 + 2*sin(0.1*linspace(1, 600, numPoints)));
trueTrajectory = [xTraj; yTraj; theta];
if saveNewData
    writematrix(trueTrajectory, ['Data/trueTrajectory', filenameSuffix, '.txt'])
end
% % Attempt to simulate IMU sensor, P4
% duration = 4; % [s], total duration of sim
% tstep = 0.002; % [s], time steps
% jerkX = rand(duration/tstep, 1) - 0.5;
% jerkY = rand(duration/tstep, 1)*2 - 1;
% accelX = 0;

%% Test LaserRangeFinder class and ideal measurements (ray casting) w/ plotting
saveSimMeas = [];
saveIdealMeas = [];
resolution = 180;
featureInterval = 3;
planeSmoothThres = 0.7;
edgeSmoothThres = 0.1;
sigmaLaser = 0.5;
isNoisy = true;
laserSpan = 90; % [deg]
laser = LaserRangeFinder(700, map, sigmaLaser, resolution, laserSpan, featureInterval); % initialize laser range finder obj
if saveNewData
    laserFilename = ['Data/laser', filenameSuffix, '.mat'];
    save(laserFilename, 'laser')
end
f = figure;
f.Position = [100, 300, 1200, 450];
for i = 1:numPoints
    pose = [xTraj(i), yTraj(i), theta(i)];
    plotOccMap(map);
    plot(xTraj, yTraj, 'g')
    laser.readIdealLaser(pose, laserSpan);
    worldPoints = laser.getWorldPointsFromBeam(pose, isNoisy);
    laser.visualizeSensor(pose, worldPoints, true);
%     laser.updateSmoothness(planeSmoothThres, edgeSmoothThres);
    saveSimMeas = [saveSimMeas; laser.simMeas'];
    saveIdealMeas = [saveIdealMeas; laser.idealMeas'];
    pause(0.00001)
    hold off
end

if saveNewData
    writematrix(saveSimMeas, ['Data/saveSimMeas', filenameSuffix, '.txt'])
    writematrix(saveIdealMeas, ['Data/saveIdealMeas', filenameSuffix, '.txt'])
end

% f2 = figure;
% f2.Position = [100, 300, 1200, 450];
% hold on
% for i = 1:numPoints
%     scatter(laser.rawXWorld(i,:), laser.rawYWorld(i,:), '.')
% end
% title('Scatter plot of all measured points with known pose of sensor')

%% List of data files saved (suffixes listed here) and their specs
%%%% All with span 90, resolution 180
% 200WD: 200 points, frequency of y and theta 0.1, wide span

%%%% All with span 75, resolution 150
% 500HD: 500 points, frequency of y and theta 0.4

%%%% All with span 75, resolution 75
% 50LF: 50 points, frequency of y and theta 0.05, reduced amplitude as well
% 200LF: 200 points, frequency of y and theta 0.1
% 500: 500 points, frequency of y and theta 0.4
% 5000: 5000 points, frequency of y and theta 0.4
% LF: 1000 points, frequency of y and theta 0.1
