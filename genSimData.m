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
filenameSuffix = '12141834'; % mmddhhmm
saveNewData = true;
numPoints = 75;
yPoints = [75; 100; 150; 170];% [170; 150; 160; 130]; % 150; 170; 155; 200];
xPoints = [25; 100; 200; 300]% [300; 375; 475; 575]; % 400; 500; 600; 700];
trajPoly = polyfit(xPoints, yPoints, 3);
xTraj = linspace(25,300,numPoints);
yTraj = polyval(trajPoly, xTraj)...
    + 6*sin(0.1*linspace(1, 300, numPoints));
zTraj = zeros(1, numPoints);
plot(xTraj, yTraj, 'g')
title('Visualize occupancy map with trajectory');
theta = deg2rad(-45 + 5*sin(0.1*linspace(1, 600, numPoints)));
trueTrajectory = [xTraj; yTraj; theta];
if saveNewData
    writematrix(trueTrajectory, ['Data/trueTrajectory', filenameSuffix, '.txt'])
end

%% Test LaserRangeFinder class and ideal measurements (ray casting) w/ plotting
saveSimMeas = [];
saveIdealMeas = [];
laserSpan = 75; % [deg]
resolution = 150;
featureInterval = 3;
planeSmoothThres = 0.7;
edgeSmoothThres = 0.1;
sigmaLaser = 0.5;
isNoisy = true;
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
    
    if saveNewData
        filename = ['Figures/sim', filenameSuffix, '/gridAndCast', num2str(i), '.jpg'];
        if ~exist(['Figures/sim', filenameSuffix], 'dir')
            mkdir(['Figures/sim', filenameSuffix]);
        end
        saveas(f, filename)
    end
    pause(0.00001)
    hold off
end

if saveNewData
    writematrix(saveSimMeas, ['Data/saveSimMeas', filenameSuffix, '.txt'])
    writematrix(saveIdealMeas, ['Data/saveIdealMeas', filenameSuffix, '.txt'])
end

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
