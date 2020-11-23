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
numPoints = 1000;
yPoints = [100; 75; 125; 160; 150; 170; 155; 200];
xPoints = [25; 100; 200; 300; 400; 500; 600; 700];
trajPoly = polyfit(xPoints, yPoints, 5);
xTraj = linspace(25,600,numPoints);
yTraj = polyval(trajPoly, xTraj)...
    + 15*sin(0.4*linspace(1, 600, numPoints));
plot(xTraj, yTraj)
theta = deg2rad(-52.5...
    + 22.5*sin(0.4*linspace(1, 600, numPoints)));
% duration = 4; % [s], total duration of sim
% tstep = 0.002; % [s], time steps
% jerkX = rand(duration/tstep, 1) - 0.5;
% jerkY = rand(duration/tstep, 1)*2 - 1;
% accelX = 0;

%% Test LaserRangeFinder class and ideal measurements (ray casting) w/ plotting
laser = LaserRangeFinder(700, map, 0);
laserSpan = 75; % [deg]
f = figure;
f.Position = [100, 300, 1200, 450];
for i = 1:numPoints
    pose = [xTraj(i), yTraj(i), theta(i)];
    plotOccMap(map);
    plot(xTraj, yTraj, 'g')
    laser.readIdealLaser(pose, laserSpan, 45);
    laser.visualizeSensor(pose, true);
    pause(0.00001)
    hold off
end

