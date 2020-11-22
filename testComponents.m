%% SCRIPT FOR TESTING INDIVIDUAL COMPONENTS
% Author: Manan Shah

%% Reset workspace
close all
clear
clc

%% Occupancy map generation
map = genOccMap();
plotOccMap(map);