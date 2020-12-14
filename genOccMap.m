function map = genOccMap()
% Function to generate occumpancy map
% {Priority 3} [input] genMode: 1=saved map, else random (random gen will be developed
%                  as p3
% [output] map: (300, 800) binary matrix, with ones in the bottom half to
%               represent ground, can be directly plotted as a scatter plot
%               Each pixel represents 1 cm. 8m x 3m map of ground

map = zeros(300, 800);
currentHeight = 25; % [cm]
currentX = 100;
stepHeight = 10; % [cm]
stepWidth = 25; % [cm]
numStepUp = 7;
numStepDown = 3;

% populate starting flat ground
map(1:currentHeight, 1:currentX) = 1;

% populate some steps up
for i = 1:numStepUp
    map(1:currentHeight+stepHeight, currentX+1:currentX+stepWidth) = 1;
    currentHeight = currentHeight + stepHeight;
    currentX = currentX + stepWidth;
end

% populate some more flat ground
map(1:currentHeight, currentX+1:currentX+50) = 1;
currentX = currentX + 50;

% populate some steps down
for i = 1:numStepDown
    map(1:currentHeight-stepHeight, currentX+1:currentX+stepWidth) = 1;
    currentHeight = currentHeight - stepHeight;
    currentX = currentX + stepWidth;
end

% populate some more flat ground
map(1:currentHeight, currentX+1:currentX+50) = 1;
currentX = currentX + 50;

% populate obstacle
map(1:currentHeight+round(stepHeight*1.5),...
    currentX+1:currentX+round(stepWidth*0.5)) = 1;
currentX = currentX + round(stepWidth*0.5);

% populate some more flat ground
map(1:currentHeight, currentX+1:currentX+75) = 1;
currentX = currentX+75;

% populate obstacle
map(1:currentHeight+round(stepHeight*0.5), currentX+1:currentX+round(stepWidth*1.5)) = 1;
currentX = currentX + round(stepWidth*1.5);

% populate some more flat ground
map(1:currentHeight, currentX+1:currentX+50) = 1;
currentX = currentX+50;

rampLength = 50;

% populate ramp up
for i = 1:rampLength
    map(1:round(currentHeight+0.5), currentX+1:currentX+2) = 1;
    currentHeight = round(currentHeight + 0.5);
    currentX = currentX + 1;
end

% populate till end of map
map(1:currentHeight, currentX+1:size(map,2)) = 1;

end