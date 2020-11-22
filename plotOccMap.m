function plotOccMap(map)
% generate plot to show occupancy map created by genOccMap.m
figure()
hold on
axis equal
X = [];
Y = [];

for y = 1:size(map, 1)
    for x = 1:size(map, 2)
        if map(y, x) == 1
%             scatter(x, y, 5,'MarkerEdgeColor',[0 0 0], 'MarkerFaceColor',[0 0 0])
            X = [X, x];
            Y = [Y, y];
        end
    end
end
xlim([0, x])
ylim([0,y])
scatter(X, Y, 10,'MarkerEdgeColor',[0 0 0], 'MarkerFaceColor',[0 0 0])
xlabel('x (cm)')
ylabel('y (cm)')

end