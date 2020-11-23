% Idea Laser Class definition

classdef LaserRangeFinder < handle
    properties
        occMap
        range
        rayVec
        variance
        idealMeas
        mapX
        mapY
        sensorIndices
    end % end properties
    methods
        function laser = LaserRangeFinder(range, occMap, variance)
            laser.range = range;
            laser.occMap = occMap;
            laser.rayVec = linspace(1, range, range);
            laser.variance = variance;
            [laser.mapY, laser.mapX] = size(occMap);
        end % end init
        function readIdealLaser(obj, pose, span, resolution)
            % Ray casting to generate ideal laser measurements, given laser pose
            % [input] pose: [x,y,theta] of laser in world frame
            % [input] span: scalar degrees, span of laser scan
            % [input] resolution: scalar, number of readings between 0-179 deg
            % modifies obj.laserMeas: (resolution, 1) vectory of laser range readings

            obj.idealMeas = zeros(resolution,1);
            visibility = zeros(1, obj.range);
            sensorIndices = round(linspace(90 - span/2, 90 + span/2, resolution));
            obj.sensorIndices = sensorIndices;

            for i = 1:resolution
                % computes all possible x,y of current ray up to max range
                xRayEnd = pose(1) + obj.rayVec*cos(-pi/2 + pose(3)...
                    + deg2rad(sensorIndices(i)+1));
                yRayEnd = pose(2) + obj.rayVec*sin(-pi/2 + pose(3)...
                    + deg2rad(sensorIndices(i)+1));
                xRayEnd = round(xRayEnd);
                yRayEnd = round(yRayEnd);
                xRayEnd(xRayEnd>obj.mapX) = obj.mapX - 1;
                yRayEnd(yRayEnd>obj.mapY) = obj.mapY - 1;
                xRayEnd(xRayEnd<=1) = 1;
                yRayEnd(yRayEnd<=1) = 1;
                for k = 1:obj.range
                    visibility(k) = obj.occMap(yRayEnd(k), xRayEnd(k));
                end
                beamLength = obj.range - sum(visibility);
                obj.idealMeas(i) = beamLength;
            end
        end % end readIdealLaser method
        function visualizeSensor(obj, pose, plotBeams)
            % plots occupancy map, sensor pose, laser beams
            % [input] pose: [x,y,theta] pose of sensor
            % [input] plotBeams: logical, if false, doesn't plot individual
            %                    beams
            
            % First, make a custom triangle marker showing current sensor pose
            markerSize = 25;
            markerAngle = pi/9;
            markerX = [pose(1);
                       pose(1) + markerSize*cos(pose(3) + pi - markerAngle);
                       pose(1) + markerSize*cos(pose(3) - pi + markerAngle);
                       pose(1)];
            markerY = [pose(2);
                       pose(2) + markerSize*sin(pose(3) + pi - markerAngle);
                       pose(2) + markerSize*sin(pose(3) - pi + markerAngle);
                       pose(2)];
            plot(markerX, markerY, 'b', 'LineWidth', 3)
            
            % Now, plot all the rays
            if plotBeams
                for i = 1:length(obj.sensorIndices)
                    x2 = pose(1) + obj.idealMeas(i)*cos(-pi/2 + pose(3)...
                        + deg2rad(obj.sensorIndices(i)+1));
                    y2 = pose(2) + obj.idealMeas(i)*sin(-pi/2 + pose(3)...
                        + deg2rad(obj.sensorIndices(i)+1));
                    plot([pose(1), x2], [pose(2), y2], 'r')
                end
            end
        end % end visualizeSensor
    end % end methods
end % end classdef