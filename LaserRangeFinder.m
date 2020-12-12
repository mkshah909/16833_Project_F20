% Idea Laser Class definition

classdef LaserRangeFinder < handle
    properties
        occMap % save occupancy map to save on read/write time
        range % [cm] max range of sensor
        rayVec % vector of all ray measurements, used for ray casting
        sigma % artificial sigma to add noise to sensor model
        idealMeas % [m1, m2, ..., mn] n laser measurements, at every 1 deg
        simMeas  % same as above, but with simulated noise
        mapX % x size of map computed once and saved for ray casting
        mapY % y size of map computed once and saved for ray casting
        rayAngles % vector of all angles measured by ray, 0deg starting from left
        rawXWorld % (n, len(rayAngles)) sized array. each row is ...
            % a scan, different columns are x readings in world co-ordinates
            % corresponding to ray angles
        rawYWorld
        rawXLaser % same map as above, but in laser co-ordinates
        rawYLaser
        featureInterval % number of neighboring readings to look at on each
            % side to compute smoothness for each point
        savedXEdge % world coordinates
        savedYEdge
        savedXPlane
        savedYPlane
        mapNormals
        currentXEdge % world coordinates
        currentYEdge
        currentXPlane
        currentYPlane
        currentNormals
        currentRawSmoothness
        resolution % scalar, number of readings between 0-179 deg
        
    end % end properties
    methods
        function laser = LaserRangeFinder(range, occMap, sigma, resolution, span, featureInterval)
            laser.range = range;
            laser.occMap = occMap;
            laser.rayVec = linspace(1, range, range);
            laser.sigma = sigma;
            [laser.mapY, laser.mapX] = size(occMap);
            laser.featureInterval = featureInterval;
            laser.resolution = resolution;
            laser.rayAngles = linspace(90 - span/2, 90 + span/2, resolution);
        end % end init
        
        
        function readIdealLaser(obj, pose, span)
            % Ray casting to generate ideal laser measurements, given laser pose
            % [input] pose: [x,y,theta] of laser in world frame
            % [input] span: scalar degrees, span of laser scan
            % modifies obj.laserMeas: (resolution, 1) vectory of laser range readings

            resolution = obj.resolution;
            obj.idealMeas = zeros(resolution,1);
            obj.simMeas = zeros(resolution,1);
            visibility = zeros(1, obj.range);
            rayAngles = obj.rayAngles;

            for i = 1:resolution
                % computes all possible x,y of current ray up to max range
                xRayEnd = pose(1) + obj.rayVec*cos(-pi/2 + pose(3)...
                    + deg2rad(rayAngles(i)+1));
                yRayEnd = pose(2) + obj.rayVec*sin(-pi/2 + pose(3)...
                    + deg2rad(rayAngles(i)+1));
                % round to match integer pixel values
                xRayEnd = round(xRayEnd);
                yRayEnd = round(yRayEnd);
                % enforce min max map limits
                xRayEnd(xRayEnd>obj.mapX) = obj.mapX - 1;
                yRayEnd(yRayEnd>obj.mapY) = obj.mapY - 1;
                xRayEnd(xRayEnd<=1) = 1;
                yRayEnd(yRayEnd<=1) = 1;
                hitWall = false;
                
                for k = 1:obj.range
                    occMapPixel = obj.occMap(yRayEnd(k), xRayEnd(k));
                    if occMapPixel > 0
                        hitWall = true;
                    end
                    if hitWall
                        visibility(k:end) = 1;
                        break
                    end
                end
%                 for k = 1:obj.range
%                     visibility(k) = obj.occMap(yRayEnd(k), xRayEnd(k));
%                 end
                beamLength = obj.range - sum(visibility);
                obj.idealMeas(i) = beamLength;
                obj.simMeas(i) = normrnd(beamLength, obj.sigma);
                visibility = zeros(obj.range, 1);
            end
        end % end readIdealLaser method
        
        
        function worldPoints = getWorldPointsFromBeam(obj, pose, isNoisy)
            % convert raw beam measurements to world co-ordinates
            % [input] pose: [x,y,theta] pose of sensor
            % [input] plotBeams: logical, if false, doesn't plot individual
            %                    beams
            % [input] isNoisy: logical, if true, use simMeas, not idealMeas
            % [output] worldPoints: [x; y; z] homogenized points, 3xN size
            scanX = [];
            scanY = [];
            for i = 1:length(obj.rayAngles)
                if isNoisy
                    singleMeas = obj.simMeas(i);
                else
                    singleMeas = obj.idealMeas(i);
                end
                x2 = pose(1) + singleMeas*cos(-pi/2 + pose(3)...
                    + deg2rad(obj.rayAngles(i)));
                y2 = pose(2) + singleMeas*sin(-pi/2 + pose(3)...
                    + deg2rad(obj.rayAngles(i)));
                scanX = [scanX, x2];
                scanY = [scanY, y2];
            end
            worldPoints = [scanX;
                           scanY;
                           zeros(1, length(obj.rayAngles))];
        end % end getPointsFromBeam method
        
        
        function laserPoints = getLaserPointsFromBeam(obj, isNoisy)
            % convert raw beam measurements to world co-ordinates
            % [input] plotBeams: logical, if false, doesn't plot individual
            %                    beams
            % [input] isNoisy: logical, if true, use simMeas, not idealMeas
            % [output] worldPoints: [x; y; z] homogenized points, 3xN size
            scanX = [];
            scanY = [];
            for i = 1:obj.resolution
                if isNoisy
                    singleMeas = obj.simMeas(i);
                else
                    singleMeas = obj.idealMeas(i);
                end
                x2 = 0 + singleMeas*cos(-pi/2 + deg2rad(obj.rayAngles(i)+1));
                y2 = 0 + singleMeas*sin(-pi/2 + deg2rad(obj.rayAngles(i)+1));
                scanX = [scanX, x2];
                scanY = [scanY, y2];
            end
            laserPoints = [scanX;
                           scanY;
                           zeros(1, length(obj.rayAngles))];
        end % end getLaserPointsFromBeam method
        
        
        function visualizeSensor(obj, pose, worldPoints, plotBeams)
            % plots occupancy map, sensor pose, laser beams, also adds
            %   noise
            % [input] pose: [x,y,theta] pose of sensor
            % [input] plotBeams: logical, if false, doesn't plot individual
            %                    beams
            % [input] worldPoints: 3xn homogenized world points
            % [input] isNoisy: logical, if true, use simMeas, not idealMeas
            
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
            
            if plotBeams
                for i = 1:length(obj.rayAngles)
                    x2 = worldPoints(1,i);
                    y2 = worldPoints(2,i);
                    plot([pose(1), x2], [pose(2), y2], 'r')
                end
            end
        end % end visualizeSensor
        
        
        function updateSmoothness(obj, planeThres, edgeThres)
            % assign smoothness value to each measured point
            % save points in relevant edge/plane list
            % [input] planeThres: scalar, smoothness>planeThres -> planar
            % [input] edgeThres: scalar, smoothness<edgeThres -> edge
            interval = obj.featureInterval;
            smoothnessVals = NaN(1,obj.resolution);
            obj.currentNormals = zeros(2, obj.resolution);
            
            for i = interval+1:obj.resolution-interval
                % gather neighboring points
                xSubset = obj.rawXWorld(end, i-interval:i-1);
                xSubset = [xSubset, obj.rawXWorld(end, i+1:i+interval)];
                ySubset = obj.rawYWorld(end, i-interval:i-1);
                ySubset = [ySubset, obj.rawYWorld(end, i+1:i+interval)];
                currX = obj.rawXWorld(end, i);
                currY = obj.rawYWorld(end, i);
                % compute smoothness, formulated in LOAM paper
%                 den = interval*2*norm([currX, currY]);
%                 smoothness = sum(sqrt((currX - xSubset).^2 + (currY-ySubset).^2))./den;
                vectors = [currX-xSubset(1), currX-xSubset(end);
                           currY-ySubset(1), currY-ySubset(end)];
                vectors = vectors./norm(vectors,1); % normalize to unit vector
                smoothness = abs(sum(vectors(:,1).*vectors(:,2))); % dot product
                smoothnessVals(i) = smoothness;
                % compute average normal from all points (edges will get
                % filtered later)
                normal = [-currY+ySubset(1:interval), -ySubset(interval+1:end)+currY;
                          currX-xSubset(1:interval), xSubset(interval+1:end)-currX];
                normal = mean(normal,2);
                normal = normal./norm(normal,2);
                obj.currentNormals(:,i) = normal;
            end % for
            % obtain indices based on thresholds
            planeInd = smoothnessVals>planeThres;
            edgeInd = smoothnessVals<edgeThres;
            obj.currentXEdge = obj.rawXWorld(end, edgeInd);
            obj.currentYEdge = obj.rawYWorld(end, edgeInd);
            obj.currentXPlane = obj.rawXWorld(end, planeInd);
            obj.currentYPlane = obj.rawYWorld(end, planeInd);
            obj.currentNormals = obj.currentNormals(:, planeInd);
            obj.currentRawSmoothness = smoothnessVals;
        end % end updateSmoothness function
        
        
    end % end methods
end % end classdef