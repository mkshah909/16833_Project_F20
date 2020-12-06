function pose = tform2pose(tform)
% returns [x,y,z,theta] 2d pose (z always 0) from 2d rigid body transform

tformMat = tform.T';

tformMat(:, 4) = tformMat(:, 4)./tformMat(4,4); % normalize, just in case needed

theta = atan2(tformMat(2,1), tformMat(1,1));

pose = [tformMat(1,4); tformMat(2,4); theta];

end